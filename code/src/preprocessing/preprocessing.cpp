#include <binary_set.hxx>

#include "cli_descriptions.hpp"
#include "solver.hpp"

// TODO: Can we parallelize the preprocessing?
// TODO: Is there a way to know which action is achieving a fixed fact? If so, can we fix those as well (actions and first achievers)?
void Solver::preprocess_() {
    const auto& choices = params_.get<cli_desc::preprocess, std::string>();
    const bool use_l = choices.find('l') != std::string::npos;
    const bool use_a = choices.find('a') != std::string::npos;
    const bool use_f = choices.find('f') != std::string::npos;
    const bool use_b = choices.find('b') != std::string::npos;
    const bool use_d = choices.find('d') != std::string::npos;
    const bool use_o = choices.find('o') != std::string::npos;
    const bool use_i = choices.find('i') != std::string::npos;

    // Kept across passes (remapped on fact elimination) so a new landmark computation can be compared against the previous one
    std::vector<std::vector<unsigned int>> landmarks(inst_.n);

    global_.fixed_facts = BinarySet{inst_.n};
    global_.fixed_actions = BinarySet{inst_.m};

    // Simplifications enable one another (e.g. removing an achiever can only grow landmark sets, which in turn enables new simplifications), so
    // the steps are repeated until convergence. Each step, however, is idempotent on unchanged inputs, so it must be repeated only when its
    // inputs changed since it last ran:
    //  - l: an achiever disappeared, i.e. an (action, effect) pair was removed (deleted by 'a' or gone with an eliminated fact/action), or the goal
    //       shrank ('i') -> all tracked by nfadd, which drops on every such event: effect-less actions cannot affect landmarks, and the only step
    //       that shrinks the preconditions of a surviving action ('i', whose eliminated facts are exactly the ones its prefix reached) always
    //       eliminates the prefix actions that added them
    //  - a: a landmark set changed (across runs they can only grow); erasability of an effect depends only on landmarks and preconditions, and
    //       shrinking a precondition set can only shrink the landmark union, i.e. erase fewer effects
    //  - b: an effect entry was deleted ('a'), an action was eliminated after 'b' ran ('d' or 'i'), or the goal shrank ('i'); eliminating the
    //       facts/actions 'b' itself found never changes relevance of the survivors (they were unreachable in the backward exploration), and a
    //       growing fixed_facts/fixed_actions can only protect more facts/actions from elimination
    //  - d: a landmark set changed or a pre/eff set shrank ('a', or any fact elimination — including the one 'i' performs, which can collapse
    //       distinct actions into duplicates); eliminating actions cannot create new dominations (the domination condition is pairwise and
    //       transitive) and newly fixed actions can never be dominated (they are excluded from the candidates outright)
    //  - i: the fixed actions may have changed ('l' re-ran: achievers shrank, so actions can become fixed even with unchanged landmarks; or 'o'
    //       fixed one) or preconditions shrank (facts eliminated by 'b'), possibly enabling new fixed/0-cost applications; eliminating actions
    //       alone can never enable an application (it never satisfies a precondition), and 'i' saturates internally so it never re-arms itself
    //  - o: fixes at most one action per pass (fixing one changes the symmetry graph); a successful fix re-arms itself so the loop keeps running
    //       until no further action can be fixed, and also re-arms 'i' (the newly fixed action's preconditions become fixed facts). It is not
    //       gated on a pending flag: it runs on every pass, since any change to the instance reshapes the symmetry graph
    bool pending_a = true;
    bool pending_b = true;
    bool pending_d = true;
    bool pending_i = true;
    bool pending_o = true;
    unsigned int nfadd_at_last_l = inst_.nfadd + 1;  // makes 'l' pending on the first pass

    // Convergence tripwire: every pass beyond the first must be caused by a change, and each change strictly shrinks n, m or nfadd (or grows a
    // landmark set, which re-arms at most one extra settling pass, or grows fixed_actions via 'o', at most m times) — a generous linear bound
    // catches any future non-terminating trigger
    const unsigned int max_passes = (2 * (inst_.n + inst_.m + inst_.nfadd)) + 3;
    unsigned int passes{0};

    while ((use_l && nfadd_at_last_l != inst_.nfadd) || (use_a && pending_a) || (use_b && pending_b) || (use_d && pending_d) ||
           (use_i && pending_i) || (use_o && pending_o)) {
        myassert(++passes <= max_passes, "Preprocessing loop did not converge");
        global_.eliminated_facts = BinarySet{inst_.n};
        global_.eliminated_actions = BinarySet{inst_.m};

        if (use_l && nfadd_at_last_l != inst_.nfadd) {
            nfadd_at_last_l = inst_.nfadd;
            pending_i = true;  // fixed actions are rebuilt on every 'l' run and can grow even when the landmark sets are unchanged
            if (prep_fact_landmarks_(landmarks)) {
                pending_a = true;
                pending_d = true;
            }
        }

        if (use_a && pending_a) {
            pending_a = false;
            if (prep_first_adders_(landmarks)) {
                pending_b = true;
                pending_d = true;
            }
        }

        if (use_b && pending_b) {
            pending_b = false;
            BinarySet relevant_variables(inst_.n);
            prep_relevance_backward_(relevant_variables);
            // 'f' needs the relevant variables computed by 'b' in this same pass: with none computed, every action would look irrelevant
            if (use_f) {
                prep_relevance_forward_(relevant_variables);
            }
        }

        if (!global_.eliminated_facts.empty()) {
            prep_eliminated_facts_(landmarks);
            pending_d = true;  // effect sets of surviving actions may have shrunk
            pending_i = true;  // precondition sets shrank too: fixed/0-cost actions may have become applicable
        }

        if (use_d && pending_d) {
            pending_d = false;
            if (prep_dominated_actions_(landmarks)) {
                pending_b = true;  // facts that were relevant only as preconditions of dominated actions are now irrelevant
            }
        }

        prep_eliminated_actions_();

        prep_fixed_actions_();  // Fixes all preconditions and effects of fixed actions

        if (use_o) {
            pending_o = prep_orbital_probing_();
            pending_i |= pending_o;  // the newly fixed action's preconditions become fixed facts, possibly enabling new fixed/0-cost applications
        }

        if (use_i && pending_i) {
            pending_i = false;
            const bool i_changed = prep_initial_action_sequencing_(landmarks);  // also recounts nfadd (the repetition trigger for 'l')
            // 'i' eliminates the prefix actions and every fact they reached. Unlike the other elimination steps it does so on its own, past the
            // shared elimination block above, so it has to re-arm the same steps that block re-arms (and 'b', since it also drops actions and can
            // shrink the goal): the survivors come out with smaller pre/eff sets, which can turn previously distinct actions into duplicates ('d')
            // and previously relevant facts/actions into unreachable ones ('b')
            pending_o |= i_changed;
            pending_d |= i_changed;
            pending_b |= i_changed;
            // If we reach the goal state with the prefix, the goal is set to empty without changing any other data structure -> this is fine ONLY
            // because the check immediatelly after this line terminates the execution with an optimality proof and only the prefix is considered
            // (empty rest of the solution since the goal is already reached)...
        }

        // Consistency tripwires: every step must leave the instance counters in sync with the actual containers
        myassert(inst_.m == inst_.actions.size() && inst_.m == inst_.actions_names.size() && inst_.m == global_.fixed_actions.capacity(),
                 "Action count out of sync after a preprocessing pass");
        myassert(inst_.n == inst_.goal.capacity() && inst_.n == landmarks.size() && inst_.n == global_.fixed_facts.capacity(),
                 "Fact count out of sync after a preprocessing pass");
        myassert(
            [this]() -> bool {
                unsigned int nfadd_check{0};
                for (const auto& act : inst_.actions) {
                    nfadd_check += act.eff_sparse.size();
                }

                return inst_.nfadd == nfadd_check;
            }(),
            "nfadd out of sync with the effect entries after a preprocessing pass");

        if (inst_.goal.empty()) {  // Goal reached during preprocessing
            // ignoring the prefix, the optimal a 0-cost solution, so we need to update both lower bound and incumbent to 0
            global_.best_bound = 0;
            global_.best_incumbent = 0;
            global_.solution = {};
            throw EarlyExit("preprocessing", EarlyExit::OPTIMAL);
        }
    }

    logger_[DEBUG] << std::format("Computed a prefix of cost {}", global_.cost_prefix);
    logger_[DEBUG] << std::format("Size of the prefix: {}", global_.solution_prefix.size());
}

auto Solver::prep_fact_landmarks_(std::vector<std::vector<unsigned int>>& landmarks_ret) -> bool {
    // use the last bit as a flag to tell the BinarySet is full
    std::vector<BinarySet> landmarks(inst_.n, BinarySet{inst_.n + 1});
    for (unsigned int fact = 0; fact < inst_.n; fact++) {
        landmarks[fact].add(inst_.n);
    }

    BinarySet s_set{inst_.n};

    // add to the queue all initial actions (those with no preconditions)
    std::deque<unsigned int> actions_queue;
    BinarySet acts_in_queue{inst_.m};
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (inst_.actions[act_i].pre_sparse.empty()) {
            actions_queue.push_back(act_i);
            acts_in_queue.add(act_i);
        }
    }

    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto& pre : inst_.actions[act_i].pre_sparse) {
            act_with_pre[pre].push_back(act_i);
        }
    }

    // Watch preconditions for O(1) applicability checks against s_set.
    // watch_s[act] = one precondition of act not yet in s_set (inst_.n = sentinel: action is applicable)
    // watching_s[fact] = actions that are currently watching 'fact'
    // applicable[act] = true once all preconditions of act are in s_set
    std::vector<unsigned int> watch_s(inst_.m, inst_.n);
    std::vector<std::vector<unsigned int>> watching_s(inst_.n);
    BinarySet applicable{inst_.m};
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (inst_.actions[act_i].pre_sparse.empty()) {
            applicable.add(act_i);
        } else {
            // s_set is empty at init — first precondition is always unsatisfied
            const unsigned int watched = inst_.actions[act_i].pre_sparse[0];
            watch_s[act_i] = watched;
            watching_s[watched].push_back(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const auto act_i = actions_queue.front();
        const auto& act = inst_.actions[act_i];
        actions_queue.pop_front();
        acts_in_queue.remove(act_i);

        BinarySet x_a{inst_.n + 1};
        for (const auto& eff : act.eff_sparse) {
            x_a.add(eff);
        }

        // Compute the union of precondition landmarks once for all effects of this action.
        BinarySet x_union{x_a};
        bool x_union_full = false;
        for (const auto& pre : act.pre_sparse) {
            // if variable pre has the "full" flag then the union generates a "full" bitfield
            if (landmarks[pre][inst_.n]) {
                x_union_full = true;
                break;
            }
            x_union |= landmarks[pre];
        }

        for (const auto& eff : act.eff_sparse) {
            s_set.add(eff);

            // Update watch preconditions for actions watching this newly reached fact.
            // Swap-erase pattern: we always remove act_j from watching_s[eff] since eff is
            // now in s_set and should never be watched again.
            for (size_t wi = 0; wi < watching_s[eff].size();) {
                const unsigned int act_j = watching_s[eff][wi];
                bool moved = false;
                for (const auto& pre : inst_.actions[act_j].pre_sparse) {
                    if (!s_set[pre]) {
                        watch_s[act_j] = pre;
                        watching_s[pre].push_back(act_j);
                        moved = true;
                        break;
                    }
                }
                watching_s[eff][wi] = watching_s[eff].back();
                watching_s[eff].pop_back();
                if (!moved) {
                    applicable.add(act_j);
                    watch_s[act_j] = inst_.n;  // sentinel: no unsatisfied precondition
                }
            }

            // if x_union is already full, intersection with landmarks[eff] cannot shrink it —
            // no useful landmark update is possible for this effect
            if (x_union_full) {
                continue;
            }

            BinarySet lm_new{x_union};

            // if the set for variable eff is the full set of variables, the intersection
            // generates back lm_new -> we can skip the intersection
            if (!landmarks[eff][inst_.n]) {
                lm_new &= landmarks[eff];
            }

            // we already know that lm_new is not the full set now, so if the set for variable
            // eff is the full set, we know that lm_new is not equal to the set for variable eff
            if (!landmarks[eff][inst_.n] && lm_new == landmarks[eff]) {
                continue;
            }

            landmarks[eff] = lm_new;
            for (const auto& act_j : act_with_pre[eff]) {
                if (applicable[act_j] && !acts_in_queue[act_j]) {
                    actions_queue.push_back(act_j);
                    acts_in_queue.add(act_j);
                }
            }
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing landmark extraction (main loop)", EarlyExit::TIMELIMIT);
        }
    }

    // list of fact landmarks for each variable
    std::vector<std::vector<unsigned int>> new_landmarks(inst_.n);
    for (unsigned int fact_p = 0; fact_p < inst_.n; fact_p++) {
        for (unsigned int fact_q = 0; fact_q < inst_.n; fact_q++) {
            if (landmarks[fact_p][fact_q] || landmarks[fact_p][inst_.n]) {
                new_landmarks[fact_p].push_back(fact_q);
            }
        }
    }

    // list of actions that have as effect variable p
    std::vector<std::vector<unsigned int>> act_with_eff(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto& eff : inst_.actions[act_i].eff_sparse) {
            act_with_eff[eff].push_back(act_i);
        }
    }

    // Rebuilt from scratch on every run: landmark sets only grow and achiever sets only shrink between two runs, so a fresh computation always
    // re-derives (a superset of) the previous run's fixed sets
    global_.fixed_facts.clear();
    global_.fixed_actions.clear();

    for (const auto g_fact : inst_.goal) {
        for (const auto& g_factlm : new_landmarks[g_fact]) {
            if (global_.fixed_facts[g_factlm]) {
                continue;
            }
            global_.fixed_facts.add(g_factlm);
            unsigned int count{0};
            unsigned int cand_act = 0;
            for (const auto& act_i : act_with_eff[g_factlm]) {
                cand_act = act_i;
                count++;
                if (count > 1) {
                    break;
                }
            }
            if (count == 1) {
                global_.fixed_actions.add(cand_act);
            }
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing landmark extraction (goal loop)", EarlyExit::TIMELIMIT);
        }
    }

    const bool changed = new_landmarks != landmarks_ret;
    landmarks_ret = std::move(new_landmarks);
    return changed;
}

auto Solver::prep_first_adders_(std::vector<std::vector<unsigned int>>& landmarks) -> bool {
    BinarySet fact_lm_for_a(inst_.n);
    unsigned int erased{0};

    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        fact_lm_for_a.clear();
        for (const auto& pre : inst_.actions[act_i].pre_sparse) {
            for (const auto& fact_q : landmarks[pre]) {
                fact_lm_for_a.add(fact_q);
            }
        }
        erased += static_cast<unsigned int>(
            std::erase_if(inst_.actions[act_i].eff_sparse, [&fact_lm_for_a](const auto val) { return fact_lm_for_a[val]; }));

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing first adders extraction", EarlyExit::TIMELIMIT);
        }
    }
    inst_.nfadd -= erased;
    return erased > 0;
}

void Solver::prep_relevance_backward_(BinarySet& relevant_variables) {
    BinarySet relevant_actions(inst_.m);
    std::queue<unsigned int> relevant_facts_queue;

    // compute first round of relevand variables and actions
    for (const auto& g_fact : inst_.goal) {
        relevant_variables.add(g_fact);
        relevant_facts_queue.push(g_fact);
    }

    std::vector<std::vector<unsigned int>> act_with_eff(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto eff : inst_.actions[act_i].eff_sparse) {
            act_with_eff[eff].push_back(act_i);
        }
    }

    while (!relevant_facts_queue.empty()) {
        const auto fact = relevant_facts_queue.front();
        relevant_facts_queue.pop();

        for (const auto& act_i : act_with_eff[fact]) {
            if (relevant_actions[act_i]) {
                continue;
            }
            relevant_actions.add(act_i);
            for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                // relevant_variables is empty at the beginning of this function, so this is equivalent to a check of "pre" already being in the
                // queue
                if (relevant_variables[pre]) {
                    continue;
                }
                relevant_variables.add(pre);
                relevant_facts_queue.push(pre);
            }
        }

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing backwards relevance analysis", EarlyExit::TIMELIMIT);
        }
    }

    // eliminate actions and variables that are not relevant (or landmarks)
    global_.eliminated_facts |= !(relevant_variables | global_.fixed_facts);  // eliminating variables will be done at once, later
    global_.eliminated_actions |= !relevant_actions;                          // eliminating actions will be done at once, later

    // Eliminate non-goal facts that aren't precondition of any action
    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (global_.eliminated_actions[act_i]) {
            continue;
        }
        for (const auto& pre : inst_.actions[act_i].pre_sparse) {
            if (global_.eliminated_facts[pre]) {
                continue;
            }
            act_with_pre[pre].push_back(act_i);
        }
    }

    for (unsigned int i = 0; i < inst_.n; i++) {
        if (!inst_.goal[i] && act_with_pre[i].empty()) {
            global_.eliminated_facts.add(i);
        }
    }
}

void Solver::prep_relevance_forward_(BinarySet& relevant_variables) {
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (global_.eliminated_actions[act_i] || global_.fixed_actions[act_i]) {
            continue;
        }
        if (!bs_intersects(relevant_variables, inst_.actions[act_i].eff_sparse)) {
            global_.eliminated_actions.add(act_i);
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing forward relevance analysis", EarlyExit::TIMELIMIT);
        }
    }
}

auto Solver::prep_dominated_actions_(std::vector<std::vector<unsigned int>>& landmarks) -> bool {
    const auto& rem_act{(!global_.eliminated_actions).sparse()};

    std::vector<BinarySet> act_flm(inst_.m, BinarySet(inst_.n));
    for (const auto& act_i : rem_act) {
        for (const auto& var_i : inst_.actions[act_i].pre_sparse) {
            for (const auto& fact : landmarks[var_i]) {
                act_flm[act_i].add(fact);
            }
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing dominated actions removal (first loop)", EarlyExit::TIMELIMIT);
        }
    }

    bs_searcher candidates{inst_.n};
    std::vector<BinarySet> actions_effects(inst_.m, BinarySet(inst_.n));
    for (const auto& act_i : rem_act) {
        for (const auto& val : inst_.actions[act_i].eff_sparse) {
            actions_effects[act_i].add(val);
        }
        if (global_.fixed_actions[act_i]) {
            continue;
        }
        candidates.add(act_i, actions_effects[act_i]);
    }

    BinarySet dominated_actions{inst_.m};

    for (const auto& dominant_act : rem_act) {
        if (dominated_actions[dominant_act]) {
            continue;
        }

        for (const auto& dominated_act : candidates.find_subsets(actions_effects[dominant_act])) {
            if (dominant_act == dominated_act || inst_.actions[dominant_act].cost > inst_.actions[dominated_act].cost ||
                !bs_contains(act_flm[dominated_act], inst_.actions[dominant_act].pre_sparse)) {
                continue;
            }

            dominated_actions.add(dominated_act);
            global_.eliminated_actions.add(dominated_act);
            candidates.remove(dominated_act, actions_effects[dominated_act]);
        }

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing dominated actions removal (main loop)", EarlyExit::TIMELIMIT);
        }
    }
    return !dominated_actions.empty();
}

void Solver::prep_eliminated_facts_(std::vector<std::vector<unsigned int>>& landmarks) {
    if (global_.eliminated_facts.empty()) {
        return;
    }
    myassert(global_.eliminated_facts.capacity() == inst_.n, "eliminated_facts capacity out of sync with the fact count");
    myassert(landmarks.size() == inst_.n, "landmarks out of sync with the fact count");

    unsigned int removed{static_cast<unsigned int>(global_.eliminated_facts.sparse().size())};
    myassert(removed < inst_.n, "Attempted to eliminate every fact (the caller must handle this case beforehand)");

    std::vector<unsigned int> removed_offsets(inst_.n, 0);
    unsigned int current_removed = 0;
    for (unsigned int i = 0; i < inst_.n; i++) {
        removed_offsets[i] = current_removed;
        if (global_.eliminated_facts[i]) {
            ++current_removed;
        }
    }

    inst_.nfadd = 0;
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        std::vector<unsigned int> new_pre;
        std::vector<unsigned int> new_eff;
        new_pre.reserve(inst_.actions[act_i].pre_sparse.size());
        new_eff.reserve(inst_.actions[act_i].eff_sparse.size());
        for (const auto& pre : inst_.actions[act_i].pre_sparse) {
            if (!global_.eliminated_facts[pre]) {
                new_pre.push_back(pre - removed_offsets[pre]);
            }
        }
        for (const auto& eff : inst_.actions[act_i].eff_sparse) {
            if (!global_.eliminated_facts[eff]) {
                new_eff.push_back(eff - removed_offsets[eff]);
            }
        }
        inst_.actions[act_i].pre_sparse = std::move(new_pre);
        inst_.actions[act_i].eff_sparse = std::move(new_eff);
        inst_.nfadd += inst_.actions[act_i].eff_sparse.size();
    }

    // Adjust goal
    BinarySet new_goal(inst_.n - removed);
    for (const auto gfact : inst_.goal) {
        if (!global_.eliminated_facts[gfact]) {
            new_goal.add(gfact - removed_offsets[gfact]);
        }
    }
    inst_.goal = new_goal;

    // Adjust landmarks
    unsigned int counter{0};
    for (const auto fact : !global_.eliminated_facts) {
        std::vector<unsigned int> copy(landmarks[fact]);
        landmarks[counter].clear();
        for (const auto& factlm : copy) {
            if (global_.eliminated_facts[factlm]) {
                continue;
            }
            landmarks[counter].push_back(factlm - removed_offsets[factlm]);
        }
        counter++;
    }
    landmarks.resize(counter);

    // Adjust fixed_facts
    BinarySet new_fixed_facts(inst_.n - removed);
    for (const auto fact : global_.fixed_facts) {
        if (!global_.eliminated_facts[fact]) {
            new_fixed_facts.add(fact - removed_offsets[fact]);
        }
    }
    global_.fixed_facts = new_fixed_facts;

    inst_.n -= removed;
}

void Solver::prep_eliminated_actions_() {
    if (global_.eliminated_actions.empty()) {
        return;
    }
    myassert(global_.eliminated_actions.capacity() == inst_.m, "eliminated_actions capacity out of sync with the action count");
    myassert(global_.eliminated_actions.size() < inst_.m, "Attempted to eliminate every action (the caller must handle this case beforehand)");

    BinarySet new_fixed_actions(static_cast<unsigned int>(inst_.m - global_.eliminated_actions.size()));
    unsigned int write_pos = 0;
    for (unsigned int read_pos = 0; read_pos < inst_.actions.size(); ++read_pos) {
        if (global_.eliminated_actions[read_pos]) {
            continue;
        }

        if (write_pos != read_pos) {
            // Remove eliminated actions
            inst_.actions[write_pos] = std::move(inst_.actions[read_pos]);
            // Remove eliminated action names
            inst_.actions_names[write_pos] = std::move(inst_.actions_names[read_pos]);
        }

        // Adjust positions of actions in fixed_actions
        if (global_.fixed_actions[read_pos]) {
            new_fixed_actions.add(write_pos);
        }
        ++write_pos;
    }
    inst_.actions.resize(write_pos);
    inst_.actions_names.resize(write_pos);
    global_.fixed_actions = new_fixed_actions;

    inst_.m = write_pos;

    // Eliminated actions took their effect entries with them: recount the first adders
    unsigned int count{0};
    for (const auto& act : inst_.actions) {
        count += act.eff_sparse.size();
    }
    inst_.nfadd = count;
}

void Solver::prep_fixed_actions_() {
    for (const auto act_i : global_.fixed_actions) {
        global_.fixed_facts |= inst_.actions[act_i].pre_sparse;
        global_.fixed_facts |= inst_.actions[act_i].eff_sparse;
    }
}

void Solver::prep_setup_helpers_() {
    global_.hplus_fadd_cpx_start.clear();
    global_.hplus_fadd_cpx_start.reserve(inst_.m);
    unsigned int sum{0};
    for (const auto& act : inst_.actions) {
        global_.hplus_fadd_cpx_start.push_back(sum);
        sum += act.eff_sparse.size();
    }
    global_.act_with_pre = std::vector<std::vector<unsigned int>>(inst_.n);
    global_.act_with_eff = std::vector<std::vector<unsigned int>>(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; ++act_i) {
        for (const auto& pre_i : inst_.actions[act_i].pre_sparse) {
            global_.act_with_pre[pre_i].push_back(act_i);
        }
        for (const auto& eff_i : inst_.actions[act_i].eff_sparse) {
            global_.act_with_eff[eff_i].push_back(act_i);
        }
    }

    global_.eliminated_facts = BinarySet{1};
    global_.eliminated_actions = BinarySet{1};

    // When preprocessing is skipped these are never initialized; keep them valid (empty) here
    if (global_.fixed_facts.capacity() == 0) {
        global_.fixed_facts = BinarySet{inst_.n};
    }
    if (global_.fixed_actions.capacity() == 0) {
        global_.fixed_actions = BinarySet{inst_.m};
    }

    // Since we removed initial facts, the initial state is empty, so initial actions are those with no preconditions
    global_.initial_actions.clear();
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (inst_.actions[act_i].pre_sparse.empty()) {
            global_.initial_actions.push_back(act_i);
        }
    }

    global_.goal_sparse = inst_.goal.sparse();
}
