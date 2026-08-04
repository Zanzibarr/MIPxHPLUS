#include <binary_set.hxx>

#include "cli_descriptions.hpp"
#include "solver.hpp"

// TODO: Can we parallelize the preprocessing?
void Solver::preprocess_() {
    const auto& choices = params_.get<cli_desc::preprocess, std::string>();
    const bool use_l = choices.find('l') != std::string::npos;
    const bool use_a = choices.find('a') != std::string::npos;
    const bool use_f = choices.find('f') != std::string::npos;
    const bool use_b = choices.find('b') != std::string::npos;
    const bool use_d = choices.find('d') != std::string::npos;
    const bool use_symm = choices.find('o') != std::string::npos || choices.find('s') != std::string::npos;
    const bool use_i = choices.find('i') != std::string::npos;

    // Kept across passes (remapped on fact elimination) so a new landmark computation can be compared against the previous one
    std::vector<std::vector<unsigned int>> landmarks(inst_.n);

    global_.fixed_facts = BinarySet{inst_.n};
    global_.fixed_actions = BinarySet{inst_.m};
    // The steps never eliminate anything themselves, they only mark: the shared block at the end of each pass performs the elimination and leaves
    // both sets empty again, so every step sees them holding exactly what the pass has marked so far
    global_.eliminated_facts = BinarySet{inst_.n};
    global_.eliminated_actions = BinarySet{inst_.m};

    // Simplifications enable one another (e.g. removing an achiever can only grow landmark sets, which in turn enables new simplifications), so the
    // steps are repeated until convergence. Each one is idempotent on unchanged inputs, so it re-runs only when its own inputs changed:
    //  - l: an achiever disappeared or the goal shrank, both tracked by nfadd
    //  - a: a landmark set changed
    //  - b: an effect entry was deleted, an action was eliminated after 'b' ran, or the goal shrank
    //  - d: a landmark set changed, or a fact was marked, by 'b' (it must then run before 'o', which reasons on the surviving facts too) or by 'o'
    //  - i: the fixed actions changed, or a precondition set shrank
    //  - o: ungated, since any change to the instance reshapes the symmetry graph; it fixes at most one action per pass and re-arms itself
    bool pending_a = true;
    bool pending_b = true;
    bool pending_d = true;
    bool pending_i = true;
    bool pending_symm = true;
    unsigned int nfadd_at_last_l = inst_.nfadd + 1;  // makes 'l' pending on the first pass

    // Convergence tripwire: every pass beyond the first must be caused by a change, and each change strictly shrinks n, m or nfadd (or grows a
    // landmark set, which re-arms at most one extra settling pass, or grows fixed_actions via 'o', at most m times) — a generous linear bound
    // catches any future non-terminating trigger
    const unsigned int max_passes = (2 * (inst_.n + inst_.m + inst_.nfadd)) + 3;
    unsigned int passes{0};

    while ((use_l && nfadd_at_last_l != inst_.nfadd) || (use_a && pending_a) || (use_b && pending_b) || (use_d && pending_d) ||
           (use_i && pending_i) || (use_symm && pending_symm)) {
        static int counter = 0;
        logger_[DEBUG] << std::format("======= PREPROCESSING ITERATION {} ========", counter++);

        myassert(++passes <= max_passes, "Preprocessing loop did not converge");

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

        // What 'b' marked shrinks the pre/eff sets of every survivor, so 'd' and 'i' must reconsider the instance in this very pass: 'o' reasons on
        // the surviving facts too, and two actions differing only on facts 'b' just marked reach it as an only-action generator unless 'd' ran first
        if (!global_.eliminated_facts.empty()) {
            pending_d = true;
            pending_i = true;
        }

        if (use_d && pending_d) {
            pending_d = false;
            if (prep_dominated_actions_(landmarks)) {
                pending_b = true;  // facts that were relevant only as preconditions of dominated actions are now irrelevant
            }
        }

        const unsigned int marks_before_o = global_.eliminated_facts.size();
        if (use_symm) {
            pending_symm = prep_symmetry_breaking_();
            pending_i |= pending_symm;  // the newly fixed action's preconditions become fixed facts, possibly enabling new fixed/0-cost applications
        }
        // An only-fact generator fixes every action, so the twin 'o' drops duplicates the one it keeps in every pre/eff set: no effect subset
        // relation moves, and 'i' (pure reachability over those sets) reaches both twins together, so it needs nothing. 'd' does: its other half
        // tests pre(dominant) against landmarks that 'l' computed at the top of the pass, on an instance the generator is not a symmetry of, so one
        // twin can sit in a landmark set without the other and dropping it can satisfy a containment that used to fail
        if (global_.eliminated_facts.size() > marks_before_o) {
            pending_d = true;
        }

        if (use_i && pending_i) {
            pending_i = false;
            const bool i_changed = prep_initial_action_sequencing_();
            // The prefix takes actions and facts with it, so the survivors come out with smaller pre/eff sets: previously distinct actions can become
            // duplicates ('d'), previously relevant facts and actions can become unreachable ('b'), and the instance reshapes the symmetry graph
            // ('o')
            pending_symm |= i_changed;
            pending_d |= i_changed;
            pending_b |= i_changed;
        }

        // ~~~~~~ Apply what the pass marked ~~~~~ //
        // Actions first: fact elimination then only rewrites the pre/eff sets of the survivors, and its nfadd recount is the one that stands
        prep_eliminated_actions_();
        prep_eliminated_facts_(landmarks);

        prep_fixed_actions_();  // fixes all preconditions and effects of the surviving fixed actions

        // Consistency tripwires: every step must leave the instance counters in sync with the actual containers
        myassert(inst_.m == inst_.actions.size() && inst_.m == inst_.actions_names.size() && inst_.m == global_.fixed_actions.capacity() &&
                     inst_.m == global_.eliminated_actions.capacity(),
                 "Action count out of sync after a preprocessing pass");
        myassert(inst_.n == inst_.goal.capacity() && inst_.n == landmarks.size() && inst_.n == global_.fixed_facts.capacity() &&
                     inst_.n == global_.eliminated_facts.capacity(),
                 "Fact count out of sync after a preprocessing pass");
        // Each eliminator resets its own marker set, and returns early exactly when the set is already empty (and still correctly sized, since it
        // eliminated nothing): either way neither set outlives the pass
        myassert(global_.eliminated_facts.empty() && global_.eliminated_actions.empty(), "A marker set outlived a preprocessing pass");
        myassert(
            [this]() -> bool {
                unsigned int nfadd_check{0};
                for (const auto& act : inst_.actions) {
                    nfadd_check += act.eff_sparse.size();
                }

                return inst_.nfadd == nfadd_check;
            }(),
            "nfadd out of sync with the effect entries after a preprocessing pass");

        const auto n_facts = inst_.n - (global_.fixed_facts | inst_.goal).size();
        const auto n_actions = inst_.m - global_.fixed_actions.size();
        const auto n_effects = inst_.nfadd;
        logger_[DEBUG] << std::format("Facts: {:>5} Actions: {:>5} Effects: {:>5}", n_facts, n_actions, n_effects);

        // Goal reached during preprocessing: 'i' empties the goal without touching anything else, which is fine only because we stop right here with
        // an optimality proof, the prefix being the whole solution
        if (inst_.goal.empty()) {
            // ignoring the prefix, the optimal is a 0-cost solution, so we need to update both lower bound and incumbent to 0
            global_.best_bound = 0;
            global_.best_incumbent = 0;
            global_.solution = {};
            logger_[DEBUG] << std::format("Computed a prefix of cost {}", global_.cost_prefix);
            logger_[DEBUG] << std::format("Size of the prefix: {}", global_.solution_prefix.size());
            throw EarlyExit("preprocessing", EarlyExit::OPTIMAL);
        }
    }

    // Fixed facts can be treated as goal facts (we cannot move this step in the preprocessing loop, since there's a step that eliminates non-goal
    // facts that have no actions needing them... in that case goal and fixed facts are different, and keeping a small goal enables more eliminations)
    if (params_.get<cli_desc::preprocess, std::string>().find('g') != std::string::npos) {
        inst_.goal |= global_.fixed_facts;
    }

    logger_[DEBUG] << std::format("Computed a prefix of cost {}", global_.cost_prefix);
    logger_[DEBUG] << std::format("Size of the prefix: {}", global_.solution_prefix.size());
}

void Solver::prep_initial_state_removal_() {
    logger_[INFO] << ("Removing initial state variables");
    std::vector<size_t> istate_offsets(inst_.n);
    size_t n_opt{inst_.n};
    for (unsigned int i = 0, counter = 0; i < inst_.n; i++) {
        if (inst_.start[i]) {
            counter++;
            n_opt--;
        }
        istate_offsets[i] = counter;
    }
    inst_.n = static_cast<unsigned int>(n_opt);
    BinarySet goal_opt{inst_.n};
    for (const auto var : inst_.goal) {
        if (!inst_.start[var]) {
            goal_opt.add(static_cast<unsigned int>(var - istate_offsets[var]));
        }
    }
    inst_.goal = goal_opt;
    inst_.nfadd = 0;
    for (size_t i = 0; i < inst_.m; i++) {
        std::vector<unsigned int> act_pre_irem;
        std::vector<unsigned int> act_eff_irem;
        act_pre_irem.reserve(inst_.actions[i].pre_sparse.size());
        act_eff_irem.reserve(inst_.actions[i].eff_sparse.size());
        for (const auto& var : inst_.actions[i].pre_sparse) {
            if (!inst_.start[var]) {
                act_pre_irem.push_back(static_cast<unsigned int>(var - istate_offsets[var]));
            }
        }
        for (const auto& var : inst_.actions[i].eff_sparse) {
            if (!inst_.start[var]) {
                act_eff_irem.push_back(static_cast<unsigned int>(var - istate_offsets[var]));
            }
        }
        inst_.actions[i].pre_sparse = std::move(act_pre_irem);
        inst_.actions[i].eff_sparse = std::move(act_eff_irem);
        inst_.nfadd += inst_.actions[i].eff_sparse.size();
    }

    // Set this to an empty set of capacity 1 so an error is thrown if this is ever used (different size BinarySet)... if they are of the same size,
    // then this is correct (empty binary set of the correct number of facts)
    inst_.start = BinarySet{1};

    logger_[DEBUG] << std::format("{} variables (istate-removed)", inst_.n);
    logger_[DEBUG] << std::format("{} first adders", inst_.nfadd);

    const bool is_infeasible = (  // add here other fast feasibility checks
        inst_.m == 0 && !inst_.goal.empty());

    if (is_infeasible) {
        throw EarlyExit("performing binary expansion", EarlyExit::INFEASIBLE);
    }
}

void Solver::prep_eliminated_facts_(std::vector<std::vector<unsigned int>>& landmarks) {
    if (global_.eliminated_facts.empty()) {
        return;
    }
    myassert(global_.eliminated_facts.capacity() == inst_.n, "eliminated_facts capacity out of sync with the fact count");
    myassert(landmarks.size() == inst_.n, "landmarks out of sync with the fact count");

    unsigned int removed{static_cast<unsigned int>(global_.eliminated_facts.size())};
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

    // Adjust landmarks: the surviving facts come in increasing order, so counter never runs ahead of fact and the entry can be moved down and then
    // filtered in place, rather than copied aside to survive being cleared
    unsigned int counter{0};
    for (const auto fact : !global_.eliminated_facts) {
        if (counter != fact) {
            landmarks[counter] = std::move(landmarks[fact]);
        }
        std::erase_if(landmarks[counter], [this](const auto factlm) { return global_.eliminated_facts[factlm]; });
        for (auto& factlm : landmarks[counter]) {
            factlm -= removed_offsets[factlm];
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

    inst_.n -= removed;

    global_.eliminated_facts = BinarySet{inst_.n};
    global_.fixed_facts = new_fixed_facts;
}

void Solver::prep_eliminated_actions_() {
    if (global_.eliminated_actions.empty()) {
        return;
    }
    myassert(global_.eliminated_actions.capacity() == inst_.m, "eliminated_actions capacity out of sync with the action count");
    myassert(global_.eliminated_actions.size() < inst_.m, "Attempted to eliminate every action (the caller must handle this case beforehand)");

    BinarySet new_fixed_actions(static_cast<unsigned int>(inst_.m - global_.eliminated_actions.size()));
    unsigned int write_pos = 0;
    unsigned int nfadd{0};  // eliminated actions take their effect entries with them: recount while compacting, instead of in a pass of its own
    for (unsigned int read_pos = 0; read_pos < inst_.m; ++read_pos) {
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
        nfadd += inst_.actions[write_pos].eff_sparse.size();
        ++write_pos;
    }

    inst_.m = write_pos;
    inst_.nfadd = nfadd;

    inst_.actions.resize(inst_.m);
    inst_.actions_names.resize(inst_.m);

    global_.eliminated_actions = BinarySet{inst_.m};
    global_.fixed_actions = new_fixed_actions;
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
