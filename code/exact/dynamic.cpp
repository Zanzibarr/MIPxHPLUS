#include "dynamic.hpp"

#include <list>
#include <numeric>  // std::accumulate
#include <set>
#include <tuple>

#if HPLUS_INTCHECK == 0
#define INTCHECK_PQ false
#endif
#include "pq.hxx"

static std::tuple<std::vector<size_t>, std::vector<size_t>, std::vector<size_t>, binary_set, unsigned int> cb_analyze_candidate_point(
    const hplus::instance& inst, const double* xstar) {
    // split among used and unused actions
    std::vector<size_t> used_acts, unused_acts;
    for (size_t act_i_cpx = 0; act_i_cpx < inst.m_opt; act_i_cpx++) {
        if (xstar[act_i_cpx] > HPLUS_CPX_INT_ROUNDING)
            used_acts.push_back(act_i_cpx);
        else
            unused_acts.push_back(act_i_cpx);
    }

    // init queue of applicable actions
    std::deque<size_t> applicable_queue;
    for (const auto& act_i_cpx : used_acts) {
        if (inst.actions[inst.act_cpxtoidx[act_i_cpx]].pre.empty()) applicable_queue.push_back(act_i_cpx);
    }

    // dividing actions in reachable / unreachable, building up the set of reachable facts and computing the partial cost
    std::vector<size_t> reachable_acts;
    binary_set reachable_state{inst.n};
    unsigned int partial_cost{0};
    while (!applicable_queue.empty()) {
        // retrieve an applicable action
        const auto act_i_cpx{applicable_queue.front()};
        applicable_queue.pop_front();
        reachable_acts.push_back(act_i_cpx);
        const auto act_i{inst.act_cpxtoidx[act_i_cpx]};
        partial_cost += inst.actions[act_i].cost;

        // compute new state
        const auto& new_state{reachable_state | inst.actions[act_i].eff};
        if (new_state == reachable_state) continue;

        // find new applicable actions
        for (const auto& p : inst.actions[act_i].eff_sparse) {
            if (reachable_state[p]) continue;  // skip facts already reached

            // look among actions that have that new fact in their precondition: since the fact is a new reached fact, all actions that might be
            // already applied won't appear here
            for (const auto& act_j : inst.act_with_pre[p]) {
                const auto& act_j_cpx{inst.act_opt_conv[act_j]};
                if (!std::binary_search(used_acts.begin(), used_acts.end(), act_j_cpx)) continue;  // if the action wasn't used, skip

                // if the actions is applicable and isn't already in the queue, add it
                if (new_state.contains(inst.actions[act_j].pre) &&
                    std::find(applicable_queue.begin(), applicable_queue.end(), act_j_cpx) == applicable_queue.end())
                    applicable_queue.push_back(act_j_cpx);
            }
        }

        // update reachable state
        reachable_state = new_state;
    }

    return {used_acts, unused_acts, reachable_acts, reachable_state, partial_cost};
}

static void cb_print_info(CPXCALLBACKCONTEXTptr& context, const double& cost, const logger& log) {
    static pthread_mutex_t cb_print_info_mutex = PTHREAD_MUTEX_INITIALIZER;
    static int call_count{0};
    if (call_count % 100 == 0) {
        double lower_bound = CPX_INFBOUND;
        CPX_HANDLE_CALL(log, CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &lower_bound));
        double incumbent = CPX_INFBOUND;
        CPX_HANDLE_CALL(log, CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_SOL, &incumbent));
        double gap = (1 - lower_bound / incumbent) * 100;
        pthread_mutex_lock(&cb_print_info_mutex);
        log.print_info("Pruned infeasible solution - cost : %7d - incumbent : %7d - gap : %6.2f%%.", static_cast<int>(cost),
                       static_cast<int>(incumbent), gap);
        pthread_mutex_unlock(&cb_print_info_mutex);
    }
    pthread_mutex_lock(&cb_print_info_mutex);
    call_count++;
    pthread_mutex_unlock(&cb_print_info_mutex);
}

static std::tuple<int*, double*, int> cb_compute_minimal_landmark(const hplus::instance& inst, const std::vector<size_t>& used_acts,
                                                                  const std::vector<size_t>& reachable_acts, const std::vector<size_t>& unused_acts,
                                                                  binary_set& reachable_state) {
    // init unapplicable actions with used unreachable actions
    std::vector<size_t> unapplicable;
    std::set_difference(used_acts.begin(), used_acts.end(), reachable_acts.begin(), reachable_acts.end(), std::back_inserter(unapplicable));

    // TODO: Maybe here I can find a larger extension set, and get a smaller landmark(???)
    std::vector<size_t> extension;

    // try to expand the set of actions looking among unused ones
    for (const auto& act_i_cpx : unused_acts) {
        const auto& act_i{inst.act_cpxtoidx[act_i_cpx]};

        // if the effects of this action won't change the reachable state, just apply it
        if (reachable_state.contains(inst.actions[act_i].eff)) {
            insert_sorted(extension, act_i_cpx);
            continue;
        }

        // if the action is unreachable, then it won't change the set of reachable facts, I can add it to the extension
        if (!reachable_state.contains(inst.actions[act_i].pre)) {
            insert_sorted(extension, act_i_cpx);
            // add this actions to the unapplicable actions, to eventually add its effects when it'd become applicable
            insert_sorted(unapplicable, act_i_cpx);
            continue;
        }

        // here the action is reachable

        // if the effect of this action reaches the goal, we don't add it to the expansion
        if ((reachable_state | inst.actions[act_i].eff).contains(inst.goal)) continue;

        // simulate the effects of using this action
        binary_set state_sim{reachable_state | inst.actions[act_i].eff};
        std::vector<size_t> new_applicable;
        while (true) {
            bool skip{true};
            for (const auto& act_j_cpx : unapplicable) {
                if (std::binary_search(new_applicable.begin(), new_applicable.end(), act_j_cpx)) continue;
                const auto& act_j{inst.act_cpxtoidx[act_j_cpx]};
                // if now a (previously) unapplicable action is applicable, add its effects to the simulated state
                if (state_sim.contains(inst.actions[act_j].pre)) {
                    insert_sorted(new_applicable, act_j_cpx);
                    state_sim |= inst.actions[act_j].eff;
                    skip = false;
                }
            }
            if (skip) break;
        }

        // if the new state doesn't contain the goal, then we can update the reachable state and remove the applicable actions from the previously
        // unapplicable ones
        if (!state_sim.contains(inst.goal)) {
            insert_sorted(extension, act_i_cpx);
            reachable_state = state_sim;
            const auto& it{
                std::set_difference(unapplicable.begin(), unapplicable.end(), new_applicable.begin(), new_applicable.end(), unapplicable.begin())};
            unapplicable.resize(it - unapplicable.begin());
        }
    }

    // TODO: Maybe try saturation as showed in the paper

    // compute the landmark as the set of actions that, if applied, would reach the goal
    std::vector<size_t> landmark;
    std::set_difference(unused_acts.begin(), unused_acts.end(), extension.begin(), extension.end(), std::back_inserter(landmark));

    int* ind{new int[inst.n_fadd]};
    double* val{new double[inst.n_fadd]};
    int nnz{0};
    for (const auto& act_i_cpx : landmark) {
        int var_count{-1};
        int init_nnz{nnz};
        for ([[maybe_unused]] const auto& p : inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff_sparse) {
            var_count++;
            if (inst.fadd_e[inst.act_cpxtoidx[act_i_cpx]][p]) continue;
            ind[nnz] = inst.m_opt + inst.fadd_cpx_start[act_i_cpx] + var_count;
            val[nnz++] = 1;
            if (inst.fadd_f[inst.act_cpxtoidx[act_i_cpx]][p]) {
                nnz = init_nnz;
                ind[nnz++] = inst.m_opt + inst.fadd_cpx_start[act_i_cpx] + var_count;
                break;
            }
        }
    }

    return {ind, val, nnz};
}

// TODO: Maybe find a way to improve more the solution, raher than blindly using actions used by CPLEX
static std::tuple<bool, int*, double*, unsigned int, size_t> cb_find_heur_sol(const hplus::instance& inst,
                                                                              const std::vector<size_t>& applicable_actions_sequence) {
    size_t ncols{inst.m_opt + inst.n_opt + inst.n_fadd};
    int* ind{new int[ncols]};
    double* val{new double[ncols]};
    for (size_t i = 0; i < ncols; i++) {
        ind[i] = i;
        val[i] = 0;
    }

    binary_set state{inst.n_opt};
    unsigned int hcost = 0;
    bool changed = false;
    for (const auto& act_i_cpx : applicable_actions_sequence) {
        if (state.contains(inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff) && inst.actions[inst.act_cpxtoidx[act_i_cpx]].cost != 0) {
            changed = true;
            continue;
        }
        val[act_i_cpx] = 1;
        int var_count{-1};
        for (const auto& var_i : inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;
            val[inst.m_opt + inst.fadd_cpx_start[act_i_cpx] + var_count] = 1;
            val[inst.m_opt + inst.n_fadd + inst.var_opt_conv[var_i]] = 1;
        }
        state |= inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff;
        hcost += inst.actions[inst.act_cpxtoidx[act_i_cpx]].cost;
        if (state.contains(inst.goal) && inst.actions[inst.act_cpxtoidx[act_i_cpx]].cost != 0) {
            if (act_i_cpx != applicable_actions_sequence[applicable_actions_sequence.size() - 1]) changed = true;
            break;
        }
    }

    return {changed, ind, val, hcost, ncols};
}

int CPXPUBLIC dynamic::cpx_callback(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle) {
    auto& [inst, env, stats, log] = *static_cast<cpx_callback_user_handle*>(user_handle);

    ASSERT_LOG(log, context_id == CPX_CALLBACKCONTEXT_CANDIDATE);
    double start_time{env.timer.get_time()};

    // get candidate point
    double* xstar{new double[inst.m_opt]};
    double cost{CPX_INFBOUND};
    CPX_HANDLE_CALL(log, CPXcallbackgetcandidatepoint(context, xstar, 0, inst.m_opt - 1, &cost));

    // get info from the candidate point
    auto [used_acts, unused_acts, reachable_acts, reachable_state, partial_cost] = cb_analyze_candidate_point(inst, xstar);
    delete[] xstar;
    xstar = nullptr;

    // solution is feasible (all used actions are reachable)
    if (used_acts.size() == reachable_acts.size()) {
        INTCHECK_ASSERT_LOG(log, reachable_state.contains(inst.goal));
        return 0;
    }

    // print info for debugging
#if HPLUS_VERBOSE >= 100
    cb_print_info(context, cost, log);
#endif

    constexpr double rhs{1};
    constexpr int izero{0};
    constexpr char sense{'G'};

    // the goal state here was reached
    if (reachable_state.contains(inst.goal)) {
        // Post the (possibly) improved feasible solution
        auto [changed, ind, val, hcost, size] = cb_find_heur_sol(inst, reachable_acts);
        if (changed) CPX_HANDLE_CALL(log, CPXcallbackpostheursoln(context, size, ind, val, hcost, CPXCALLBACKSOLUTION_NOCHECK))
        delete[] val;
        val = nullptr;
        delete[] ind;
        ind = nullptr;

        // reject solution: //TODO: Maybe I can add a constraint here
        constexpr int tmpind{0};
        constexpr double tmpval{0};
        CPX_HANDLE_CALL(log, CPXcallbackrejectcandidate(context, 0, 0, &rhs, &sense, &izero, &tmpind, &tmpval));
        return 0;
    }

    // reject the candidate point adding the minimal landmark as constraint
    std::sort(reachable_acts.begin(), reachable_acts.end());
    auto [ind, val, size]{cb_compute_minimal_landmark(inst, used_acts, reachable_acts, unused_acts, reachable_state)};
    CPX_HANDLE_CALL(log, CPXcallbackrejectcandidate(context, 1, size, &rhs, &sense, &izero, ind, val));
    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    // TODO: I could add more constraints (cut section landmark / SEC)

    pthread_mutex_lock(&(stats.callback_time_mutex));
    stats.nusercuts++;
    stats.callback += env.timer.get_time() - start_time;
    pthread_mutex_unlock(&(stats.callback_time_mutex));

    return 0;
}

void dynamic::build_cpx_model(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log,
                              hplus::statistics& stats) {
    PRINT_VERBOSE(log, "Building Dynamic model.");

    const auto stopchk1 = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    // ====================================================== //
    // =================== TIGHTER BOUNDS =================== //
    // ====================================================== //

    unsigned int max_steps{static_cast<unsigned int>(inst.m_opt)};
    if (env.tight_bounds) {
        // number of variables
        if (inst.n_opt < max_steps) max_steps = inst.n_opt;

        // max number of steps to reach heuristic
        if (env.heur != "none") {
            unsigned int min_act_cost{inst.actions[0].cost + 1};  // +1 to avoid it being 0
            unsigned int n_act_zerocost{0};
            for (const auto& act_i : inst.act_rem) {
                if (inst.actions[act_i].cost == 0)
                    n_act_zerocost++;
                else if (inst.actions[act_i].cost < min_act_cost)
                    min_act_cost = inst.actions[act_i].cost;
            }
            const unsigned int nsteps{static_cast<unsigned int>(std::ceil(static_cast<double>(inst.best_sol.cost) / min_act_cost)) + n_act_zerocost};
            if (nsteps < max_steps) max_steps = nsteps;
        }
    }
    stopchk1();

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    size_t curr_col{0};
    double* objs{new double[inst.m_opt]};
    double* lbs{new double[inst.m_opt]};
    double* ubs{new double[inst.m_opt]};
    char* types{new char[inst.m_opt]};

    const auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
        delete[] types;
        types = nullptr;
        delete[] ubs;
        ubs = nullptr;
        delete[] lbs;
        lbs = nullptr;
        delete[] objs;
        objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];
    };

    const auto stopchk2 = [&objs, &lbs, &ubs, &types]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] types;
            types = nullptr;
            delete[] ubs;
            ubs = nullptr;
            delete[] lbs;
            lbs = nullptr;
            delete[] objs;
            objs = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // -------- actions ------- //
    const size_t act_start{curr_col};
    size_t count{0};
    for (const auto& act_i : inst.act_rem) {
        objs[count] = static_cast<double>(inst.actions[act_i].cost);
        lbs[count] = (inst.act_f[act_i] ? 1 : 0);
        ubs[count] = 1;
        types[count++] = 'B';
    }

    curr_col += count;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, count, objs, lbs, ubs, types, nullptr));
    stopchk2();

    resize_cpx_arrays(inst.n_opt);

    // --- first archievers --- //
    const size_t fa_start{curr_col};
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        size_t count_var{0};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            objs[count_var] = 0;
            lbs[count_var] = (inst.fadd_f[act_i][var_i] ? 1 : 0);
            ubs[count_var] = (inst.fadd_e[act_i][var_i] ? 0 : 1);
            types[count_var++] = 'B';
        }
        curr_col += count_var;
        CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, count_var, objs, lbs, ubs, types, nullptr));
        count++;
        stopchk2();
    }

    // ------- variables ------ //
    const size_t var_start{curr_col};
    count = 0;
    for (const auto& var_i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = (inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    curr_col += count;

    CPX_HANDLE_CALL(log, CPXnewcols(cpxenv, cpxlp, count, objs, lbs, ubs, types, nullptr));
    stopchk2();

    delete[] types;
    types = nullptr;
    delete[] ubs;
    ubs = nullptr;
    delete[] lbs;
    lbs = nullptr;
    delete[] objs;
    objs = nullptr;

    stats.nvar_base = inst.n_opt + inst.m_opt + inst.n_fadd;
    stats.nvar_acyclic = curr_col - stats.nvar_base;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const auto get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
    const auto get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
    const auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_count) {
        return static_cast<int>(fa_start + inst.fadd_cpx_start[inst.act_opt_conv[act_idx]] + var_count);
    };

    int* ind{new int[inst.m_opt + 1]};
    double* val{new double[inst.m_opt + 1]};
    int nnz{0};
    constexpr char sense_e{'E'}, sense_l{'L'};
    constexpr double rhs_0{0};
    constexpr int begin{0};

    const auto stopchk3 = [&ind, &val]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] val;
            val = nullptr;
            delete[] ind;
            ind = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    stats.nconst_base = 0;
    stats.nconst_acyclic = 0;

    // p = \sum_{a\in A,p\in eff(a)}(fadd(a, p)), \forall p \in V
    for (const auto& var_i : inst.var_rem) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        bool fixed = false;
        for (const auto& act_i : inst.act_with_eff[var_i]) {
            // if one first adder is fixed, then also the variable should be fixed
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                fixed = true;
                CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                break;
            }
            // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            size_t var_count = static_cast<size_t>(std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), var_i) -
                                                   inst.actions[act_i].eff_sparse.begin());
            ind[nnz] = get_fa_idx(act_i, var_count);
            val[nnz++] = -1;
        }
        // if we fixed the variable due to a fixed first adder (note also that if we had a fixed first adder, we already have all other first
        // adders for that effect eliminated), we don't need the constraint we're adding.
        if (fixed) continue;

        // if nnz == 1, then we'd have p = 0, meaning we could simply fix this variable to 0
        if (nnz == 1) {
            const char fix = 'B';
            const double zero = 0;
            CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &zero));
        } else {
            stats.nconst_base++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr));
        }
        stopchk3();
    }

    // \sum_{a\in A, p\in eff(a), q\in pre(a)}(fadd(a, p)) <= q, \forall p,q\in V
    for (const auto& p : inst.var_rem) {
        for (const auto& q : inst.var_rem) {
            nnz = 0;
            ind[nnz] = get_var_idx(q);
            val[nnz++] = -1;
            bool fixed = false;
            for (const auto& act_i : inst.act_with_eff[p]) {
                if (!inst.actions[act_i].pre[q]) continue;
                // if the first adder is fixed, than we have 1 <= q, hence we can directly fix q
                if (inst.fadd_f[act_i][p]) {
                    const char fix = 'B';
                    const double one = 1;
                    fixed = true;
                    CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                    break;
                }
                // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
                else if (inst.fadd_e[act_i][p])
                    continue;
                size_t var_count = static_cast<size_t>(std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), p) -
                                                       inst.actions[act_i].eff_sparse.begin());
                ind[nnz] = get_fa_idx(act_i, var_count);
                val[nnz++] = 1;
            }
            // since we have a fixed first adder, all other first adder for that effect are eliminated, so we don't need the constraint
            if (fixed) continue;
            // if nnz == 1 than we have -p <= 0, hence it's always true, we can ignore this constraint
            if (nnz != 1) {
                stats.nconst_base++;
                CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr));
            }
            stopchk3();
        }
    }

    if (env.tight_bounds) {
        double rhs{static_cast<double>(max_steps)};
        nnz = 0;
        for (const auto& act_i : inst.act_rem) {
            if (inst.act_f[act_i]) {
                rhs--;
                continue;
            }
            ind[nnz] = get_act_idx(act_i);
            val[nnz++] = 1;
        }
        stats.nconst_base++;
        CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs, &sense_l, &begin, ind, val, nullptr, nullptr));
    }

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    int ind_2[2];
    double val_2[2];

    // fadd(a, p) <= a, \forall a\in A, p\in eff(a)
    for (const auto& act_i : inst.act_rem) {
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            ind_2[0] = get_act_idx(act_i);
            val_2[0] = -1;
            // if the first adder was fixed, we can directly fix the action insthead of adding the constraint
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind_2, &fix, &one));
                continue;
            }
            // if the first adder was eliminated, we can skip the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            ind_2[1] = get_fa_idx(act_i, var_count);
            val_2[1] = 1;
            stats.nconst_base++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_2, val_2, nullptr, nullptr));
        }
        stopchk1();
    }

    if (env.write_lp) CPX_HANDLE_CALL(log, CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

void dynamic::post_cpx_warmstart([[maybe_unused]] CPXENVptr& cpxenv, [[maybe_unused]] CPXLPptr& cpxlp, [[maybe_unused]] const hplus::instance& inst,
                                 [[maybe_unused]] const hplus::environment& env, [[maybe_unused]] const logger& log) {
    PRINT_VERBOSE(log, "Posting warm start to Dynamic model.");
    ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

    binary_set state{inst.n};

    const auto& warm_start{inst.best_sol.plan};

    const size_t ncols{static_cast<size_t>(CPXgetnumcols(cpxenv, cpxlp))};
    int* cpx_sol_ind{new int[ncols]};
    double* cpx_sol_val{new double[ncols]};
    for (size_t i = 0; i < ncols; i++) {
        cpx_sol_ind[i] = i;
        cpx_sol_val[i] = 0;
    }

    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};

    for (const auto& act_i : warm_start) {
        size_t cpx_act_idx = inst.act_opt_conv[act_i];
        cpx_sol_val[cpx_act_idx] = 1;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;

            size_t cpx_var_idx = inst.m_opt + inst.n_fadd + inst.var_opt_conv[var_i];
            cpx_sol_val[cpx_var_idx] = 1;
            size_t cpx_fad_idx = inst.m_opt + inst.fadd_cpx_start[inst.act_opt_conv[act_i]] + var_count;
            cpx_sol_val[cpx_fad_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
    }

    CPX_HANDLE_CALL(log, CPXaddmipstarts(cpxenv, cpxlp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));
    delete[] cpx_sol_ind;
    cpx_sol_ind = nullptr;
    delete[] cpx_sol_val;
    cpx_sol_val = nullptr;
}

void dynamic::store_cpx_sol([[maybe_unused]] CPXENVptr& cpxenv, [[maybe_unused]] CPXLPptr& cpxlp, [[maybe_unused]] hplus::instance& inst,
                            [[maybe_unused]] const logger& log) {
    PRINT_VERBOSE(log, "Storing Dynamic solution.");

    double* plan{new double[inst.m_opt + inst.n_fadd]};
    CPX_HANDLE_CALL(log, CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt + inst.n_fadd - 1));

    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (size_t act_i_cpx = 0; act_i_cpx < inst.m_opt; act_i_cpx++) {
        bool set_zero{true};
        for (size_t var_count = 0; var_count < inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff_sparse.size(); var_count++) {
            if (plan[inst.m_opt + inst.fadd_cpx_start[act_i_cpx] + var_count] > HPLUS_CPX_INT_ROUNDING) {
                ASSERT_LOG(log, plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
                set_zero = false;
                break;
            }
        }
        if (set_zero) {
            ASSERT_LOG(log, !(plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING && inst.actions[inst.act_cpxtoidx[act_i_cpx]].cost != 0));
            plan[act_i_cpx] = 0;
        }
    }

    // convert to std collections for easier parsing
    std::vector<size_t> cpx_result;
    cpx_result.reserve(inst.m_opt);
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (plan[i] > HPLUS_CPX_INT_ROUNDING) cpx_result.push_back(inst.act_cpxtoidx[i]);
    }
    delete[] plan;
    plan = nullptr;

    std::vector<size_t> solution;
    solution.reserve(inst.m_opt);
    binary_set remaining{cpx_result.size(), true}, state{inst.n};

    while (!remaining.empty()) {
        bool intcheck{false};
        for (const auto& i : remaining) {
            if (!state.contains(inst.actions[cpx_result[i]].pre)) continue;

            remaining.remove(i);
            state |= inst.actions[cpx_result[i]].eff;
            solution.push_back(cpx_result[i]);
            intcheck = true;
        }
        ASSERT_LOG(log, intcheck);
    }

    // store solution
    hplus::solution rankooh_sol{
        solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [&inst](const unsigned int acc, const size_t index) {
            return acc + inst.actions[index].cost;
        }))};
    hplus::update_sol(inst, rankooh_sol, log);
}