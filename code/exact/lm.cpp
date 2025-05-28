#include "lm.hpp"

#include <list>
#include <numeric>  // std::accumulate
#include <set>
#include <stack>
#include <tuple>
#include <unordered_set>

#if HPLUS_INTCHECK == 0
#define INTCHECK_PQ false
#endif
#include "pq.hxx"

// ##################################################################### //
// ############################## CALLBACK ############################# //
// ##################################################################### //

static std::tuple<std::vector<size_t>, std::vector<size_t>, std::vector<size_t>, binary_set, std::vector<binary_set>> cb_analyze_candidate_point(
    const hplus::instance& inst, const double* xstar) {
    // split among used and unused actions
    std::vector<size_t> used_acts, unused_acts;
    for (size_t act_i_cpx = 0; act_i_cpx < inst.m_opt; act_i_cpx++) {
        if (xstar[act_i_cpx] > HPLUS_CPX_INT_ROUNDING)
            used_acts.push_back(act_i_cpx);
        else
            unused_acts.push_back(act_i_cpx);
    }

    // get used first archievers
    std::vector<binary_set> used_first_archievers(inst.m_opt, binary_set(inst.n));
    for (size_t act_i_cpx = 0; act_i_cpx < inst.m_opt; act_i_cpx++) {
        size_t var_count{0};
        for (const auto& p : inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff_sparse) {
            size_t idx = inst.m_opt + inst.fadd_cpx_start[act_i_cpx] + var_count;
            if (xstar[idx] > HPLUS_CPX_INT_ROUNDING) used_first_archievers[act_i_cpx].add(p);
            var_count++;
        }
    }

    // init queue of applicable actions
    std::deque<size_t> applicable_queue;
    for (const auto& act_i_cpx : used_acts) {
        if (inst.actions[inst.act_cpxtoidx[act_i_cpx]].pre.empty()) applicable_queue.push_back(act_i_cpx);
    }

    // dividing actions in reachable / unreachable, building up the set of reachable facts and computing the partial cost
    std::vector<size_t> reachable_acts;
    binary_set reachable_state{inst.n};
    while (!applicable_queue.empty()) {
        // retrieve an applicable action
        const auto act_i_cpx{applicable_queue.front()};
        applicable_queue.pop_front();
        reachable_acts.push_back(act_i_cpx);
        const auto act_i{inst.act_cpxtoidx[act_i_cpx]};

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

    return {used_acts, unused_acts, reachable_acts, reachable_state, used_first_archievers};
}

static void cb_print_info(CPXCALLBACKCONTEXTptr& context, const double& cost, const logger& log) {
    static int call_count{0};
    if (call_count % 100 == 0) {
        double lower_bound = CPX_INFBOUND;
        CPX_HANDLE_CALL(log, CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &lower_bound));
        double incumbent = CPX_INFBOUND;
        CPX_HANDLE_CALL(log, CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_SOL, &incumbent));
        double gap = (1 - lower_bound / incumbent) * 100;
        log.print_info("ILP  -  Pruned infeasible integer solution  - cost: %3d, gap: %4.2f%%.", static_cast<int>(cost), gap);
    }
    call_count++;  // not thread safe, but it's not that important to print at a slightly wrong rate
}

// ====================================================== //
// ===================== IMPROVE SOL ==================== //
// ====================================================== //

static std::tuple<int*, double*, unsigned int, size_t> cb_find_heur_sol(const hplus::instance& inst,
                                                                        const std::vector<size_t>& applicable_actions_sequence) {
    size_t ncols{inst.m_opt + inst.n_opt + inst.n_fadd};
    int* ind{new int[ncols]};
    double* val{new double[ncols]};
    for (size_t i = 0; i < ncols; i++) {
        ind[i] = i;
        val[i] = 0;
    }

    binary_set state{inst.n};
    unsigned int hcost = 0;
    for (const auto& act_i_cpx : applicable_actions_sequence) {
        if (state.contains(inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff)) continue;
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
        if (state.contains(inst.goal)) break;
    }

    return {ind, val, hcost, ncols};
}

// ====================================================== //
// ====================== LANDMARKS ===================== //
// ====================================================== //

// From HaslumMLM12
static std::vector<size_t> cb_compute_minimal_landmark(const hplus::instance& inst, const std::vector<size_t>& used_acts,
                                                       const std::vector<size_t>& reachable_acts, const std::vector<size_t>& unused_acts,
                                                       const binary_set& reachable_state) {
    // init unapplicable actions with used unreachable actions
    std::vector<size_t> unapplicable;
    std::set_difference(used_acts.begin(), used_acts.end(), reachable_acts.begin(), reachable_acts.end(), std::back_inserter(unapplicable));

    std::vector<size_t> extension;
    binary_set state{reachable_state};

    // try to expand the set of actions looking among unused ones
    for (const auto& act_i_cpx : unused_acts) {
        const auto& act_i{inst.act_cpxtoidx[act_i_cpx]};

        // if the effects of this action won't change the reachable state, just apply it
        if (state.contains(inst.actions[act_i].eff)) {
            insert_sorted(extension, act_i_cpx);
            continue;
        }

        // if the action is unreachable, then it won't change the set of reachable facts, I can add it to the extension
        if (!state.contains(inst.actions[act_i].pre)) {
            insert_sorted(extension, act_i_cpx);
            // add this actions to the unapplicable actions, to eventually add its effects when it'd become applicable
            insert_sorted(unapplicable, act_i_cpx);
            continue;
        }

        // here the action is reachable

        // if the effect of this action reaches the goal, we don't add it to the expansion
        binary_set state_sim{state | inst.actions[act_i].eff};
        if (state_sim.contains(inst.goal)) continue;

        // simulate the effects of using this action
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
            if (skip) [[unlikely]]
                break;
        }

        // if the new state doesn't contain the goal, then we can update the reachable state and remove the applicable actions from the previously
        // unapplicable ones
        if (!state_sim.contains(inst.goal)) {
            insert_sorted(extension, act_i_cpx);
            state = state_sim;
            const auto& it{
                std::set_difference(unapplicable.begin(), unapplicable.end(), new_applicable.begin(), new_applicable.end(), unapplicable.begin())};
            unapplicable.resize(it - unapplicable.begin());
        }
    }

    // compute the landmark as the set of actions that, if applied, would reach the goal
    std::vector<size_t> landmark;
    std::set_difference(unused_acts.begin(), unused_acts.end(), extension.begin(), extension.end(), std::back_inserter(landmark));

    return landmark;
}

// From BonetComplete
static std::vector<size_t> cb_compute_complete_landmark(const hplus::instance& inst, const std::vector<size_t>& unused_acts,
                                                        const binary_set& reachable_state) {
    std::vector<size_t> landmark;
    for (const auto& act_i_cpx : unused_acts) {
        if (reachable_state.contains(inst.actions[inst.act_cpxtoidx[act_i_cpx]].pre) &&
            !reachable_state.contains(inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff))
            landmark.push_back(act_i_cpx);
    }

    return landmark;
}

static std::tuple<int*, double*, int, double*, char*, int*> cb_cpxconvert_landmark_cut(const hplus::instance& inst,
                                                                                       const std::vector<std::vector<size_t>>& landmarks) {
    const size_t max_size = landmarks.size() * inst.m_opt;

    int* ind{new int[max_size]};
    double* val{new double[max_size]};
    int nnz{0};
    double* rhs{new double[landmarks.size()]};
    char* sense{new char[landmarks.size()]};
    int* izero{new int[landmarks.size()]};
    int lm_counter{0};

    for (const auto& landmark : landmarks) {
        rhs[lm_counter] = 1;
        sense[lm_counter] = 'G';
        izero[lm_counter++] = nnz;
        for (const auto& act_i_cpx : landmark) {
            ind[nnz] = act_i_cpx;
            val[nnz++] = 1;
        }
    }

    return {ind, val, nnz, rhs, sense, izero};
}

// ====================================================== //
// ======================= S.E.C. ======================= //
// ====================================================== //

static bool cycles_dfs(const std::vector<std::set<size_t>>& graph, const size_t& start, const size_t& current, std::vector<size_t>& path,
                       std::unordered_set<size_t>& in_path, std::unordered_set<size_t>& visited, std::vector<std::vector<size_t>>& cycles,
                       const std::unordered_set<size_t>& cycle_found) {
    // Skip if current node is already in another cycle
    if (cycle_found.count(current)) return false;

    path.push_back(current);
    in_path.insert(current);
    visited.insert(current);

    for (size_t neighbor : graph[current]) {
        // If we find the start node, we've found a cycle
        if (neighbor == start) {
            cycles.push_back(path);
            return true;
        }

        // If neighbor is already in another cycle, skip
        if (cycle_found.count(neighbor)) continue;

        // If neighbor is already in current path, we found a smaller cycle
        if (in_path.count(neighbor)) {
            // Extract the cycle from the current path
            std::vector<size_t> small_cycle;
            auto it = path.begin();
            // Find the position of neighbor in path
            while (it != path.end() && *it != neighbor) ++it;

            // Copy from neighbor position to end of path
            small_cycle.assign(it, path.end());

            cycles.push_back(small_cycle);
            return true;
        }

        // Skip if already visited in another path and didn't lead to a cycle
        if (visited.count(neighbor)) continue;

        // Recurse
        if (cycles_dfs(graph, start, neighbor, path, in_path, visited, cycles, cycle_found)) return true;
    }

    // Backtrack
    path.pop_back();
    in_path.erase(current);
    return false;
}

static std::vector<std::vector<size_t>> cb_compute_sec(const hplus::instance& inst, const std::vector<binary_set>& used_first_archievers,
                                                       const std::vector<size_t>& unreachable_acts) {
    // build graph G=<A, {(ai, aj) ai, aj in A | ai is first archiever (in the current solution) to a precondition of aj}>
    std::vector<std::set<size_t>> graph(inst.m_opt);
    for (const auto& act_i_cpx : unreachable_acts) {
        for (const auto& p : used_first_archievers[act_i_cpx]) {
            for (const auto& act_j : inst.act_with_pre[p]) {
                if (std::binary_search(unreachable_acts.begin(), unreachable_acts.end(), inst.act_opt_conv[act_j]))
                    graph[act_i_cpx].insert(inst.act_opt_conv[act_j]);
            }
        }
    }

    std::vector<std::vector<size_t>> cycles;
    std::unordered_set<size_t> cycle_found;

    for (size_t start = 0; start < graph.size(); start++) {
        // Skip if we already found a cycle containing this node
        if (cycle_found.count(start)) continue;

        std::vector<size_t> path;
        std::unordered_set<size_t> in_path;
        std::unordered_set<size_t> visited;

        // Run DFS to find a cycle starting from this node
        if (cycles_dfs(graph, start, start, path, in_path, visited, cycles, cycle_found)) {
            // Mark all nodes in the found cycle as having a cycle
            for (size_t node : cycles.back()) {
                cycle_found.insert(node);
            }
        }
    }

    return cycles;
}

static std::tuple<int*, double*, int, double*, char*, int*> cb_cpxconvert_sec_cut(const hplus::instance& inst,
                                                                                  const std::vector<binary_set> used_first_archievers,
                                                                                  const std::vector<std::vector<size_t>>& cycles) {
    const size_t max_size = cycles.size() * inst.n_opt;

    int* ind{new int[max_size]};
    double* val{new double[max_size]};
    int nnz{0};
    double* rhs{new double[cycles.size()]};
    char* sense{new char[cycles.size()]};
    int* izero{new int[cycles.size()]};
    int sec_counter{0};

    for (const auto& cycle : cycles) {
        sense[sec_counter] = 'L';
        izero[sec_counter] = nnz;
        rhs[sec_counter] = -1;

        // first archievers in the cycle
        int var_count{-1};
        for (size_t i = 0; i < cycle.size() - 1; i++) {
            var_count = -1;
            for (const auto& p : inst.actions[inst.act_cpxtoidx[cycle[i]]].eff_sparse) {
                var_count++;
                if (!used_first_archievers[cycle[i]][p] || !inst.actions[inst.act_cpxtoidx[cycle[i + 1]]].pre[p]) continue;

                ind[nnz] = inst.m_opt + inst.fadd_cpx_start[cycle[i]] + var_count;
                val[nnz++] = 1;
                rhs[sec_counter]++;
            }
        }
        var_count = -1;
        for (const auto& p : inst.actions[inst.act_cpxtoidx[cycle[cycle.size() - 1]]].eff_sparse) {
            var_count++;
            if (!used_first_archievers[cycle[cycle.size() - 1]][p] || !inst.actions[inst.act_cpxtoidx[cycle[0]]].pre[p]) continue;

            ind[nnz] = inst.m_opt + inst.fadd_cpx_start[cycle[cycle.size() - 1]] + var_count;
            val[nnz++] = 1;
            rhs[sec_counter]++;
        }

        sec_counter++;
    }

    return {ind, val, nnz, rhs, sense, izero};
}

// ====================================================== //
// ============ LANDMARKS FOR FRACT SOLUTIONS =========== //
// ====================================================== //

static void cpx_open_lmcut_model(CPXENVptr& lmcutenv, CPXLPptr& lmcutlp, const logger& log) {
    int cpxerror;
    lmcutenv = CPXopenCPLEX(&cpxerror);
    CPX_HANDLE_CALL(log, cpxerror);
    lmcutlp = CPXcreateprob(lmcutenv, &cpxerror, "lmcutmodel");
    CPX_HANDLE_CALL(log, cpxerror);
    // log file
    CPX_HANDLE_CALL(log, CPXsetintparam(lmcutenv, CPXPARAM_ScreenOutput, HPLUS_DEF_CPX_SCREENOUTPUT));
    CPX_HANDLE_CALL(log, CPXsetintparam(lmcutenv, CPX_PARAM_CLONELOG, HPLUS_DEF_CPX_CLONELOG));
    // tolerance
    CPX_HANDLE_CALL(log, CPXsetdblparam(lmcutenv, CPXPARAM_MIP_Tolerances_MIPGap, HPLUS_DEF_CPX_TOL_GAP));
    // memory/size limits
    CPX_HANDLE_CALL(log, CPXsetdblparam(lmcutenv, CPXPARAM_MIP_Limits_TreeMemory, HPLUS_DEF_CPX_TREE_MEM));
    CPX_HANDLE_CALL(log, CPXsetdblparam(lmcutenv, CPXPARAM_WorkMem, HPLUS_DEF_CPX_WORK_MEM));
    CPX_HANDLE_CALL(log, CPXsetintparam(lmcutenv, CPXPARAM_MIP_Strategy_File, HPLUS_DEF_CPX_STRAT_FILE));
    // terminate condition
    CPX_HANDLE_CALL(log, CPXsetterminate(lmcutenv, &global_terminate));
    // log file
    // CPX_HANDLE_CALL(log, CPXsetlogfilename(lmcutenv, HPLUS_CPLEX_OUTPUT_DIR "/log/lmcutmodel.log", "w"));
    CPXwriteparam(lmcutenv, HPLUS_CPLEX_OUTPUT_DIR "/.prm");
}

static void cpx_build_lmcut_model(CPXENVptr& lmcutenv, CPXLPptr& lmcutlp, const hplus::instance& inst, const logger& log) {
    // ~~~~~~~~~~~~~~ VARIABLES ~~~~~~~~~~~~~~ //
    size_t ncols{inst.m_opt + inst.n_opt};
    const auto get_za_idx = [&inst](size_t idx) { return inst.act_opt_conv[idx]; };  // 0 -> inst.m_opt - 1: z_a
    const auto get_yp_idx = [&inst](size_t idx) {                                    // inst.m_opt + 1 -> ncols - 1: y_p
        return inst.m_opt + inst.var_opt_conv[idx];
    };

    double* objs{new double[ncols]};
    double* lbs{new double[ncols]};
    double* ubs{new double[ncols]};
    char* types{new char[ncols]};
    for (size_t var_count = 0; var_count < ncols; var_count++) {
        objs[var_count] = 0;
        lbs[var_count] = 0;
        ubs[var_count] = 1;
        types[var_count] = 'B';
    }
    for (size_t var_count = 0; var_count < inst.m_opt; var_count++) {
        objs[var_count] = 1;  // min \sum_{a\in A}(x^*_a z_a)
    }
    CPX_HANDLE_CALL(log, CPXnewcols(lmcutenv, lmcutlp, ncols, objs, lbs, ubs, types, nullptr));
    delete[] types;
    types = nullptr;
    delete[] ubs;
    ubs = nullptr;
    delete[] lbs;
    lbs = nullptr;
    delete[] objs;
    objs = nullptr;

    // ~~~~~~~~~~~~~ CONSTRAINTS ~~~~~~~~~~~~~ //
    int* ind{new int[inst.n_opt + 1]};
    double* val{new double[inst.n_opt + 1]};
    int nnz{0};
    constexpr char sense{'L'};
    constexpr int begin{0};

    for (const auto& act_i : inst.act_rem) {
        nnz = 0;
        ind[nnz] = get_za_idx(act_i);
        val[nnz++] = -1;
        double rhs{static_cast<double>(inst.actions[act_i].pre_sparse.size()) - 1};
        for (const auto& p : inst.actions[act_i].pre_sparse) {
            ind[nnz] = get_yp_idx(p);
            val[nnz++] = 1;
        }
        for (const auto& q : inst.actions[act_i].eff_sparse) {
            ind[nnz] = get_yp_idx(q);
            val[nnz] = -1;
            CPX_HANDLE_CALL(log, CPXaddrows(lmcutenv, lmcutlp, 0, 1, nnz + 1, &rhs, &sense, &begin, ind, val, nullptr, nullptr));
        }
    }

    nnz = 0;
    for (const auto& p : inst.goal) {
        ind[nnz] = get_yp_idx(p);
        val[nnz++] = 1;
    }
    double rhs{static_cast<double>(inst.goal.sparse().size()) - 1};
    CPX_HANDLE_CALL(log, CPXaddrows(lmcutenv, lmcutlp, 0, 1, nnz, &rhs, &sense, &begin, ind, val, nullptr, nullptr));

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;
}

void lm::cpx_close_lmcut_model(CPXENVptr& lmcutenv, CPXLPptr& lmcutlp, const logger& log) {
    CPX_HANDLE_CALL(log, CPXfreeprob(lmcutenv, &lmcutlp));
    CPX_HANDLE_CALL(log, CPXcloseCPLEX(&lmcutenv));
}

void lm::cpx_create_lmcut_model(const hplus::instance& inst, const logger& log, CPXENVptr& lmcutenv, CPXLPptr& lmcutlp) {
    cpx_open_lmcut_model(lmcutenv, lmcutlp, log);
    cpx_build_lmcut_model(lmcutenv, lmcutlp, inst, log);
}

// ====================================================== //
// ====================== CALLBACK ====================== //
// ====================================================== //

static void cpx_cand_callback(CPXCALLBACKCONTEXTptr& context, const hplus::instance& inst, const hplus::environment& env, double& cand_time,
                              int& usercuts_lm, int& usercuts_sec, const logger& log) {
    double start_time{env.timer.get_time()};

    // get candidate point
    double* xstar{new double[inst.m_opt + inst.n_fadd]};
    double cost{CPX_INFBOUND};
    CPX_HANDLE_CALL(log, CPXcallbackgetcandidatepoint(context, xstar, 0, inst.m_opt + inst.n_fadd - 1, &cost));

    // get info from the candidate point
    auto [used_acts, unused_acts, reachable_acts, reachable_state, used_first_archievers] = cb_analyze_candidate_point(inst, xstar);
    delete[] xstar;
    xstar = nullptr;

    // solution is feasible (all used actions are reachable)
    if (used_acts.size() == reachable_acts.size()) {
        INTCHECK_ASSERT_LOG(log, reachable_state.contains(inst.goal));
        return;
    }

    // print info for debugging
#if HPLUS_VERBOSE >= 100
    cb_print_info(context, cost, log);
#endif

    if (reachable_state.contains(inst.goal)) {
        // Post the (possibly) improved feasible solution
        auto [ind, val, hcost, size] = cb_find_heur_sol(inst, reachable_acts);
        CPX_HANDLE_CALL(log, CPXcallbackpostheursoln(context, size, ind, val, hcost, CPXCALLBACKSOLUTION_NOCHECK));
        delete[] val;
        val = nullptr;
        delete[] ind;
        ind = nullptr;

        constexpr int tmpind{0};
        constexpr double tmpval{0};
        constexpr char tmpsense{'E'};
        constexpr int tmpizero{0};
        constexpr double tmprhs{0};
        CPX_HANDLE_CALL(log, CPXcallbackrejectcandidate(context, 0, 0, &tmprhs, &tmpsense, &tmpizero, &tmpind, &tmpval));
        return;
    }

    std::sort(reachable_acts.begin(), reachable_acts.end());

    std::vector<std::vector<size_t>> landmarks_list;
    // adding the minimal landmark as constraint
    if (env.minimal_landmark) landmarks_list.push_back(cb_compute_minimal_landmark(inst, used_acts, reachable_acts, unused_acts, reachable_state));
    // adding the complete landmark as constraint
    if (env.complete_landmark) landmarks_list.push_back(cb_compute_complete_landmark(inst, unused_acts, reachable_state));

    // convert the landmark into CPLEX data structures
    auto [ind, val, size, rhs, sense, izero] = cb_cpxconvert_landmark_cut(inst, landmarks_list);

    // reject the candidate point and add the landmarks as new cuts
    CPX_HANDLE_CALL(log, CPXcallbackrejectcandidate(context, landmarks_list.size(), size, rhs, sense, izero, ind, val));

    delete[] rhs;
    rhs = nullptr;
    delete[] sense;
    sense = nullptr;
    delete[] izero;
    izero = nullptr;
    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    // compute the SEC and add them as new cuts
    int n_sec{0};
    if (env.sec) {
        std::vector<size_t> unreachable_acts;
        std::set_difference(used_acts.begin(), used_acts.end(), reachable_acts.begin(), reachable_acts.end(), std::back_inserter(unreachable_acts));
        const auto& cycles{cb_compute_sec(inst, used_first_archievers, unreachable_acts)};
        auto [secind, secval, secsize, secrhs, secsense, secizero] = cb_cpxconvert_sec_cut(inst, used_first_archievers, cycles);
        n_sec = cycles.size();
        CPX_HANDLE_CALL(log, CPXcallbackrejectcandidate(context, cycles.size(), secsize, secrhs, secsense, secizero, secind, secval));
        delete[] secizero;
        secizero = nullptr;
        delete[] secsense;
        secsense = nullptr;
        delete[] secrhs;
        secrhs = nullptr;
        delete[] secval;
        secval = nullptr;
        delete[] secind;
        secind = nullptr;
    }

    usercuts_lm += landmarks_list.size();
    usercuts_sec += n_sec;
    cand_time += env.timer.get_time() - start_time;
}

static void cpx_relax_callback(CPXCALLBACKCONTEXTptr& context, const hplus::instance& inst, const hplus::environment& env,
                               lm::thread_data& thread_data, const logger& log) {
    // if I reached the termination condition, exit
    if (CHECK_STOP()) return;

    // check if we are at the root node
    int nodedepth{-1};
    CPX_HANDLE_CALL(log, CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEDEPTH, &nodedepth));
    if (nodedepth != 0) return;

    double start_time{env.timer.get_time()};

    // get relaxation point (the first inst.m_opt variables are operators indicator variables)
    double* relax_point{new double[inst.m_opt]};
    double _{CPX_INFBOUND};
    CPX_HANDLE_CALL(log, CPXcallbackgetrelaxationpoint(context, relax_point, 0, inst.m_opt - 1, &_));

    // change objective function to vlm-det MIP model
    int* ind{new int[inst.m_opt]};
    for (size_t i = 0; i < inst.m_opt; i++) {
        ind[i] = i;
        if (relax_point[i] < HPLUS_EPSILON) relax_point[i] = 0;
    }
    CPX_HANDLE_CALL(log, CPXchgobj(thread_data.lmcutenv, thread_data.lmcutlp, inst.m_opt, ind, relax_point));
    delete[] ind;
    ind = nullptr;
#if !HPLUS_INTCHECK
    delete[] relax_point;
    relax_point = nullptr;
#endif

    // run vlm-det model
    CPX_HANDLE_CALL(log, CPXmipopt(thread_data.lmcutenv, thread_data.lmcutlp));
    int status{CPXgetstat(thread_data.lmcutenv, thread_data.lmcutlp)};
    if (status != CPXMIP_OPTIMAL && status != CPXMIP_OPTIMAL_TOL) {
        log.print_warn("CPXgetstat: %d.", status);
        delete[] relax_point;
        relax_point = nullptr;
        return;
    }

    // get obj val --> < 1: a landmark is violated, >= 1: no landmark is violated
    double cutval{-1};
    CPX_HANDLE_CALL(log, CPXgetobjval(thread_data.lmcutenv, thread_data.lmcutlp, &cutval));
    if (cutval >= 1 - HPLUS_EPSILON) {
#if HPLUS_INTCHECK
        delete[] relax_point;
        relax_point = nullptr;
#endif
        // [[DEBUG]] exiting callback
        log.print_warn(" LP  -  Landmark not violated by lp relax.  - cutval: %f.", cutval);
        return;
    }

    // here a landmark is violated

    // get the partition of facts
    double* facts_partition{new double[inst.n_opt]};
    CPX_HANDLE_CALL(log, CPXgetx(thread_data.lmcutenv, thread_data.lmcutlp, facts_partition, inst.m_opt, inst.m_opt + inst.n_opt - 1));
    binary_set reach(inst.n);
    for (const auto p : inst.var_rem) {
        if (facts_partition[inst.var_opt_conv[p]] >= HPLUS_CPX_INT_ROUNDING) reach.add(p);
    }
    delete[] facts_partition;
    facts_partition = nullptr;

    // filter the solution (actions with xa* = 0 might weaken the partition)
    bool found{false};
    while (!found) {
        for (const auto& p : reach) {
            found = false;
            for (const auto& act_i : inst.act_with_eff[p]) {
                if (reach.contains(inst.actions[act_i].pre)) {
                    found = true;
                    break;
                }
            }
            if (!found) reach.remove(p);
        }
    }
    INTCHECK_ASSERT_LOG(log, !reach.contains(inst.goal));

    // compute landmark
    ind = new int[inst.m_opt];
    double* val{new double[inst.m_opt]};
    static constexpr double rhs{1};
    static constexpr char sense{'G'};
    static constexpr int begin{0}, purgeable{CPX_USECUT_FORCE}, local{0};
    int nnz{0};
#if HPLUS_INTCHECK
    double cutvalpost{0};
#endif
    for (const auto act_i : inst.act_rem) {
        if (reach.contains(inst.actions[act_i].pre) && !reach.contains(inst.actions[act_i].eff)) {
            ind[nnz] = inst.act_opt_conv[act_i];
            val[nnz++] = 1;
#if HPLUS_INTCHECK
            cutvalpost += relax_point[inst.act_opt_conv[act_i]];
#endif
        }
    }

    // [[DEBUG]] see what is the minimum cost of the actions in the landmark
    unsigned int costcheck{inst.actions[inst.act_cpxtoidx[ind[0]]].cost};
    for (int i = 0; i < nnz; i++) costcheck = std::min(costcheck, inst.actions[inst.act_cpxtoidx[ind[i]]].cost);

    // [[DEBUG]] check that the landmark violation corresponds to the one computed by the vlm-det model
#if HPLUS_INTCHECK
    ASSERT_LOG(log, abs(cutval - cutvalpost) < HPLUS_EPSILON);
    delete[] relax_point;
    relax_point = nullptr;
#endif

    // add the landmark as a cut
    CPX_HANDLE_CALL(log, CPXcallbackaddusercuts(context, 1, nnz, &rhs, &sense, &begin, ind, val, &purgeable, &local));

    // cleanup and logging
    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    thread_data.usercuts_lm++;
    double lb{-1};
    CPX_HANDLE_CALL(log, CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &lb));
    log.print_warn(" LP  -  Landmark violated by lp relaxation  - size: %3d, min-cut: %1.3f, min-cost: %3d, lb: %3.3f.", nnz, cutval, costcheck, lb);
    thread_data.relax_time += env.timer.get_time() - start_time;
}

int CPXPUBLIC lm::cpx_callback_hub(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle) {
    auto& [inst, env, log, _1, _2, thread_data]{*static_cast<lm::cpx_callback_user_handle*>(user_handle)};
    int thread_id{-1};
    CPX_HANDLE_CALL(log, CPXcallbackgetinfoint(context, CPXCALLBACKINFO_THREADID, &thread_id));

    switch (context_id) {
        case CPX_CALLBACKCONTEXT_CANDIDATE:
            cpx_cand_callback(context, inst, env, thread_data[thread_id].cand_time, thread_data[thread_id].usercuts_lm,
                              thread_data[thread_id].usercuts_sec, log);
            break;
        case CPX_CALLBACKCONTEXT_RELAXATION:
            cpx_relax_callback(context, inst, env, thread_data[thread_id], log);
            break;
        default:
            log.raise_error("Unhandled callback context: %ld.", context_id);
    }

    return 0;
}

void lm::create_thread_data(const hplus::instance& inst, const hplus::environment& env, lm::cpx_callback_user_handle& callback_data,
                            const logger& log) {
    cpx_create_lmcut_model(inst, log, callback_data.lmcutenv, callback_data.lmcutlp);
    for (int i = 0; i < env.threads; i++) {
        int cpxerror{-1};
        callback_data.thread_specific_data.emplace_back(
            lm::thread_data{.usercuts_lm = 0, .usercuts_sec = 0, .cand_time = 0, .relax_time = 0, .lmcutenv = nullptr, .lmcutlp = nullptr});
        callback_data.thread_specific_data[i].lmcutenv = CPXopenCPLEX(&cpxerror);
        CPX_HANDLE_CALL(log, cpxerror);
        CPXreadcopyparam(callback_data.thread_specific_data[i].lmcutenv, HPLUS_CPLEX_OUTPUT_DIR "/.prm");
        callback_data.thread_specific_data[i].lmcutlp =
            CPXcloneprob(callback_data.thread_specific_data[i].lmcutenv, callback_data.lmcutlp, &cpxerror);
    }
}

void lm::sync_and_close_threads(const hplus::environment& env, hplus::statistics& stats, lm::cpx_callback_user_handle& callback_data,
                                const logger& log) {
    for (auto& data : callback_data.thread_specific_data) {
        stats.cand_callback += data.cand_time;
        stats.relax_callback += data.relax_time;
        stats.usercuts_lm += data.usercuts_lm;
        stats.usercuts_sec += data.usercuts_sec;
        if (env.fract_cuts) cpx_close_lmcut_model(data.lmcutenv, data.lmcutlp, log);
    }
    if (env.fract_cuts) cpx_close_lmcut_model(callback_data.lmcutenv, callback_data.lmcutlp, log);
}

// ##################################################################### //
// ############################# BASE MODEL ############################ //
// ##################################################################### //

void lm::build_cpx_model(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log,
                         hplus::statistics& stats) {
    PRINT_VERBOSE(log, "Building LM model.");

    const auto stopchk1 = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

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

    // Constraint C1
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

    // Constraint C2
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

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    int ind2[3];
    double val2[3];

    // Constraint C3
    for (const auto& act_i : inst.act_rem) {
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            ind2[0] = get_act_idx(act_i);
            val2[0] = -1;
            // if the first adder was fixed, we can directly fix the action insthead of adding the constraint
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                CPX_HANDLE_CALL(log, CPXchgbds(cpxenv, cpxlp, 1, ind2, &fix, &one));
                continue;
            }
            // if the first adder was eliminated, we can skip the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            ind2[1] = get_fa_idx(act_i, var_count);
            val2[1] = 1;
            stats.nconst_base++;
            CPX_HANDLE_CALL(log, CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind2, val2, nullptr, nullptr));
        }
        stopchk1();
    }

    // ~~~~~~~~~ Modeling acyclicity ~~~~~~~~~ //
    // left to the callbacks

    if (env.write_lp) CPX_HANDLE_CALL(log, CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

// ##################################################################### //
// ############################# WARM START ############################ //
// ##################################################################### //

void lm::post_cpx_warmstart(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Posting warm start to LM model.");
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

            size_t cpx_fad_idx = inst.m_opt + inst.fadd_cpx_start[inst.act_opt_conv[act_i]] + var_count;
            cpx_sol_val[cpx_fad_idx] = 1;
            size_t cpx_var_idx = inst.m_opt + inst.n_fadd + inst.var_opt_conv[var_i];
            cpx_sol_val[cpx_var_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
    }

    CPX_HANDLE_CALL(log, CPXaddmipstarts(cpxenv, cpxlp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));

    delete[] cpx_sol_ind;
    cpx_sol_ind = nullptr;
    delete[] cpx_sol_val;
    cpx_sol_val = nullptr;
}

// ##################################################################### //
// ########################### STORE SOLUTION ########################## //
// ##################################################################### //

void lm::store_cpx_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Storing LM solution.");

    double* plan{new double[inst.m_opt]};
    CPX_HANDLE_CALL(log, CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt - 1));

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
    hplus::solution sol{
        solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [&inst](const unsigned int acc, const size_t index) {
            return acc + inst.actions[index].cost;
        }))};
    hplus::update_sol(inst, sol, log);
}
