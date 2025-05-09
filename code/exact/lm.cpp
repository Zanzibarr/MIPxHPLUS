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
// ====================== CALLBACK ====================== //
// ====================================================== //

static void cpx_cand_callback(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const hplus::environment& env, hplus::statistics& stats,
                              const logger& log) {
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

    pthread_mutex_lock(&(stats.callback_time_mutex));
    stats.nusercuts += landmarks_list.size() + n_sec;
    stats.callback += env.timer.get_time() - start_time;
    pthread_mutex_unlock(&(stats.callback_time_mutex));
}

static void cpx_relax_callback(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, hplus::statistics& stats, const logger& log) {
    int nodeid{-1};
    CPX_HANDLE_CALL(log, CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEUID, &nodeid));
    if (nodeid != 0) return;

    // LM
    double* xstar{new double[inst.m_opt]};
    double _{CPX_INFBOUND};
    CPX_HANDLE_CALL(log, CPXcallbackgetrelaxationpoint(context, xstar, 0, inst.m_opt - 1, &_));

    std::vector<double> var_values(inst.n, 0), act_values(inst.m_opt, -1);
    std::list<size_t> actions_queue;
    binary_set state(inst.n), act_in_queue(inst.m_opt);

    // initial actions (no preconditions)
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (xstar[i] <= HPLUS_EPSILON) continue;
        if (inst.actions[inst.act_cpxtoidx[i]].pre_sparse.empty()) {
            actions_queue.push_back(i);
            act_in_queue.add(i);
        }
    }

    while (!actions_queue.empty()) {
        size_t act_i_cpx{actions_queue.front()};
        actions_queue.pop_front();
        act_in_queue.remove(act_i_cpx);

        // choose value to set to the action
        double act_val{act_values[act_i_cpx] < 0 ? 0 : act_values[act_i_cpx]};
        act_values[act_i_cpx] = xstar[act_i_cpx];

        if (act_values[act_i_cpx] <= HPLUS_EPSILON) continue;

        for (const auto& var_i : inst.actions[inst.act_cpxtoidx[act_i_cpx]].pre_sparse)
            act_values[act_i_cpx] = std::min(act_values[act_i_cpx], var_values[var_i]);

        if (act_values[act_i_cpx] <= HPLUS_EPSILON) continue;

        // update values of the effects
        double diff{act_values[act_i_cpx] - act_val};
        if (diff < HPLUS_EPSILON && state.contains(inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff)) continue;
        state |= inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff;
        for (const auto& var_i : inst.actions[inst.act_cpxtoidx[act_i_cpx]].eff_sparse) {
            var_values[var_i] += diff;
            for (const auto& act_j : inst.act_with_pre[var_i]) {
                if (!state.contains(inst.actions[act_j].pre)) continue;
                if (act_in_queue[inst.act_opt_conv[act_j]]) continue;
                actions_queue.push_back(inst.act_opt_conv[act_j]);
                act_in_queue.add(inst.act_opt_conv[act_j]);
            }
        }
    }

    // compute reachable state
    binary_set reachable_state(inst.n);
    for (size_t i = 0; i < inst.n; i++) {
        if (var_values[i] >= 1 - HPLUS_EPSILON) reachable_state.add(i);
    }

    if (reachable_state.contains(inst.goal)) {
        delete[] xstar;
        xstar = nullptr;
        return;
    }

    // compute landmark
    double cut_section = 0;
    std::vector<size_t> landmark;
    landmark.reserve(inst.m_opt);
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (reachable_state.contains(inst.actions[inst.act_cpxtoidx[i]].pre) && !reachable_state.contains(inst.actions[inst.act_cpxtoidx[i]].eff)) {
            landmark.push_back(i);
            cut_section += xstar[i];
        }
    }

    delete[] xstar;
    xstar = nullptr;

    if (cut_section >= 1 - HPLUS_EPSILON) return;

    int* ind = new int[inst.m_opt];
    double* val{new double[inst.m_opt]};
    int nnz{0};
    static constexpr int begin{0}, purgeable{CPX_USECUT_FORCE}, local{0};
    static constexpr double rhs{1};
    static constexpr char sense{'G'};
    for (const auto& act_i_cpx : landmark) {
        ind[nnz] = act_i_cpx;
        val[nnz++] = 1;
    }

    ASSERT_LOG(log, nnz != 0);
    CPX_HANDLE_CALL(log, CPXcallbackaddusercuts(context, 1, nnz, &rhs, &sense, &begin, ind, val, &purgeable, &local));

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    pthread_mutex_lock(&(stats.callback_time_mutex));
    stats.nusercuts++;
    pthread_mutex_unlock(&(stats.callback_time_mutex));

    // SEC
    // {
    //     double* xstar = new double[inst.n_fadd];
    //     double _{-1};
    //     CPX_HANDLE_CALL(log, CPXcallbackgetrelaxationpoint(context, xstar, inst.m_opt, inst.m_opt + inst.n_fadd, &_));

    //     std::vector<std::set<size_t>> graph(inst.m_opt);
    //     for (size_t act_i_cpx = 0; act_i_cpx < inst.m_opt; act_i_cpx++) {
    //         const auto& act{inst.actions[inst.act_cpxtoidx[act_i_cpx]]};
    //         for (size_t idx = 0; idx < act.eff_sparse.size(); idx++) {
    //             size_t var_i{act.eff_sparse[idx]};
    //             if (xstar[inst.fadd_cpx_start[act_i_cpx] + idx] < HPLUS_EPSILON) continue;
    //             for (const auto& act_j : inst.act_with_pre[var_i]) graph[act_i_cpx].insert(inst.act_opt_conv[act_j]);
    //         }
    //     }

    //     std::vector<std::vector<size_t>> cycles;
    //     std::unordered_set<size_t> cycle_found;

    //     for (size_t start = 0; start < graph.size(); start++) {
    //         // Skip if we already found a cycle containing this node
    //         if (cycle_found.count(start)) continue;

    //         std::vector<size_t> path;
    //         std::unordered_set<size_t> in_path;
    //         std::unordered_set<size_t> visited;

    //         // Run DFS to find a cycle starting from this node
    //         if (cycles_dfs(graph, start, start, path, in_path, visited, cycles, cycle_found)) {
    //             // Mark all nodes in the found cycle as having a cycle
    //             for (size_t node : cycles.back()) {
    //                 cycle_found.insert(node);
    //             }
    //         }
    //     }
    //     const size_t max_size = cycles.size() * inst.n_opt;

    //     if (max_size == 0) {
    //         delete[] xstar;
    //         xstar = nullptr;
    //         return;
    //     }

    //     int* secind{new int[max_size]};
    //     double* secval{new double[max_size]};
    //     int nnz = 0;
    //     double* secrhs{new double[cycles.size()]};
    //     char* secsense{new char[cycles.size()]};
    //     int* secizero{new int[cycles.size()]};
    //     int *secpurgeable{new int[cycles.size()]}, *seclocal{new int[cycles.size()]};
    //     int sec_counter{0};

    //     for (const auto& cycle : cycles) {
    //         secsense[sec_counter] = 'L';
    //         secizero[sec_counter] = nnz;
    //         secrhs[sec_counter] = -1;
    //         secpurgeable[sec_counter] = CPX_USECUT_FORCE;
    //         seclocal[sec_counter] = 0;

    //         // first archievers in the cycle
    //         for (size_t i = 0; i < cycle.size() - 1; i++) {
    //             for (size_t idx = 0; idx < inst.actions[inst.act_cpxtoidx[cycle[i]]].eff_sparse.size(); idx++) {
    //                 size_t p{inst.actions[inst.act_cpxtoidx[cycle[i]]].eff_sparse[idx]};
    //                 if (xstar[inst.fadd_cpx_start[cycle[i]] + idx] < HPLUS_EPSILON || !inst.actions[inst.act_cpxtoidx[cycle[i + 1]]].pre[p])
    //                 continue;

    //                 secind[nnz] = inst.m_opt + inst.fadd_cpx_start[cycle[i]] + idx;
    //                 secval[nnz++] = 1;
    //                 secrhs[sec_counter]++;
    //             }
    //         }
    //         int var_count{-1};
    //         for (const auto& p : inst.actions[inst.act_cpxtoidx[cycle[cycle.size() - 1]]].eff_sparse) {
    //             var_count++;
    //             if (xstar[inst.fadd_cpx_start[cycle[cycle.size() - 1]] + var_count] < HPLUS_EPSILON ||
    //                 !inst.actions[inst.act_cpxtoidx[cycle[0]]].pre[p])
    //                 continue;

    //             secind[nnz] = inst.m_opt + inst.fadd_cpx_start[cycle[cycle.size() - 1]] + var_count;
    //             secval[nnz++] = 1;
    //             secrhs[sec_counter]++;
    //         }

    //         ASSERT_LOG(log, secrhs[sec_counter] >= 0);
    //         sec_counter++;
    //     }
    //     CPX_HANDLE_CALL(log, CPXcallbackaddusercuts(context, cycles.size(), nnz, secrhs, secsense, secizero, secind, secval, secpurgeable,
    //     seclocal)); delete[] seclocal; seclocal = nullptr; delete[] secpurgeable; secpurgeable = nullptr; delete[] secizero; secizero = nullptr;
    //     delete[] secsense;
    //     secsense = nullptr;
    //     delete[] secrhs;
    //     secrhs = nullptr;
    //     delete[] secval;
    //     secval = nullptr;
    //     delete[] secind;
    //     secind = nullptr;
    //     delete[] xstar;
    //     xstar = nullptr;

    //     pthread_mutex_lock(&(stats.callback_time_mutex));
    //     stats.nusercuts += cycles.size();
    //     pthread_mutex_unlock(&(stats.callback_time_mutex));
    // }
}

int CPXPUBLIC lm::cpx_callback_hub(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle) {
    auto& [inst, env, stats, log]{*static_cast<lm::cpx_callback_user_handle*>(user_handle)};

    switch (context_id) {
        case CPX_CALLBACKCONTEXT_CANDIDATE:
            cpx_cand_callback(context, inst, env, stats, log);
            break;
        case CPX_CALLBACKCONTEXT_RELAXATION:
            cpx_relax_callback(context, inst, stats, log);
            break;
        default:
            log.raise_error("Unhandled callback context: %ld.", context_id);
    }

    return 0;
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
