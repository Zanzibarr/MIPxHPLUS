#include "cand_callback.hpp"

#include <atomic>
#include <deque>
#include <numeric>

/**
 * Analyze the current candidate point: generate data structures to be used in later parts of the callback
 */
[[nodiscard]]
static std::tuple<std::vector<unsigned int>, binary_set, std::vector<unsigned int>, binary_set, std::vector<binary_set>> candidatepoint_info(
    const hplus::instance& inst, const std::vector<double>& xstar) {
    binary_set used_actions(inst.m);
    std::vector<unsigned int> reachable_action_sequence;
    binary_set unreachable_actions(inst.m);
    std::vector<unsigned int> unused_actions;
    binary_set reachable_state(inst.n);
    std::vector<binary_set> used_first_achievers(inst.m, binary_set(inst.n));

    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        // Divide actions in used or unused
        if (xstar[act_i] > HPLUS_CPX_INT_ROUNDING) {
            used_actions.add(act_i);
            unreachable_actions.add(act_i);

            // Check for used first achievers
            for (unsigned int i = 0; i < inst.actions[act_i].eff_sparse.size(); i++) {
                unsigned int idx{inst.m + inst.fadd_cpx_start[act_i] + i};
                if (xstar[idx] > HPLUS_CPX_INT_ROUNDING) used_first_achievers[act_i].add(inst.actions[act_i].eff_sparse[i]);
            }
        } else
            unused_actions.push_back(act_i);
    }

    // Only actions with no preconditions can be applied at first
    std::deque<unsigned int> queue;
    for (const auto& act_i : used_actions) {
        if (inst.actions[act_i].pre_sparse.empty()) queue.push_back(act_i);
    }

    // Compute the set of reachable facts and actions that can actually be used (without using infeasible loops)
    while (!queue.empty()) {
        // This actions is now applicable, store it as such
        unsigned int act_i{queue.front()};
        queue.pop_front();
        reachable_action_sequence.push_back(act_i);
        unreachable_actions.remove(act_i);

        // Compute new state
        const auto& new_state{reachable_state | inst.actions[act_i].eff};
        if (new_state == reachable_state) continue;  // No change in state -> no new applicable actions

        // Find new applicable actions
        for (const auto& p : inst.actions[act_i].eff_sparse) {
            if (reachable_state[p]) continue;  // Already in state, skip

            // Check for actions that can now be applied
            for (const auto& act_j : inst.act_with_pre[p]) {
                if (!used_actions[act_j]) continue;
                if (new_state.contains(inst.actions[act_j].pre) && std::find(queue.begin(), queue.end(), act_j) == queue.end())
                    queue.push_back(act_j);
            }
        }
        // Update the reachable state
        reachable_state = std::move(new_state);
    }

    return {reachable_action_sequence, unreachable_actions, unused_actions, reachable_state, used_first_achievers};
}

/**
 * This method is called when there's a solution that reaches the goal, but there are unreachable actions, so there's a loop: we can reject this
 * solution and easily compute the better solution without using the unreachable actions (and other if some optimizations are applicable)
 */
static void reject_with_new_sol(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
                                const std::vector<unsigned int>& reachable_action_sequence) {
    unsigned int ncols{inst.m + inst.nfadd + inst.n};
    std::vector<int> ind(ncols);
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(ncols, 0.0);
    hplus::solution sol{.sequence = std::vector<unsigned int>(), .cost = 0, .updating = false};

    // Compute a better solution (we already know that we can reach the goal)
    binary_set state{inst.n};
    unsigned int cost{0};
    for (const auto& act_i : reachable_action_sequence) {
        if (state.contains(inst.actions[act_i].eff))
            continue;  // If this action has no effects on this current state, we can optimize the solution by ignoring this action
        sol.sequence.push_back(act_i);
        sol.cost += inst.actions[act_i].cost;
        val[act_i] = 1;
        for (unsigned int i = 0; i < inst.actions[act_i].eff_sparse.size(); i++) {
            if (state[inst.actions[act_i].eff_sparse[i]]) continue;
            unsigned int fadd_idx = inst.m + inst.fadd_cpx_start[act_i] + i;
            val[fadd_idx] = 1;
            unsigned int var_idx = inst.m + inst.nfadd + inst.actions[act_i].eff_sparse[i];
            val[var_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
        cost += inst.actions[act_i].cost;
        if (state.contains(inst.goal)) break;  // If we already reached the goal, we can exit early (ignore all other applicable actions)
    }

    // Store the new solution
    hplus::update_sol(exec, inst, sol, stats);

    // Give CPLEX the better solution
    CPX_HANDLE_CALL(CPXcallbackpostheursoln(context, ncols, ind.data(), val.data(), static_cast<double>(cost), CPXCALLBACKSOLUTION_NOCHECK));

    constexpr int rejind{0}, rejbegin{0};
    constexpr double rejval{0.0}, rejrhs{0.0};
    constexpr char rejsense{'E'};

    // Reject the current solution
    CPX_HANDLE_CALL(CPXcallbackrejectcandidate(context, 0, 0, &rejrhs, &rejsense, &rejbegin, &rejind, &rejval));
}

void callbacks::candidate_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
                                   unsigned int& usercuts_lm, unsigned int& usercuts_sec, double& cand_time) {
    double start_time = GET_TIME();

    std::vector<double> xstar(inst.m + inst.nfadd);
    double cost{CPX_INFBOUND};
    CPX_HANDLE_CALL(CPXcallbackgetcandidatepoint(context, xstar.data(), 0, inst.m + inst.nfadd - 1, &cost));

    const auto& [reachable_action_sequence, unreachable_actions, unused_actions, reachable_state, used_first_achievers] =
        candidatepoint_info(inst, xstar);

    // If the solution is feasible, we don't have to cut it
    if (unreachable_actions.empty()) {
        ASSERT(reachable_state.contains(inst.goal));
        unsigned int cost{0};
        for (const auto& act_i : reachable_action_sequence) cost += inst.actions[act_i].cost;
        // Check wether this is a better solution than the one we already have
        hplus::update_sol(exec, inst, hplus::solution{reachable_action_sequence, cost, false}, stats);
        cand_time += GET_TIME() - start_time;
        return;
    }

    // The feasibility of the solution is determined by the existance of unreachable actions -> the goal might be reachable even if some actions are
    // unreachable, in this case we know that we can obtain the same (or a better) solution by using a strict subset of used actions
    // NOTE!: This rejects the candidate solution, BUT we post another one that is guaranteed to be a better one (either only less actions, or even a
    // better incumbent)
    if (reachable_state.contains(inst.goal)) {
        reject_with_new_sol(context, exec, inst, stats, reachable_action_sequence);
        return;
    }

    // Here we know that there are unreachable actions, and the goal is unreachable -> we need to find some cuts

    // exec.cand_cuts is a string containing one letter per type of cut to be applied
    // -> f : frontier landmark cuts
    // -> c : complementary landmark cuts
    // -> s : SEC
    if (exec.cand_cuts.find('f') != std::string::npos) usercuts_lm += cand_cuts::frontier_lm(context, inst, unused_actions, reachable_state);
    if (exec.cand_cuts.find('c') != std::string::npos)
        usercuts_lm += cand_cuts::complementary_lm(context, inst, unreachable_actions, unused_actions, reachable_state);
    if (exec.cand_cuts.find('s') != std::string::npos) usercuts_sec += cand_cuts::sec(context, inst, unreachable_actions, used_first_achievers);

    cand_time += GET_TIME() - start_time;
}