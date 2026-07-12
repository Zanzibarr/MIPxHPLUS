#include <binary_set.hxx>

#include "cli_descriptions.hpp"
#include "constants.hpp"
#include "solver.hpp"
#include "utils.hpp"

auto CPXPUBLIC Solver::hplus_callback_hub_(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle) -> int {
    auto* solver = static_cast<Solver*>(userhandle);
    switch (contextid) {
        case CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS:
            solver->hplus_progress_callback_(context);
            break;
        case CPX_CALLBACKCONTEXT_CANDIDATE:
            solver->hplus_candidate_callback_(context);
            break;
        case CPX_CALLBACKCONTEXT_RELAXATION:
            solver->hplus_relaxation_callback_(context);
            break;
        default:
            solver->logger_[FATAL] << std::format("Unhandled CPLEX callback context: {}", contextid);
    }

    return 0;
}

void Solver::hplus_progress_callback_(CPXCALLBACKCONTEXTptr context) {
    double best_lb{-1};
    call_cplex(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));

    // From CPLEX Documentation:
    // Important: This bound may be infinite (-CPX_INFBOUND or CPX_INFBOUND, depending on the objective sense) if no bound has been computed yet, or
    // if no bound is available for the calling thread.
    // Warning: Note that the best bound may exceed the value of the best feasible solution when
    // optimality is proven.
    // We update the best bound only if we know it is a valid one (hence lower than the best incumbent we know)
    if (is_lw_or_eq_double(best_lb, global_.best_incumbent)) {
        try_update_best_bound_(best_lb);
    }

    // TODO: [lb_rootnode-v2] Track the root-node lower bound here, gated on NODECOUNT, instead of in the relaxation callback (which is not guaranteed
    // to fire below the root: see elevators-sat11-strips-p19, 471 nodes with every relaxation callback at depth 0). nodecount == 0 means the root is
    // still being processed; it keeps growing across restarts, so post-restart root reprocessing is deliberately NOT counted as root.
    // NOTE: this intentionally diverges from the old implementation (depth-based + final-bound fallback), so enable it only once the regression
    // campaign against the old codebase is over. When enabling: uncomment this block, delete the [lb_rootnode-v1] block in
    // hplus_relaxation_callback_ and switch the fallback in hplus_cplex_gather_info_ to its [lb_rootnode-v2] version.
    // CPXLONG nodecount{-1};
    // call_cplex(CPXcallbackgetinfolong(context, CPXCALLBACKINFO_NODECOUNT, &nodecount));
    // if (nodecount == 0) {
    //     // Still at the root: track the highest bound seen so far
    //     double current = global_.relax_last_root_lb.load();
    //     while (best_lb > current && !global_.relax_last_root_lb.compare_exchange_weak(current, best_lb)) {
    //     }
    // } else {
    //     // First progress event after leaving the root: freeze the root bound
    //     bool expected = false;
    //     if (global_.relax_lb_rootnode_recorded.compare_exchange_strong(expected, true)) {
    //         stats_.gauge_record<"lb_rootnode">(global_.relax_last_root_lb);
    //     }
    // }

    // Sometimes our best incumbent (obtained in a callback) doesn't get processed immediatelly by CPLEX... if we realize that we can already prove
    // optimality, we can send an early exit signal
    // ATTENTION: This breaks determinism
    if (is_gr_or_eq_double(global_.best_bound, global_.best_incumbent)) {
        // We already have the optimality proof... we don't need to do anything else...
        GLOBAL_TERMINATE_CONDITION = 1;  // We cannot throw a C++ exception since CPLEX has C code and it doesn't know how to handle it...
        return;
    }
}

// ##################################################################### //
// ######################### CANDIDATE CALLBACK ######################## //
// ##################################################################### //

void Solver::hplus_candidate_callback_(CPXCALLBACKCONTEXTptr context) {
    auto _callback_timer = scoped_timer("cand_callback");
    stats_.counter_inc<"cand_calls">();

    const unsigned int size = inst_.m + inst_.nfadd;
    if (local_.cand_xstar.size() != size) {
        local_.cand_xstar = std::vector<double>(size);
    }
    double cost{CPX_INFBOUND};
    call_cplex(CPXcallbackgetcandidatepoint(context, local_.cand_xstar.data(), 0, static_cast<int>(size - 1), &cost));

    hplus_candidate_get_info_();

    // If the solution is feasible, we don't have to cut it
    if (local_.cand_unreachable_actions.empty()) {
        myassert(local_.cand_reachable_state.superset_of(inst_.goal),
                 "Solution with no unreachable actions that doesn't reach the goal has been found in the candidate callback");
        // Check wether this is a better solution than the one we already have
        try_update_best_incumbent_(local_.cand_reachable_action_sequence, cost);
        return;
    }

    // The feasibility of the solution is determined by the existance of unreachable actions -> the goal might be reachable even if some actions are
    // unreachable, in this case we know that we can obtain the same (or a better) solution by using a strict subset of used actions
    // NOTE!: This rejects the candidate solution, BUT we post another one that is guaranteed to be a better one (either only less actions, or even a
    // better incumbent)
    if (local_.cand_reachable_state.superset_of(inst_.goal)) {
        hplus_reject_candidate_with_new_sol_(context, local_.cand_reachable_action_sequence);
        return;
    }

    // Here we know that there are unreachable actions, and the goal is unreachable -> we need to find some cuts

    const auto& cand_cuts = params_.get<cli_desc::cand_cuts, std::string>();
    if (cand_cuts == "sec") {
        hplus_separate_cand_sec_cut_(context);
    } else if (cand_cuts == "lm-f") {
        hplus_separate_cand_lmfront_cut_(context);
    } else if (cand_cuts == "lm-c") {
        hplus_separate_cand_lmcomp_cut_(context);
    } else if (cand_cuts == "lmcut") {
        hplus_separate_cand_lmcut_cut_(context, '0');
    } else if (cand_cuts == "lmcut-g") {
        hplus_separate_cand_lmcut_cut_(context, 'g');
    } else if (cand_cuts == "lmcut-c") {
        hplus_separate_cand_lmcut_cut_(context, 'c');
    } else {
        logger_[FATAL] << std::format("Unhandled {} parameter in candidate callback: {}", cli_desc::cand_cuts.view(), cand_cuts);
    }
}

void Solver::hplus_candidate_get_info_() {
    if (local_.cand_used_actions.capacity() != inst_.m) {
        local_.cand_used_actions = BinarySet(inst_.m);
    } else {
        local_.cand_used_actions.clear();
    }
    local_.cand_reachable_action_sequence.clear();
    local_.cand_unreachable_actions.clear();
    local_.cand_unused_actions.clear();
    if (local_.cand_reachable_state.capacity() != inst_.n) {
        local_.cand_reachable_state = BinarySet(inst_.n);
    } else {
        local_.cand_reachable_state.clear();
    }

    for (unsigned int act_i = 0; act_i < inst_.m; ++act_i) {
        // Divide actions in used or unused
        if (local_.cand_xstar[act_i] > constants::cpx_int_rounding) {
            local_.cand_used_actions.add(act_i);
            local_.cand_unreachable_actions.push_back(act_i);

            // Check for used first achievers
            for (unsigned int i = 0; i < inst_.actions[act_i].eff_sparse.size(); i++) {
                unsigned int idx{inst_.m + global_.hplus_fadd_cpx_start[act_i] + i};
                if (local_.cand_xstar[idx] > constants::cpx_int_rounding) {
                }
            }
        } else {
            local_.cand_unused_actions.push_back(act_i);
        }
    }

    // Only actions with no preconditions can be applied at first
    std::deque<unsigned int> queue;
    std::unordered_set<unsigned int> act_in_queue;
    for (const auto act_i : local_.cand_used_actions) {
        if (inst_.actions[act_i].pre_sparse.empty()) {
            queue.push_back(act_i);
            act_in_queue.insert(act_i);
        }
    }

    // Compute the set of reachable facts and actions that can actually be used (without using infeasible loops)
    while (!queue.empty()) {
        // This actions is now applicable, store it as such
        unsigned int act_i{queue.front()};
        queue.pop_front();
        act_in_queue.erase(act_i);
        local_.cand_reachable_action_sequence.push_back(act_i);
        local_.cand_unreachable_actions.erase(local_.cand_unreachable_actions.begin() + sorted_find(local_.cand_unreachable_actions, act_i));

        // Compute new state
        if (bs_contains(local_.cand_reachable_state, inst_.actions[act_i].eff_sparse)) {
            continue;
        }
        std::vector<unsigned int> new_eff(inst_.actions[act_i].eff_sparse.begin(), inst_.actions[act_i].eff_sparse.end());
        std::erase_if(new_eff, [](const auto val) { return local_.cand_reachable_state[val]; });
        local_.cand_reachable_state |= new_eff;

        // Find new applicable actions
        for (const auto& fact : new_eff) {
            // Check for actions that can now be applied
            for (const auto& act_j : global_.act_with_pre[fact]) {
                if (!local_.cand_used_actions[act_j]) {
                    continue;
                }
                if (bs_contains(local_.cand_reachable_state, inst_.actions[act_j].pre_sparse) && !act_in_queue.contains(act_j)) {
                    queue.push_back(act_j);
                    act_in_queue.insert(act_j);
                }
            }
        }
    }
}

void Solver::hplus_reject_candidate_with_new_sol_(CPXCALLBACKCONTEXTptr context, const std::vector<unsigned int>& solution) {
    unsigned int ncols{inst_.m + inst_.nfadd + inst_.n};
    std::vector<int> ind(ncols);
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(ncols, 0.0);
    std::vector<unsigned int> new_sol;
    double cost{0};

    // Compute a better solution (we already know that we can reach the goal)
    BinarySet state{inst_.n};
    for (const auto& act_i : solution) {
        if (bs_contains(state, inst_.actions[act_i].eff_sparse)) {
            continue;  // If this action has no effects on this current state, we can optimize the solution by ignoring this action
        }
        new_sol.push_back(act_i);
        cost += inst_.actions[act_i].cost;
        val[act_i] = 1;
        for (unsigned int i = 0; i < inst_.actions[act_i].eff_sparse.size(); i++) {
            if (state[inst_.actions[act_i].eff_sparse[i]]) {
                continue;
            }
            unsigned int fadd_idx = inst_.m + global_.hplus_fadd_cpx_start[act_i] + i;
            val[fadd_idx] = 1;
            unsigned int var_idx = inst_.m + inst_.nfadd + inst_.actions[act_i].eff_sparse[i];
            val[var_idx] = 1;
        }
        state |= inst_.actions[act_i].eff_sparse;
        if (state.superset_of(inst_.goal)) {
            break;  // If we already reached the goal, we can exit early (ignore all other applicable actions)
        }
    }

    // Store the new solution
    try_update_best_incumbent_(new_sol, cost);

    // Give CPLEX the better solution
    // ATTENTION: This breaks determinism
    call_cplex(
        CPXcallbackpostheursoln(context, static_cast<int>(ncols), ind.data(), val.data(), static_cast<double>(cost), CPXCALLBACKSOLUTION_NOCHECK));

    constexpr int rejind{0};
    constexpr int rejbegin{0};
    constexpr double rejval{0.0};
    constexpr double rejrhs{0.0};
    constexpr char rejsense{'E'};

    // Reject the current solution
    call_cplex(CPXcallbackrejectcandidate(context, 0, 0, &rejrhs, &rejsense, &rejbegin, &rejind, &rejval));
}

// ##################################################################### //
// ######################## RELAXATION CALLBACK ######################## //
// ##################################################################### //

void Solver::hplus_relaxation_callback_(CPXCALLBACKCONTEXTptr context) {
    // Get the lp relaxation (first ever LP solution)
    static std::once_flag lb_once;
    std::call_once(lb_once, [&] {
        double best_lb{-1};
        call_cplex(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));
        stats_.gauge_record<"lb_relaxation">(best_lb);
    });

    int nodeuid{-1};
    int nodedepth{-1};
    call_cplex(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEUID, &nodeuid));
    call_cplex(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEDEPTH, &nodedepth));

    // TODO: [lb_rootnode-v1] Track the highest LB seen at the root (root = depth 0, matching the old implementation: after a restart the rebuilt root
    // has a fresh uid but depth 0, so its reprocessing counts as root); record on first transition to a non-root node. DELETE this whole if/else
    // block when enabling the [lb_rootnode-v2] tracking in hplus_progress_callback_ (otherwise the two race for relax_lb_rootnode_recorded and this
    // one wins with the wrong semantics).
    if (nodedepth == 0) {
        double best_lb{-1};
        call_cplex(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));
        double current = global_.relax_last_root_lb.load();
        while (best_lb > current && !global_.relax_last_root_lb.compare_exchange_weak(current, best_lb)) {
        }
    } else {
        bool expected = false;
        if (global_.relax_lb_rootnode_recorded.compare_exchange_strong(expected, true)) {
            stats_.gauge_record<"lb_rootnode">(global_.relax_last_root_lb);
        }
    }

    // Visit each node (except for root node) at most once
    thread_local static std::unordered_set<int> visited_nodes;
    if (nodedepth != 0 && visited_nodes.contains(nodeuid)) {
        return;
    }
    visited_nodes.insert(nodeuid);

    const auto& relax_cuts = params_.get<cli_desc::relax_cuts, std::string>();
    if (relax_cuts == "0") {
        return;
    }

    // ~~~~~~~~~ Callback starts here ~~~~~~~~ //

    auto _callback_timer = scoped_timer("relax_callback");
    stats_.counter_inc<"relax_calls">();

    const unsigned int size = inst_.m + inst_.nfadd;
    if (local_.relax_xstar.size() != size) {
        local_.relax_xstar = std::vector<double>(size);
    }
    call_cplex(CPXcallbackgetrelaxationpoint(context, local_.relax_xstar.data(), 0, static_cast<int>(size - 1), nullptr));

    // Fix numerical errors
    for (auto& val : local_.relax_xstar) {
        if (is_lw_or_eq_double(val, 0)) {
            val = 0;
        } else if (is_gr_or_eq_double(val, 1)) {
            val = 1;
        }
    }

    if (relax_cuts == "sec") {
        hplus_separate_relax_sec_cut_(context);
    } else if (relax_cuts == "lm") {
        hplus_separate_relax_lm_cut_(context, false);
    } else if (relax_cuts == "lm-m") {
        hplus_separate_relax_lm_cut_(context, true);
    } else if (relax_cuts == "lmcut") {
        hplus_separate_relax_lmcut_cut_(context, '0');
    } else if (relax_cuts == "lmcut-g") {
        hplus_separate_relax_lmcut_cut_(context, 'g');
    } else if (relax_cuts == "lmcut-c") {
        hplus_separate_relax_lmcut_cut_(context, 'c');
    } else {
        logger_[FATAL] << std::format("Unhandled {} parameter in relaxation callback: {}", cli_desc::relax_cuts.view(), relax_cuts);
    }
}