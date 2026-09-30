#include <binary_set.hxx>
#include <ranges>

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
        case CPX_CALLBACKCONTEXT_BRANCHING:
            solver->hplus_branching_callback_(context);
            break;
        default:
            solver->logger_[FATAL] << std::format("Unhandled CPLEX callback context: {}", contextid);
    }

    return 0;
}

void Solver::hplus_progress_callback_(CPXCALLBACKCONTEXTptr context) {
    double best_lb{-1};
    call_cplex(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));

    CPXLONG nodecount{-1};
    call_cplex(CPXcallbackgetinfolong(context, CPXCALLBACKINFO_NODECOUNT, &nodecount));
    if (nodecount == 0) {
        // Still at the root: track the highest bound seen so far
        double current = global_.relax_last_root_lb.load();
        while (best_lb > current && !global_.relax_last_root_lb.compare_exchange_weak(current, best_lb)) {
        }
    } else {
        // First progress event after leaving the root: freeze the root bound
        bool expected = false;
        if (global_.relax_lb_rootnode_recorded.compare_exchange_strong(expected, true)) {
            stats_.gauge_record<"lb_rootnode">(actual_bound_(global_.relax_last_root_lb));
        }
    }
}

// ##################################################################### //
// ######################### CANDIDATE CALLBACK ######################## //
// ##################################################################### //

void Solver::hplus_candidate_callback_(CPXCALLBACKCONTEXTptr context) {
    auto _callback_timer = scoped_timer("cand_callback");
    stats_.counter_inc<"cand_calls">();

    const unsigned int size = inst_.m;
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

    // Give CPLEX the better solution
    call_cplex(
        CPXcallbackpostheursoln(context, static_cast<int>(ncols), ind.data(), val.data(), static_cast<double>(cost), CPXCALLBACKSOLUTION_NOCHECK));
}

// ##################################################################### //
// ######################## RELAXATION CALLBACK ######################## //
// ##################################################################### //

void Solver::hplus_relaxation_callback_(CPXCALLBACKCONTEXTptr context) {
    // Get the lp relaxation (first ever LP solution)
    std::call_once(global_.relax_lb_once, [&] {
        double best_lb{-1};
        call_cplex(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));
        stats_.gauge_record<"lb_relaxation">(actual_bound_(best_lb));
    });

    int nodeuid{-1};
    int nodedepth{-1};
    call_cplex(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEUID, &nodeuid));
    call_cplex(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEDEPTH, &nodedepth));
    int restarts{0};
    call_cplex(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_RESTARTS, &restarts));

    // A restart rebuilds the tree from a new root: node uids from the previous tree are meaningless
    if (restarts > local_.relax_visited_restart) {
        local_.relax_visited_nodes.clear();
        local_.relax_visited_restart = restarts;
    }

    if (nodedepth == 0) {  // Visit the root node at most k times (per restart)
        int iter{0};
        {
            std::scoped_lock lock(global_.relax_root_mutex);
            if (restarts > global_.relax_root_restart) {
                global_.relax_root_restart = restarts;
                global_.relax_root_iterations = 0;
            }
            iter = ++global_.relax_root_iterations;
        }

        const auto max_root_iter = params_.get<cli_desc::root_max_iter, int>();
        if (iter > max_root_iter) {
            return;
        }

    } else if (local_.relax_visited_nodes.contains(nodeuid)) {  // Visit each node (except for root node) at most once
        return;
    }
    local_.relax_visited_nodes.insert(nodeuid);

    const auto& relax_cuts = params_.get<cli_desc::relax_cuts, std::string>();
    if (relax_cuts == "0") {
        return;
    }

    // ~~~~~~~~~ Callback starts here ~~~~~~~~ //

    auto _callback_timer = scoped_timer("relax_callback");
    stats_.counter_inc<"relax_calls">();

    const unsigned int size = inst_.m;
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

// ##################################################################### //
// ######################### BRANCHING CALLBACK ######################## //
// ##################################################################### //
void Solver::hplus_branching_callback_(CPXCALLBACKCONTEXTptr context) {
    auto _callback_timer = scoped_timer("branch_callback");
    stats_.counter_inc<"branch_calls">();

    if (local_.relax_xstar.size() != inst_.m) {
        local_.relax_xstar = std::vector<double>(inst_.m);
    }
    double cost{CPX_INFBOUND};
    call_cplex(CPXcallbackgetrelaxationpoint(context, local_.relax_xstar.data(), 0, static_cast<int>(inst_.m - 1), &cost));

    // Fix numerical errors
    for (auto& val : local_.relax_xstar) {
        if (is_lw_or_eq_double(val, 0)) {
            val = 0;
        } else if (is_gr_or_eq_double(val, 1)) {
            val = 1;
        }
    }

    std::vector<unsigned int> actions_fract;
    actions_fract.reserve(inst_.m);
    for (const auto [i, val] : std::views::enumerate(local_.relax_xstar)) {
        if (!is_same_double(val, 0) && !is_same_double(val, 1)) {
            actions_fract.push_back(i);
        }
    }

    // If none is fractional, for now, record this statistics and let CPLEX choose how to branch
    if (actions_fract.empty()) {
        stats_.counter_inc<"branch_all-int">();
        return;
    }

    std::vector<double> lbs(inst_.m);
    std::vector<double> ubs(inst_.m);
    call_cplex(CPXcallbackgetlocallb(context, lbs.data(), 0, static_cast<int>(inst_.m - 1)));
    call_cplex(CPXcallbackgetlocalub(context, ubs.data(), 0, static_cast<int>(inst_.m - 1)));
    std::vector<int> fixings;
    fixings.reserve(inst_.m);
    for (const auto [lb, ub] : std::views::zip(lbs, ubs)) {
        if (is_gr_or_eq_double(lb, 1)) {
            fixings.push_back(1);
        } else if (is_lw_or_eq_double(ub, 0)) {
            fixings.push_back(0);
        } else {
            fixings.push_back(-1);
        }
    }

    auto lmcut_base = hplus_branching_compute_lmcut_(fixings);

    // Get the incumbent for basic pruning
    double incumbent{-1};
    call_cplex(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_SOL, &incumbent));

    // If LMCut returns a bound worse than the incumbent, we can prune
    if (is_gr_or_eq_double(lmcut_base, incumbent)) {
        if (lmcut_base == std::numeric_limits<double>::infinity()) {  // Prune by infeasibility
            stats_.counter_inc<"branch_lmcut-inf">();
        } else {  // Prune by duality
            stats_.counter_inc<"branch_lmcut-prune">();
        }

        call_cplex(CPXcallbackprunenode(context));
        return;
    }

    // Loop over actions we can branch on to compute the branching scores
    double max_score = -1;
    unsigned int branch_act = inst_.m;
    std::vector<std::pair<unsigned int, double>> derived_fixings;
    unsigned int fixed_0 = 0;
    unsigned int fixed_1 = 0;
    for (const auto act_i : actions_fract) {
        myassert(fixings[act_i] == -1, "Action reported as fractional was fixed to either 0 or 1 in branch callback");

        //  fix act_i to 0: compute lmcut
        fixings[act_i] = 0;
        auto lmcut_down = hplus_branching_compute_lmcut_(fixings);
        fixings[act_i] = -1;

        //  fix act_i to 1: compute lmcut
        fixings[act_i] = 1;
        auto lmcut_up = hplus_branching_compute_lmcut_(fixings);
        fixings[act_i] = -1;

        if (is_gr_or_eq_double(lmcut_down, incumbent) && is_gr_or_eq_double(lmcut_up, incumbent)) {
            stats_.counter_inc<"branch_lmcut-prune-child">();
            call_cplex(CPXcallbackprunenode(context));
            return;
        }
        if (is_gr_or_eq_double(lmcut_down, incumbent)) {
            ++fixed_1;
            derived_fixings.emplace_back(act_i, 1);
            continue;
        }
        if (is_gr_or_eq_double(lmcut_up, incumbent)) {
            ++fixed_0;
            derived_fixings.emplace_back(act_i, 0);
            continue;
        }

        // Score the child nodes
        auto score = std::max(lmcut_down - lmcut_base, constants::epsilon) * std::max(lmcut_up - lmcut_base, constants::epsilon);
        // Use a smaller epsilon since we are comparing epsilon-multiplied values
        if (is_gr_strict_double(score, max_score, constants::epsilon * 1e-2)) {
            max_score = score;
            branch_act = act_i;
        }
    }

    stats_.counter_inc<"branch_fixed-0">(fixed_0);
    stats_.counter_inc<"branch_fixed-1">(fixed_1);
    if (branch_act != inst_.m) {
        stats_.gauge_record<"branch_score">(max_score);
    }

    // If no action has been chosen, all fractional actions have been fixed by the lmcut tests: just one child with all the fixings applied
    // Otherwise, branch on the chosen action: two children, both with all fixings applied
    // TODO: The landmarks produced by the up and down children of the chosen action could be added as local cuts in the child nodes... or if there's
    // not an action to branch on, we could add those generated by computing the base lmcut
    hplus_branching_make_branches_(context, derived_fixings, branch_act == inst_.m ? std::nullopt : std::optional{branch_act},
                                   std::max(lmcut_base, cost));
}

// Create the children of the current node: each child gets all the given fixings; if branch_act is set, two children are created (with branch_act
// fixed to 0 and to 1 respectively), otherwise a single child is created with just the fixings.
// https://www.ibm.com/docs/en/cofz/22.1.2?topic=SS9UKU_22.1.2/com.ibm.cplex.zos.help/refcallablelibrary/macros/CPX_CALLBACKCONTEXT_BRANCHING.htm
void Solver::hplus_branching_make_branches_(CPXCALLBACKCONTEXTptr context, const std::vector<std::pair<unsigned int, double>>& fixings,
                                            std::optional<unsigned int> branch_act, double nodeest) {
    // Nothing to branch on and nothing to fix: let CPLEX choose how to branch
    // TODO: This should be an assert (?)
    if (!branch_act.has_value() && fixings.empty()) {
        return;
    }

    // Bound changes shared by all children (+1 slot for the branching action, if any)
    std::vector<int> varind;
    std::vector<char> varlu;
    std::vector<double> varbd;
    varind.reserve(fixings.size() + 1);
    varlu.reserve(fixings.size() + 1);
    varbd.reserve(fixings.size() + 1);
    for (const auto& [act_i, val] : fixings) {
        varind.push_back(static_cast<int>(act_i));
        varlu.push_back(is_same_double(val, 0) ? 'U' : 'L');
        varbd.push_back(val);
    }

    int seqnum{-1};
    if (!branch_act.has_value()) {
        stats_.counter_inc<"branch_single-child">();
        call_cplex(CPXcallbackmakebranch(context, static_cast<int>(varind.size()), varind.data(), varlu.data(), varbd.data(), 0, 0, nullptr, nullptr,
                                         nullptr, nullptr, nullptr, nodeest, &seqnum));
        return;
    }

    varind.push_back(static_cast<int>(*branch_act));
    // Down child: branch_act <= 0
    varlu.push_back('U');
    varbd.push_back(0);
    call_cplex(CPXcallbackmakebranch(context, static_cast<int>(varind.size()), varind.data(), varlu.data(), varbd.data(), 0, 0, nullptr, nullptr,
                                     nullptr, nullptr, nullptr, nodeest, &seqnum));
    // Up child: branch_act >= 1
    varlu.back() = 'L';
    varbd.back() = 1;
    call_cplex(CPXcallbackmakebranch(context, static_cast<int>(varind.size()), varind.data(), varlu.data(), varbd.data(), 0, 0, nullptr, nullptr,
                                     nullptr, nullptr, nullptr, nodeest, &seqnum));
}

auto Solver::hplus_branching_compute_lmcut_(const std::vector<int>& fixings) -> double {
    auto _callback_timer = scoped_timer("branch_lmcut");
    stats_.counter_inc<"branch_lmcut_calls">();

    lmcut_init_();

    double fixed_cost{0};

    // Get actions fixed to 0: those shall have a +inf cost, so that they are excluded in the hmax computation (and it effectively works as if those
    // actions were removed): an hmax of any of the goal facts of +inf now means that the task is infeasible -> prune the node (if its the base lmcut)
    // or immediatelly create a single child node with the action fixed to 1 (if it was a 0-fixing of a fractional action).
    // Get actions fixed to 1: those shall have a 0 cost and have the lmcut cost initialized by those costs...
    for (const auto [i, val] : std::views::enumerate(fixings)) {
        switch (val) {
            case -1:  // We don't touch non-fixed actions
                break;
            case 0:
                local_.lmcut_reduced_costs[i] = std::numeric_limits<double>::infinity();
                break;
            case 1:
                fixed_cost += local_.lmcut_reduced_costs[i];
                local_.lmcut_reduced_costs[i] = 0;
                break;
            default:
                logger_[FATAL] << std::format("Unhandled fixing value ({}) in branching lmcut.", val);
        }
    }

    // Compute lmcut
    const auto& [landmarks, lmcut] = lmcut_compute_private_(&Solver::lmcut_hmax_arbitrary_, 'g');

    return fixed_cost + lmcut;
}
