#include <list>

#include "relax_callback.hpp"

[[nodiscard]]
std::pair<bool, std::vector<unsigned int>> relax_cuts::get_violated_landmark(CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec,
                                                                             const hplus::instance& inst, const std::vector<double>& relax_point,
                                                                             unsigned int& flm_0, unsigned int& flm_01) {
    // if (!exec.testing) {
    std::vector<int> ind(inst.m);
    std::vector<unsigned int> landmark(0);
    std::iota(ind.begin(), ind.end(), 0);
    if (exec.timelimit > 0 && static_cast<double>(exec.timelimit) > GET_TIME()) {
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_TimeLimit, static_cast<double>(exec.timelimit) - GET_TIME()));
    } else
        throw timelimit_exception("Reached time limit.");
    // Update the objective function -> adapt the model to the current relaxed solution
    CPX_HANDLE_CALL(CPXchgobj(env, lp, inst.m, ind.data(), relax_point.data()));
    CPX_HANDLE_CALL(CPXmipopt(env, lp));
    int status = CPXgetstat(env, lp);
    if (status != CPXMIP_OPTIMAL && status != CPXMIP_OPTIMAL_TOL) return {false, {}};  // For time-limit breaching -> no solution found
    double cutval{CPX_INFBOUND};
    CPX_HANDLE_CALL(CPXgetobjval(env, lp, &cutval));

    // TODO: Remove, this is for debugging
    int nodes{CPXgetnodecnt(env, lp)};
    if (nodes > 0) {
        // CPX_HANDLE_CALL(CPXwriteprob(env, lp, HPLUS_CPLEX_OUTPUT_DIR "/lp/data-network-sat18-strips-p19_fract_nopresolve.lp",
        //                              "lp"));  // TODO: Remove... this is only for testing
        LOG_WARNING << "flmdet model gets solved outside of the root node (" << nodes << ").";
    }

    // If the value of the cut is greater than 1, than no landmark has been violated
    if (cutval >= 1 - HPLUS_EPSILON) {
        // TODO: Remove, this is for debugging
        // LOG_ERROR << "No landmark found...";
        return {false, {}};
    }

    // Now we know that a landmark is violated

    // TODO: Remove, this is for debugging
    if (cutval <= HPLUS_EPSILON)
        flm_0++;
    else
        flm_01++;

    std::vector<double> facts_partition(inst.n);
    CPX_HANDLE_CALL(CPXgetx(env, lp, facts_partition.data(), inst.m, inst.m + inst.n - 1));
    binary_set reach(inst.n);
    // Obtain the reach side of the cut (variables set to 1)
    for (unsigned int p = 0; p < inst.n; ++p) {
        if (facts_partition[p] >= HPLUS_CPX_INT_ROUNDING) reach.add(p);
    }
    // Actions crossing the cut are those that have all preconditions in the left side, and at least one effect in the right side
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (reach.contains(inst.actions[act_i].pre) && !reach.contains(inst.actions[act_i].eff)) landmark.push_back(act_i);
    }

    // TODO: Remove, this is for debugging
    // LOG_DEBUG << "Found violated landmark of size: " << landmark.size();
    return {true, landmark};
    // } else {
    //     std::vector<unsigned int> landmark;
    //     binary_set state(inst.n);

    //     std::list<unsigned int> candidates;
    //     for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
    //         if (inst.actions[act_i].pre_sparse.empty()) candidates.push_back(act_i);
    //     }

    //     const auto check_landmark = [&](const std::list<unsigned int>& candidates) {
    //         double cut_value{0};
    //         for (const auto& act_i : candidates) cut_value += relax_point[act_i];
    //         if (cut_value >= 1)
    //             return true;
    //         else {
    //             landmark = std::vector<unsigned int>(candidates.begin(), candidates.end());
    //             return false;
    //         }
    //     };

    //     while (!state.contains(inst.goal)) {
    //         if (!check_landmark(candidates)) {
    //             // TODO: Remove, this is for debugging
    //             LOG_DEBUG << "Found violated landmark of size: " << landmark.size();
    //             return {true, landmark};
    //         }

    //         unsigned int choice{*candidates.begin()};
    //         double best_value{relax_point[choice]};
    //         for (const auto& act_i : candidates) {
    //             if (relax_point[act_i] > best_value) {
    //                 choice = act_i;
    //                 best_value = relax_point[act_i];
    //             }
    //         }

    //         // add new actions to the candidates
    //         const auto& new_state = state | inst.actions[choice].eff;
    //         for (const auto& p : inst.actions[choice].eff_sparse) {
    //             if (state[p]) continue;
    //             for (const auto& act_j : inst.act_with_pre[p]) {
    //                 if (new_state.contains(inst.actions[act_j].pre) && std::find(candidates.begin(), candidates.end(), act_j) == candidates.end())
    //                     candidates.push_back(act_j);
    //             }
    //         }

    //         // purge unnecessary actions from candidates
    //         candidates.remove_if([&](unsigned int act_j) { return new_state.contains(inst.actions[act_j].eff) && !inst.fixed_actions[act_j]; });

    //         state = new_state;
    //     }

    //     // TODO: Remove, this is for debugging
    //     LOG_ERROR << "No landmark found...";
    //     return {false, {}};
    // }
}

[[nodiscard]]
unsigned int relax_cuts::lm(CPXCALLBACKCONTEXTptr context, CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec, const hplus::instance& inst,
                            const std::vector<double>& relax_point, callbacks::thread_data& data) {
    const auto& [found, landmark]{get_violated_landmark(env, lp, exec, inst, relax_point, data.flm_0, data.flm_01)};
    if (!found) return 0;
    std::vector<int> ind(landmark.size());
    int nnz{0};
    for (const auto& act_i : landmark) ind[nnz++] = act_i;
    std::vector<double> val(landmark.size(), 1.0);
    constexpr double rhs{1};
    constexpr char sense{'G'};
    constexpr int begin{0}, purgeable{CPX_USECUT_FORCE}, local{0};
    CPX_HANDLE_CALL(CPXcallbackaddusercuts(context, 1, nnz, &rhs, &sense, &begin, ind.data(), val.data(), &purgeable, &local));
    return 1;
}
