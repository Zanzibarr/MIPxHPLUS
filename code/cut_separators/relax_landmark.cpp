#include "relax_callback.hpp"

[[nodiscard]]
std::pair<bool, std::vector<unsigned int>> relax_cuts::get_violated_landmark(CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec,
                                                                             const hplus::instance& inst, const std::vector<double>& relax_point) {
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

    // If the value of the cut is greater than 1, than no landmark has been violated
    if (cutval >= 1 - HPLUS_EPSILON) return {false, {}};

    // Now we know that a landmark is violated

    std::vector<double> facts_partition(inst.n);
    CPX_HANDLE_CALL(CPXgetx(env, lp, facts_partition.data(), inst.m, inst.m + inst.n - 1));
    binary_set left(inst.n);
    // Obtain the left side of the cut (variables set to 1)
    for (unsigned int p = 0; p < inst.n; ++p) {
        if (facts_partition[p] >= HPLUS_CPX_INT_ROUNDING) left.add(p);
    }
    binary_set right(!left);
    bool found{false};
    while (!found && !right.empty()) {
        for (const auto& p : right) {
            found = false;
            for (const auto& act_i : inst.act_with_eff[p]) {
                if (left.contains(inst.actions[act_i].pre)) {
                    found = true;
                    break;
                }
            }
            if (!found) right.remove(p);
        }
    }
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (left.contains(inst.actions[act_i].pre) && !left.contains(inst.actions[act_i].eff)) landmark.push_back(act_i);
    }

    return {true, landmark};
}

[[nodiscard]]
unsigned int relax_cuts::lm(CPXCALLBACKCONTEXTptr context, CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec, const hplus::instance& inst,
                            const std::vector<double>& relax_point) {
    const auto& [found, landmark]{get_violated_landmark(env, lp, exec, inst, relax_point)};
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
