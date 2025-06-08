#include "relax_callback.hpp"

[[nodiscard]]
unsigned int relax_cuts::lm(CPXCALLBACKCONTEXTptr context, CPXENVptr& env, CPXLPptr& lp, const hplus::instance& inst,
                            const std::vector<double>& relax_point) {
    std::vector<int> ind(inst.m);
    std::iota(ind.begin(), ind.end(), 0);
    CPX_HANDLE_CALL(CPXchgobj(env, lp, inst.m, ind.data(), relax_point.data()));
    CPX_HANDLE_CALL(CPXmipopt(env, lp));
    int status = CPXgetstat(env, lp);
    if (status != CPXMIP_OPTIMAL && status != CPXMIP_OPTIMAL_TOL) {
        LOG_WARNING << "CPXgetstat: " << status;
        return 0;
    }
    double cutval{CPX_INFBOUND};
    CPX_HANDLE_CALL(CPXgetobjval(env, lp, &cutval));
    if (cutval >= 1 - HPLUS_EPSILON) return 0;

    std::vector<double> facts_partition(inst.n);
    CPX_HANDLE_CALL(CPXgetx(env, lp, facts_partition.data(), inst.m, inst.m + inst.n - 1));
    binary_set reach(inst.n);
    for (unsigned int p = 0; p < inst.n; ++p) {
        if (facts_partition[p] >= HPLUS_CPX_INT_ROUNDING) reach.add(p);
    }
    bool found{false};
    while (!found && !reach.empty()) {
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
    std::vector<double> val(inst.m);
    constexpr double rhs{1};
    constexpr char sense{'G'};
    constexpr int begin{0}, purgeable{CPX_USECUT_FORCE}, local{0};
    int nnz{0};
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (reach.contains(inst.actions[act_i].pre) && !reach.contains(inst.actions[act_i].eff)) {
            ind[nnz] = act_i;
            val[nnz++] = 1.0;
        }
    }
    CPX_HANDLE_CALL(CPXcallbackaddusercuts(context, 1, nnz, &rhs, &sense, &begin, ind.data(), val.data(), &purgeable, &local));
    return 1;
}
