/**
 * Methods for CPLEX global progress callback
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cplex.h>

#include "execution.hpp"
#include "instance.hpp"
#include "limits.hxx"

namespace callbacks {
inline void progress_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& /*exec*/, const hplus::instance& inst) {
    double best_lb{-1};
    CPX_HANDLE_CALL(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));

    // If the lower bound (CPLEX) and the best solution (ours) match, we already have the optimal solution -> exit
    if (best_lb >= inst.sol.cost - HPLUS_EPSILON) {
        GLOBAL_TERMINATE_CONDITION = 1;  // This will stop CPLEX execution
        return;
    }
}
}  // namespace callbacks