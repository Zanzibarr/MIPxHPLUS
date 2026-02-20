#include "branching.hpp"

void callbacks::branching_callback(CPXCALLBACKCONTEXTptr context, [[maybe_unused]] const hplus::execution& exec, const hplus::instance& inst,
                                   [[maybe_unused]] hplus::statistics& stats) {
    // TODO: Branching idea: compute the hadd value for each action's branching (x_a=0 and x_a=1) and branch on the one that has the highest product
    // (or distance)
    // -> up branch: set the cost of the action to 0, down branch: remove the action from the domain (or set its cost to a very high value)
    // -> this is static branching... we just need to compute the scores beforehand and then use those scores to decide... we might scale them by
    // their distance from 0.5 to prioritize more fractional variables

    // This is just a test to understand how to implement a custom branching rule
    // Get the current relaxation solution values (we're only interested in first achievers)
    std::vector<double> x(inst.nfadd);
    CPXcallbackgetrelaxationpoint(context, x.data(), inst.m, inst.m + inst.nfadd - 1, nullptr);

    // Find the most infeasible integer variable (closest to 0.5)
    int best_var = -1;
    double best_infeas = -1.0;
    for (unsigned int j = 0; j < inst.nfadd; ++j) {
        double frac = x[j] - std::floor(x[j]);
        if (frac < HPLUS_EPSILON || frac > 1 - HPLUS_EPSILON) continue;  // already integer, skip

        double infeas = std::min(frac, 1.0 - frac);  // distance to nearest integer
        if (infeas > best_infeas) {
            best_infeas = infeas;
            best_var = j;
        }
    }

    if (best_var == -1) return;  // no fractional variable found, let CPLEX decide
    const int varind = inst.m + best_var;
    int child_seqnum;

    // Create up branch
    {
        const char varlu = 'L';
        const double coef = 1.0;
        CPX_HANDLE_CALL(
            CPXcallbackmakebranch(context, 1, &varind, &varlu, &coef, 0, 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0.0, &child_seqnum));
    }

    // Create down branch
    {
        const char varlu = 'U';
        const double coef = 0.0;
        CPX_HANDLE_CALL(
            CPXcallbackmakebranch(context, 1, &varind, &varlu, &coef, 0, 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0.0, &child_seqnum));
    }
}