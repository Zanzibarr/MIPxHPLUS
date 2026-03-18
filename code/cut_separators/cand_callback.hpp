/**
 * Methods for CPLEX candidate callback
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cplex.h>

#include <algorithm>
#include <vector>

#include "bs.hxx"
#include "execution.hpp"
#include "instance.hpp"
#include "statistics.hpp"
#include "utils.hpp"

namespace cand_cuts {

/**
 * Reject the current candidate point with a Landmark Constraint
 */
inline void reject_with_lm_cut(CPXCALLBACKCONTEXTptr context, const std::vector<unsigned int>& landmark) {
    std::vector<int> ind(landmark.begin(), landmark.end());
    std::vector<double> val(landmark.size(), 1.0);
    constexpr double rhs{1.0};
    constexpr char sense{'G'};
    constexpr int begin{0};
    CPX_HANDLE_CALL(CPXcallbackrejectcandidate(context, 1, landmark.size(), &rhs, &sense, &begin, ind.data(), val.data()));
}

/**
 * Reject the current candidate point with a Subtour Elimination Constraint
 */
inline void reject_with_sec_cut(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& cycles) {
    std::vector<int> ind;
    std::vector<int> begin;
    std::vector<double> val;
    std::vector<double> rhs;
    std::vector<char> sense(cycles.size(), 'L');
    for (const auto& cycle : cycles) {
        begin.push_back(static_cast<int>(ind.size()));
        rhs.push_back(static_cast<double>(cycle.size() - 1));
        std::ranges::copy(cycle,
                          std::back_inserter(ind));  // labels in the cycle are the indexes for the first adders in the cplex model
        val.insert(val.end(), cycle.size(), 1.0);
    }
    CPX_HANDLE_CALL(CPXcallbackrejectcandidate(context, cycles.size(), static_cast<int>(ind.size()), rhs.data(), sense.data(), begin.data(),
                                               ind.data(), val.data()));
}

/**
 * Method to compute a violated S.E.C. out of the candidate solutions and reject the candidate solution
 */
[[nodiscard]]
auto add_sec_cut(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const BinarySet& unreachable_actions,
                 const std::vector<std::vector<unsigned int>>& used_first_achievers) -> unsigned int;

}  // namespace cand_cuts

namespace callbacks {

void candidate_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
                        unsigned int& usercuts_lm, unsigned int& usercuts_sec, double& cand_time, unsigned int& cand_calls);

}  // namespace callbacks
