#ifndef HPLUS_CAND_CALLBACK_HPP
#define HPLUS_CAND_CALLBACK_HPP

#include <cplex.h>

#include "../domain/hplus_algs.hpp"

namespace cand_cuts {

inline void reject_with_lm_cut(CPXCALLBACKCONTEXTptr context, const std::vector<unsigned int>& landmark) {
    std::vector<int> ind(landmark.begin(), landmark.end());
    std::vector<double> val(landmark.size(), 1.0);
    constexpr double rhs{1.0};
    constexpr char sense{'G'};
    constexpr int begin{0};
    CPX_HANDLE_CALL(CPXcallbackrejectcandidate(context, 1, landmark.size(), &rhs, &sense, &begin, ind.data(), val.data()));
}

inline void reject_with_sec_cut(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& cycles) {
    std::vector<int> ind, begin;
    std::vector<double> val;
    std::vector<double> rhs;
    std::vector<char> sense(cycles.size(), 'L');
    for (const auto& cycle : cycles) {
        begin.push_back(static_cast<int>(ind.size()));
        rhs.push_back(static_cast<double>(cycle.size() - 1));
        std::copy(cycle.begin(), cycle.end(),
                  std::back_inserter(ind));  // labels in the cycle are the indexes for the first adders in the cplex model
        val.insert(val.end(), cycle.size(), 1.0);
    }
    CPX_HANDLE_CALL(CPXcallbackrejectcandidate(context, cycles.size(), static_cast<int>(ind.size()), rhs.data(), sense.data(), begin.data(),
                                               ind.data(), val.data()));
}

[[nodiscard]]
unsigned int complementary_lm(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const binary_set& unreachable_actions,
                              const std::vector<unsigned int>& unused_actions, const binary_set& reachable_state);

[[nodiscard]]
unsigned int frontier_lm(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const std::vector<unsigned int>& unused_actions,
                         const binary_set& reachable_state);

[[nodiscard]]
unsigned int sec(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, const binary_set& unreachable_actions,
                 const std::vector<binary_set>& used_first_achievers);

}  // namespace cand_cuts

namespace callbacks {

void candidate_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
                        unsigned int& usercuts_lm, unsigned int& usercuts_sec, double& cand_time);

}  // namespace callbacks

#endif