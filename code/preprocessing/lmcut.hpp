/**
 * LMcut methods
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_LMCUT_HPP
#define HPLUS_LMCUT_HPP

#include <functional>

#include "preprocessing.hpp"

namespace lmcut {
using hmax_function_type =
    std::function<std::pair<int, double>(const std::vector<unsigned int>&, const std::vector<double>&, const std::vector<double>&)>;

/**
 * Hmax policy with arbitrary tie-breaking (FCFS)
 */
[[nodiscard]]
std::pair<int, double> hmax_arbitrary(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                      [[maybe_unused]] const std::vector<double>& _);

/**
 * Hmax policy with inverse tie-breaking (LCFS)
 */
[[nodiscard]]
std::pair<int, double> hmax_inverse(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                    [[maybe_unused]] const std::vector<double>& _);

/**
 * Hmax policy with VDM tie-breaking
 */
[[nodiscard]]
std::pair<int, double> hmax_value_decrease_minimization(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                                        const std::vector<double>& initial_hmax_values);

/**
 * Hmax policy with random tie-breaking
 */
[[nodiscard]]
std::pair<int, double> hmax_random(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                   [[maybe_unused]] const std::vector<double>& _);

/**
 * Initialization function for lmcut
 */
void init_hmax(const hplus::instance& inst, std::vector<double>& hmax_values, std::vector<int>& pcf, std::vector<double>& pcf_hmax,
               std::vector<double>& reduced_costs, std::vector<unsigned int>& initial_actions);

/**
 * Computes LMcut with the specified tie-breaking policy
 */
void compute_lmcut(const hplus::instance& inst, std::vector<double> hmax_values, std::vector<int> pcf, std::vector<double> pcf_hmax,
                   std::vector<double> reduced_costs, const std::vector<unsigned int>& goal_sparse, const std::vector<unsigned int>& initial_actions,
                   const hmax_function_type& hmax_function, std::vector<std::vector<unsigned int>>& landmarks, bool print = false);
}  // namespace lmcut

#endif