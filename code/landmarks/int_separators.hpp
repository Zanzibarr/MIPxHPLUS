/**
 * Landmark separators for integer solutions
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once


#include "bs.hxx"
#include "instance.hpp"

namespace int_lm_sep {
/**
 * Method to compute multiple violated landmarks by using the LMcut algorithm out of the candidate solutions and reject the candidate solution
 */
[[nodiscard]] auto get_lmcut_violated_landmarks(const hplus::instance& inst, const std::vector<double>& xstar)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>>;

/// Method to compute a violated landmark by using the complementary landmarks technique out of the candidate solutions and reject the candidate
/// solution
[[nodiscard]]
auto get_comp_violated_landmark(const hplus::instance& inst, const std::vector<unsigned int>& unreachable_actions,
                                const std::vector<unsigned int>& unused_actions, const BinarySet& reachable_state) -> std::vector<unsigned int>;

/// Method to compute a violated landmark by using the fontier landmarks technique out of the candidate solutions and reject the candidate solution
[[nodiscard]]
auto get_front_violated_landmark(const hplus::instance& inst, const std::vector<unsigned int>& unused_actions, const BinarySet& reachable_state)
    -> std::vector<unsigned int>;

}  // namespace int_lm_sep