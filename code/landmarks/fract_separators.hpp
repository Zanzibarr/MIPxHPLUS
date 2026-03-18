#pragma once

#include "execution.hpp"
#include "instance.hpp"

namespace fract_lm_sep {
/**
 * Compute the violated landmark with the max-flow algorithm (if there's one) out of the relaxed solution
 */
[[nodiscard]]
auto get_r3_violated_landmark(const hplus::execution& exec, const hplus::instance& inst, std::vector<double> relax_point)
    -> std::pair<bool, std::vector<unsigned int>>;

/**
 * Compute the violated landmark with LMcut (if there's one) out of the relaxed solution
 */
[[nodiscard]]
auto get_lmcut_violated_landmarks(const hplus::execution& exec, const hplus::instance& inst, const std::vector<double>& relax_point)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>>;
}  // namespace fract_lm_sep