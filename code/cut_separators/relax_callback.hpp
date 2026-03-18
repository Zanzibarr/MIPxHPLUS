/**
 * Methods for CPLEX relaxation callback
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cplex.h>

#include "cycle_det.hpp"
#include "execution.hpp"
#include "instance.hpp"

namespace callbacks {

/**
 * Thread-specific data for CPLEX callbacks
 */
struct thread_data {
    unsigned int usercuts_lm, usercuts_sec, relax_calls, cand_calls, acts_in_lm, n_lm;
    double cand_time, relax_time;
};

void relaxation_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, thread_data& data);

}  // namespace callbacks

namespace relax_cuts {

/**
 * Analyze the current relaxation point: generate data structures to be used in later parts of the callback
 */
[[nodiscard]]
auto relaxationpoint_info(const hplus::instance& inst, std::vector<double>& relax_point)
    -> std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>;

/**
 * Compute the violated landmark (if there's one) out of the relaxed solution and reject the relaxed solution
 */
[[nodiscard]]
auto add_lm_cut(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, const std::vector<double>& relax_point,
                unsigned int& n_act_in_lm, unsigned int& n_lm) -> unsigned int;

/**
 * Compute the violated S.E.C. (if there's one) out of the relaxed solution
 */
[[nodiscard]]
auto get_violated_sec(const hplus::instance& inst, const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>>;

/**
 * Compute the violated S.E.C. (if there's one) out of the relaxed solution and reject the relaxed solution
 */
[[nodiscard]]
auto add_sec_cut(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst,
                 const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights) -> unsigned int;

}  // namespace relax_cuts
