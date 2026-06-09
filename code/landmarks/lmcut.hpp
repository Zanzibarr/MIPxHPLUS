/**
 * LMcut methods
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <functional>
#include <unordered_set>

#include "instance.hpp"
#include "pq.hxx"

struct hmax_context {
    const std::vector<double>& hmax_values;
    const std::vector<double>& initial_hmax_values;
    const std::unordered_set<unsigned int>& goal_section;        // for GZD
    const std::vector<double>& reduced_costs;                    // for BD
    const std::vector<std::vector<unsigned int>>& act_with_eff;  // for BD
};

using hmax_function = std::function<std::pair<int, double>(const std::vector<unsigned int>&, const hmax_context&)>;

namespace hmax {

/// Hmax policy with arbitrary tie-breaking (FCFS)
[[nodiscard]]
auto hmax_arbitrary(const std::vector<unsigned int>& preconditions, const hmax_context& ctx) -> std::pair<int, double>;

/// Hmax policy with inverse tie-breaking (LCFS)
[[nodiscard]]
auto hmax_inverse(const std::vector<unsigned int>& preconditions, const hmax_context& ctx) -> std::pair<int, double>;

/// Hmax policy with VDM tie-breaking
[[nodiscard]]
auto hmax_value_decrease_minimization(const std::vector<unsigned int>& preconditions, const hmax_context& ctx) -> std::pair<int, double>;

/// Hmax policy with random tie-breaking
[[nodiscard]]
auto hmax_random(const std::vector<unsigned int>& preconditions, const hmax_context& ctx) -> std::pair<int, double>;

/// Hmax policy with GZD tie-breaking (prefer preconditions in the zero-cost goal zone V*)
[[nodiscard]]
auto hmax_gzd(const std::vector<unsigned int>& preconditions, const hmax_context& ctx) -> std::pair<int, double>;

/// Hmax policy with BD tie-breaking (prefer preconditions with no zero-cost achievers)
[[nodiscard]]
auto hmax_bd(const std::vector<unsigned int>& preconditions, const hmax_context& ctx) -> std::pair<int, double>;

/// Hmax policy with GZD+BD tie-breaking (GZD first, BD for remaining ties)
[[nodiscard]]
auto hmax_gzd_bd(const std::vector<unsigned int>& preconditions, const hmax_context& ctx) -> std::pair<int, double>;

}  // namespace hmax

class LMcut {
   public:
    LMcut(const hplus::instance& inst, bool greedy_min, bool complete_min);

    // Compute LMcut on the instance passed at construction using the specified hmax function
    auto compute_lmcut(const hmax_function& hmax) -> std::pair<std::vector<std::vector<unsigned int>>, double>;
    // Compute violated landmarks given an integer solution using the specified hmax function
    auto int_separation(const std::vector<unsigned int>& used_actions, const hmax_function& hmax)
        -> std::pair<bool, std::vector<std::vector<unsigned int>>>;
    // Compute violated landmarks given a fracitonal solution using the specified hmax function (here LMcut value has no meaning, so it's not
    // returned)
    auto fract_separation(const std::vector<double>& actions_weights, const hmax_function& hmax)
        -> std::pair<bool, std::vector<std::vector<unsigned int>>>;

    // Check that a landmark is valid for the instance passed at construction
    void check_landmark(const std::vector<unsigned int>& landmark);

   private:
    void init();

    auto compute_lmcut_private(const hmax_function& hmax) -> std::pair<std::vector<std::vector<unsigned int>>, double>;

    void update_and_enqueue_effects_values(priority_queue<double>& queue, unsigned int act_i);
    auto compute_goal_section(const hmax_function& hmax) -> std::unordered_set<unsigned int>;

    void update_hmax_values(const std::vector<unsigned int>& changed_actions, const hmax_function& hmax);
    auto compute_cut(const hmax_function& hmax) -> std::pair<std::vector<unsigned int>, double>;

    const hplus::instance* inst_;
    std::vector<int> pcf_;
    std::vector<double> hmax_values_;
    std::vector<double> initial_hmax_values_;  // Mainly for VDM
    std::vector<double> pcf_hmax_;             // hmax_values[pcf[act]]
    std::vector<double> reduced_costs_;
    std::unordered_set<unsigned int> goal_section_;  // Persistent V* for GZD
    std::vector<unsigned int> goal_;
    std::vector<unsigned int> initial_actions_;
    bool greedy_min_;
    bool complete_min_;
};
