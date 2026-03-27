/**
 * Methods used to compute a primal heuristic solution of h+
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <list>
#include <stack>

#include "bs.hxx"
#include "execution.hpp"
#include "instance.hpp"
#include "pq.hxx"
#include "statistics.hpp"
#include "timer.hxx"

namespace heur {

[[nodiscard]]
constexpr auto hmax(double fact_a, double fact_b) -> double {
    return fact_a > fact_b ? fact_a : fact_b;
}

[[nodiscard]]
constexpr auto hadd(double fact_a, double fact_b) -> double {
    return fact_a + fact_b;
}

struct greedychoice_userhandle {
    std::vector<double> values;                                // hmax/hadd values for each proposition
    std::vector<unsigned int> goal_sparse;                     // sparse representation of the goal
    std::stack<std::pair<unsigned int, double>> trail;         // trail for fact value changes
    std::stack<std::pair<unsigned int, double>> action_trail;  // trail for action-level changes (hadd_pre / pcf_val)
    priority_queue<double> pq;                                 // priority queue for hmax/hadd updates
    BinarySet used_actions;
    // hadd optimization: incremental precondition sums
    std::vector<double> hadd_pre;  // hadd_pre[i] = sum of values[pre] for all pre of action i
    // hmax optimization: PCF tracking, mirrors lmcut.cpp
    std::vector<int> pcf;         // argmax precondition index per action (-1 = unset)
    std::vector<double> pcf_val;  // values[pcf[i]]
};

void greedy(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
            std::pair<bool, unsigned int> (*greedy_choice)(const hplus::instance& inst, const std::list<unsigned int>&, const BinarySet&,
                                                           greedychoice_userhandle&));

[[nodiscard]]
auto greedy_choice_cost(const hplus::instance& inst, const std::list<unsigned int>& candidates, const BinarySet& state,
                        greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

[[nodiscard]]
auto greedy_choice_cxe(const hplus::instance& inst, const std::list<unsigned int>& candidates, const BinarySet& state,
                       greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

[[nodiscard]]
auto greedy_choice_hmax(const hplus::instance& inst, const std::list<unsigned int>& candidates, const BinarySet& state,
                        greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

[[nodiscard]]
auto greedy_choice_hadd(const hplus::instance& inst, const std::list<unsigned int>& candidates, const BinarySet& state,
                        greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

void init_htype_values(const hplus::instance& inst, const std::list<unsigned int>& initial_actions, std::vector<double>& values,
                       priority_queue<double>& pq, double (*h_eqtype)(double, double));

inline void heuristic(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO_S("Running heuristic search algorithm");
    auto _heur = make_scoped_timer<"heuristic">(STATS);

    switch (exec.ws) {
        case hplus::warmstart::GC:
            greedy(exec, inst, stats, greedy_choice_cost);
            break;
        case hplus::warmstart::GCXE:
            greedy(exec, inst, stats, greedy_choice_cxe);
            break;
        case hplus::warmstart::GHM:
            greedy(exec, inst, stats, greedy_choice_hmax);
            break;
        case hplus::warmstart::GHA:
            greedy(exec, inst, stats, greedy_choice_hadd);
            break;
        default:
            LOG_ERROR_S("Unhandled algorithm type in heuristic: " + std::to_string(static_cast<int>(exec.ws)));
    }
}

}  // namespace heur
