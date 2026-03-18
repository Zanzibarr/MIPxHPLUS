/**
 * Methods used to compute a primal heuristic solution of h+
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <list>
#include <stack>

#include "hplus_algs.hpp"
#include "pq.hxx"

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
    std::vector<double> values;                         // hmax/hadd values for each proposition
    std::vector<unsigned int> goal_sparse;              // sparse representation of the goal
    std::stack<std::pair<unsigned int, double>> trail;  // trail for hmax/hadd updates
    priority_queue<double> pq;                          // priority queue for hmax/hadd updates
    binary_set used_actions;
};

void greedy(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
            std::pair<bool, unsigned int> (*greedy_choice)(const hplus::instance& inst, const std::list<unsigned int>&, const binary_set&,
                                                           greedychoice_userhandle&));

[[nodiscard]]
auto greedy_choice_cost(const hplus::instance& inst, const std::list<unsigned int>& candidates, const binary_set& state,
                        greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

[[nodiscard]]
auto greedy_choice_cxe(const hplus::instance& inst, const std::list<unsigned int>& candidates, const binary_set& state,
                       greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

[[nodiscard]]
auto greedy_choice_hmax(const hplus::instance& inst, const std::list<unsigned int>& candidates, const binary_set& state,
                        greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

[[nodiscard]]
auto greedy_choice_hadd(const hplus::instance& inst, const std::list<unsigned int>& candidates, const binary_set& state,
                        greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int>;

void init_htype_values(const hplus::instance& inst, const std::list<unsigned int>& initial_actions, std::vector<double>& values,
                       priority_queue<double>& pq, double (*h_eqtype)(double, double));

inline void heuristic(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    if (VERBOSE_BASIC()) {
        LOG_INFO << "Running heuristic search algorithm";
    }

    double start_time = GET_TIME();
    stats.heur_time = static_cast<double>(exec.timelimit) - start_time;

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
            LOG_ERROR << "Unhandled algorithm type in heuristic: " << static_cast<int>(exec.ws);
    }

    stats.heur_time = GET_TIME() - start_time;
}

}  // namespace heur
