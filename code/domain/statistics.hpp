/**
 * Methods and objects for data structures related to the statistics to gather about this project execution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include "logger.hxx"
#include "stats_registry.hxx"
#include "utils.hpp"

namespace hplus {

struct statistics {
    // Costs
    unsigned int heur_cost, cost;
    // Cplex informations
    int status;
    double lower_bound;
};

inline void init(statistics& stats) { stats = statistics{.heur_cost = INFBOUND_INT, .cost = INFBOUND_INT, .status = 3, .lower_bound = 0}; }

inline void print(const statistics& stats) {
    LOG_S("---------------------- Statistics ----------------------");
    LOG << " >> Status              " << std::setw(28) << stats.status << " <<";
    LOG << " >> Lower bound         " << std::setw(28)
        << (stats.lower_bound >= INFBOUND_DBL ? "1e20" : (std::ostringstream() << std::fixed << std::setprecision(3) << stats.lower_bound).str())
        << " <<";
    LOG << " >> Heuristic           " << std::setw(28) << (stats.heur_cost >= INFBOUND_INT ? "1e20" : std::to_string(stats.heur_cost)) << " <<";
    LOG << " >> Final cost          " << std::setw(28) << (stats.cost >= INFBOUND_INT ? "1e20" : std::to_string(stats.cost)) << " <<\n";
    LOG_S(STATS.stats_report_to_str());
    LOG_S(STATS.counter_report_to_str());
    LOG_S(STATS.gauge_report_to_str());
    LOG_S("--------------------------------------------------------");
}

}  // namespace hplus
