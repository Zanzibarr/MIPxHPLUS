/**
 * Methods and objects for data structures related to the statistics to gather about this project execution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_STATS_HPP
#define HPLUS_STATS_HPP

#include <string>

namespace hplus {

struct statistics {
    // Time
    double parsing, preprocessing, heur_time, build, relax_callback, cand_callback, cutloop, cplex_execution, total;
    // Costs
    unsigned int heur_cost, cost;
    // Preprocessing
    unsigned int n_prep, m_prep, nfadd_prep;
    // Cplex informations
    int status;
    unsigned int nodes, var_base, var_acyc, const_base, const_acyc, cuts_lm, cuts_sec, cutloop_it, cand_calls, relax_calls, flm_0, flm_01;
    double lower_bound;
};

inline void init(statistics& stats) {
    stats = statistics{.parsing = 0,
                       .preprocessing = 0,
                       .heur_time = 0,
                       .build = 0,
                       .relax_callback = 0,
                       .cand_callback = 0,
                       .cutloop = 0,
                       .cplex_execution = 0,
                       .total = 0,
                       .heur_cost = INFBOUND_INT,
                       .cost = INFBOUND_INT,
                       .n_prep = 0,
                       .m_prep = 0,
                       .nfadd_prep = 0,
                       .status = 3,
                       .nodes = 0,
                       .var_base = 0,
                       .var_acyc = 0,
                       .const_base = 0,
                       .const_acyc = 0,
                       .cuts_lm = 0,
                       .cuts_sec = 0,
                       .cutloop_it = 0,
                       .cand_calls = 0,
                       .relax_calls = 0,
                       .flm_0 = 0,
                       .flm_01 = 0,
                       .lower_bound = 0};
}

inline void print(const statistics& stats) {
    LOG << "---------------------- Statistics ----------------------";
    LOG << " >> Status              " << std::setw(28) << stats.status << " <<";
    LOG << " >> Facts prep          " << std::setw(28) << stats.n_prep << " <<";
    LOG << " >> Acts prep           " << std::setw(28) << stats.m_prep << " <<";
    LOG << " >> Fadd prep           " << std::setw(28) << stats.nfadd_prep << " <<";
    LOG << " >> Base model var      " << std::setw(28) << stats.var_base << " <<";
    LOG << " >> Acyc model var      " << std::setw(28) << stats.var_acyc << " <<";
    LOG << " >> Base model const    " << std::setw(28) << stats.const_base << " <<";
    LOG << " >> Acyc model const    " << std::setw(28) << stats.const_acyc << " <<";
    LOG << " >> User cuts (lm)      " << std::setw(28) << stats.cuts_lm << " <<";
    LOG << " >> User cuts (sec)     " << std::setw(28) << stats.cuts_sec << " <<";
    LOG << " >> Fract LM at 0       " << std::setw(28) << stats.flm_0 << " <<";
    LOG << " >> Fract LM in (0,1]   " << std::setw(28) << stats.flm_01 << " <<";
    LOG << " >> Nodes expanded      " << std::setw(28) << stats.nodes << " <<";
    LOG << " >> Cutloop iterations  " << std::setw(28) << stats.cutloop_it << " <<";
    LOG << " >> Candidate callback calls " << std::setw(23) << stats.cand_calls << " <<";
    LOG << " >> Relax callback calls     " << std::setw(23) << stats.relax_calls << " <<";
    LOG << " >> Lower bound         " << std::setw(28)
        << (stats.lower_bound >= INFBOUND_DBL ? "1e20" : (std::ostringstream() << std::fixed << std::setprecision(3) << stats.lower_bound).str())
        << " <<";
    LOG << " >> Heuristic           " << std::setw(28) << (stats.heur_cost >= INFBOUND_INT ? "1e20" : std::to_string(stats.heur_cost)) << " <<";
    LOG << " >> Final cost          " << std::setw(28) << (stats.cost >= INFBOUND_INT ? "1e20" : std::to_string(stats.cost)) << " <<";
    LOG << " >> Parsing time        " << std::setw(27) << std::fixed << std::setprecision(4) << stats.parsing << "s <<";
    LOG << " >> Preprocessing time  " << std::setw(27) << std::fixed << std::setprecision(4) << stats.preprocessing << "s <<";
    LOG << " >> Heuristic time      " << std::setw(27) << std::fixed << std::setprecision(4) << stats.heur_time << "s <<";
    LOG << " >> Model build time    " << std::setw(27) << std::fixed << std::setprecision(4) << stats.build << "s <<";
    LOG << " >> Relax callback time " << std::setw(27) << std::fixed << std::setprecision(4) << stats.relax_callback << "s <<";
    LOG << " >> Cand callback time  " << std::setw(27) << std::fixed << std::setprecision(4) << stats.cand_callback << "s <<";
    LOG << " >> Cutloop time        " << std::setw(27) << std::fixed << std::setprecision(4) << stats.cutloop << "s <<";
    LOG << " >> CPLEX time          " << std::setw(27) << std::fixed << std::setprecision(4) << stats.cplex_execution << "s <<";
    LOG << " >> Total time          " << std::setw(27) << std::fixed << std::setprecision(4) << stats.total << "s <<";
    LOG << "--------------------------------------------------------";
}

}  // namespace hplus

#endif