/**
 * Methods for the main parts of this project execution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_ALGS_HPP
#define HPLUS_ALGS_HPP

#include "execution.hpp"
#include "instance.hpp"
#include "statistics.hpp"

namespace hplus {

void read_file(execution& exec, instance& inst, statistics& stats);

void run(execution& exec, instance& inst, statistics& stats);

void update_sol(const execution& exec, instance& inst, const solution& sol, statistics& stats);

}  // namespace hplus

#define VERBOSE_STATS() exec.verbosity >= hplus::verbose::STATISTICS
#define VERBOSE_BASIC() exec.verbosity >= hplus::verbose::BASIC
#define VERBOSE_DEBUG() exec.verbosity >= hplus::verbose::DEBUG

#endif