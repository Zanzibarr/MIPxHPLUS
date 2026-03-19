/**
 * Methods for the main parts of this project execution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include "execution.hpp"
#include "instance.hpp"
#include "statistics.hpp"

namespace hplus {

void read_file(execution& exec, instance& inst, statistics& stats);

void run(execution& exec, instance& inst, statistics& stats);

void update_sol(instance& inst, const solution& sol, statistics& stats);

}  // namespace hplus