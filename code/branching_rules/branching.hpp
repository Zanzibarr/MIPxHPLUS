/**
 * Methods for CPLEX branching callback
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_BRANCHING_HPP
#define HPLUS_BRANCHING_HPP

#include <cplex.h>

#include "../domain/hplus_algs.hpp"

namespace callbacks {

void branching_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, hplus::statistics& stats);

}

#endif