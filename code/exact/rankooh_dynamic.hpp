/**
 * @file rankooh_dynamic.hpp
 * @brief Methods to find the optimal solution using the model proposed in Rankooh's paper (using vertex elimination) with a dynamic approach to
 * building and solving it (using callbacks)
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef RANKOOH_DYNAMIC_HPP
#define RANKOOH_DYNAMIC_HPP

#include <cplex.h>

#include "hplus_instance.hpp"
#include "log.hxx"
#include "utils.hpp"

namespace rankooh_dynamic {

/**
 * Build the cplex model using the instance described by the inst parameter, with execution details explained in the env parameter
 */
void build_cpx_model(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

/**
 * Post as warm start a solution to cplex
 */
void post_cpx_warmstart(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

/**
 * Store the solution found by cplex inside the inst instance
 */
void store_cpx_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const logger& log);

}  // namespace rankooh_dynamic

#endif /* RANKOOH_DYNAMIC_HPP */