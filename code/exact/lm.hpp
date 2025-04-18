/**
 * @file lm.hpp
 * @brief Methods to find the optimal solution using the model proposed in Rankooh's paper (using vertex elimination) with a lm approach to
 * building and solving it (using callbacks)
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef DYNAMIC_HPP
#define DYNAMIC_HPP

#include <cplex.h>

#include "hplus_instance.hpp"
#include "log.hxx"
#include "utils.hpp"

namespace lm {

typedef struct {
    hplus::instance& inst;
    hplus::environment& env;
    hplus::statistics& stats;
    const logger& log;
} cpx_callback_user_handle;

/**
 * Callback for the lm model
 */
int CPXPUBLIC cpx_callback(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle);

/**
 * Build the cplex model using the instance described by the inst parameter, with execution details explained in the env parameter
 */
void build_cpx_model(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log,
                     hplus::statistics& stats);

/**
 * Post as warm start a solution to cplex
 */
void post_cpx_warmstart(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

/**
 * Store the solution found by cplex inside the inst instance
 */
void store_cpx_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const logger& log);

}  // namespace lm

#endif /* DYNAMIC_HPP */