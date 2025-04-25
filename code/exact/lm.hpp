/**
 * @file lm.hpp
 * @brief Methods to find the optimal solution modeling acyclicity using landmarks ans S.E.C.
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef LM_HPP
#define LM_HPP

#include <cplex.h>

#include "hplus_instance.hpp"
#include "utils.hpp"

namespace lm {

typedef struct {
    hplus::instance& inst;
    hplus::environment& env;
    hplus::statistics& stats;
    const logger& log;
    CPXENVptr cpx_fenv;
    CPXLPptr cpx_flp;
} cpx_callback_user_handle;

/**
 * Callback hub for the lm model
 */
int CPXPUBLIC cpx_callback_hub(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle);

/**
 * Building che model to extract cuts from the fractional solution
 */
void cpx_create_fract_model(const hplus::instance& inst, const logger& log, CPXENVptr& cpx_fenv, CPXLPptr& cpx_flp);

/**
 * Closing the model used to extract cuts from the fractional solution
 */
void cpx_close_fract_model(CPXENVptr& cpx_fenv, CPXLPptr& cpx_flp, const logger& log);

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

#endif /* LM_HPP */