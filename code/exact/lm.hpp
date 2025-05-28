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

#include <map>

#include "hplus_instance.hpp"
#include "utils.hpp"

namespace lm {

typedef struct {
    int usercuts_lm, usercuts_sec;
    double cand_time, relax_time;
    CPXENVptr lmcutenv;
    CPXLPptr lmcutlp;
} thread_data;

typedef struct {
    hplus::instance& inst;
    hplus::environment& env;
    const logger& log;
    CPXENVptr lmcutenv;
    CPXLPptr lmcutlp;
    std::vector<thread_data> thread_specific_data;
} cpx_callback_user_handle;

/**
 * Callback hub for the lm model
 */
int CPXPUBLIC cpx_callback_hub(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle);

/**
 * Building che model to extract cuts from the fractional solution
 */
void cpx_create_lmcut_model(const hplus::instance& inst, const logger& log, CPXENVptr& lmcutenv, CPXLPptr& lmcutlp);

/**
 * Closing the model used to extract cuts from the fractional solution
 */
void cpx_close_lmcut_model(CPXENVptr& lmcutenv, CPXLPptr& lmcutlp, const logger& log);

/**
 * Create the data for each thread for callbacks
 */
void create_thread_data(const hplus::instance& inst, const hplus::environment& env, cpx_callback_user_handle& callback_data, const logger& log);

/**
 * Sync data gathered among callbacks and correctly free all data allocated for each
 */
void sync_and_close_threads(const hplus::environment& env, hplus::statistics& stats, cpx_callback_user_handle& callback_data, const logger& log);

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