/**
 * @file algorithms.hpp
 * @brief Interface for using algorithms to solve the delete-free relaxation of the planning task
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef ALGS_H
#define ALGS_H

#include <cplex.h>
#include "utils.hpp"
#include "hplus_instance.hpp"

// ====================================================== //
// ===================== CPLEX UTILS ==================== //
// ====================================================== //

void cpx_init(CPXENVptr& env, CPXLPptr& lp, const hplus::environment& _e, const logger& _l);

void cpx_close(CPXENVptr& env, CPXLPptr& lp);

bool parse_cpx_status(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, hplus::environment& _e, const logger& _l);

// ====================================================== //
// ===================== HEURISTICS ===================== //
// ====================================================== //

void find_heuristic(hplus::instance& _i, hplus::environment& _e, const logger& _l);

// ====================================================== //
// ======================== IMAI ======================== //
// ====================================================== //

void cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void cpx_post_warmstart_imai(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void store_imai_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l);

// ====================================================== //
// ======================= RANKOOH ====================== //
// ====================================================== //

void cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void cpx_post_warmstart_rankooh(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void store_rankooh_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l);

// ====================================================== //
// ==================== DYNAMIC SMALL =================== //
// ====================================================== //

void cpx_build_dynamic_small(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void cpx_post_warmstart_dynamic_small(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void store_dynamic_small_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l);

// ====================================================== //
// ==================== DYNAMIC LARGE =================== //
// ====================================================== //

void cpx_build_dynamic_large(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void cpx_post_warmstart_dynamic_large(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l);

void store_dynamic_large_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l);

#endif /* ALGS_H */