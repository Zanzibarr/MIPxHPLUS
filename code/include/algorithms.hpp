/**
 * @file algorithms.hpp
 * @brief Interface for using algorithms to solve the delete-free relaxation of the planning task
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef ALGS_H
#define ALGS_H

#include "hplus_instance.hpp"
#include <cplex.h> // For CPLEX C API

// ====================================================== //
// ===================== CPLEX UTILS ==================== //
// ====================================================== //

void cpx_init(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::environment& env, const logger& log, bool log_file = true);

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp);

[[nodiscard]]
bool parse_cpx_status(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, hplus::environment& env, const logger& log);

// ====================================================== //
// ===================== HEURISTICS ===================== //
// ====================================================== //

void find_heuristic(hplus::instance& inst, hplus::environment& env, const logger& log);

// ====================================================== //
// ======================== IMAI ======================== //
// ====================================================== //

void cpx_build_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log, bool relaxed = false);

void cpx_post_warmstart_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

void store_imai_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log);

// ====================================================== //
// ======================= RANKOOH ====================== //
// ====================================================== //

void cpx_build_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

void cpx_post_warmstart_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

void store_rankooh_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log);

// ====================================================== //
// ==================== DYNAMIC SMALL =================== //
// ====================================================== //

void cpx_build_dynamic_small(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

void cpx_post_warmstart_dynamic_small(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

void store_dynamic_small_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log);

// ====================================================== //
// ==================== DYNAMIC LARGE =================== //
// ====================================================== //

void cpx_build_dynamic_large(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

void cpx_post_warmstart_dynamic_large(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log);

void store_dynamic_large_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log);

#endif /* ALGS_H */