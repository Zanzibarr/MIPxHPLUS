/**
 * @file algorithms.hpp
 * @brief Interface for using algorithms to solve the delete-free relaxation of
 * the planning task
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef ALGS_H
#define ALGS_H

#include <cplex.h>  // For CPLEX C API

#include "hplus_instance.hpp"

// ====================================================== //
// ===================== CPLEX UTILS ==================== //
// ====================================================== //

void cpx_init(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::environment& env, const logger& log, bool log_file = true);

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp);

[[nodiscard]]
bool parse_cpx_status(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::environment& env, const logger& log);

// ====================================================== //
// ===================== HEURISTICS ===================== //
// ====================================================== //

void run_heur(hplus::instance& inst, hplus::environment& env, const logger& log);

double find_hmax_goal(const hplus::instance& inst, const logger& log);

// ====================================================== //
// ======================== MODEL ======================= //
// ====================================================== //

void cpx_build_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log,
                    bool relaxed = false);

void run_model(hplus::instance& inst, hplus::environment& env, hplus::statistics& stats, const logger& log);

#endif /* ALGS_H */