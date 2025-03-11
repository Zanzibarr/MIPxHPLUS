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

void run_heur(hplus::instance& inst, hplus::environment& env, const logger& log);
void run_model(hplus::instance& inst, hplus::environment& env, hplus::statistics& stats, const logger& log);

#endif /* ALGS_H */