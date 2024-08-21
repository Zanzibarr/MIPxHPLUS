#ifndef RANKOOH_MODEL_H
#define RANKOOH_MODEL_H

#include "hplus_instance.hpp"
#include <cplex.h>

/**
 * Build the model from Rankooh's paper
 */
void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst);

/**
 * Store the solution found by Imai's model
 */
void HPLUS_store_rankooh_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst);

#endif