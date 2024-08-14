#include "../include/utils.hpp"
#include "../include/hplus_instance.hpp"
#include <cplex.h>

/**
 * Build the model from Imai's paper
 */
void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst);

/**
 * Store the solution found by Imai's model
 */
void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst);