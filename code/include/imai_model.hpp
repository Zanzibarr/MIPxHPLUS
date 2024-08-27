#ifndef IMAI_MODEL_H
#define IMAI_MODEL_H

#include "hplus_instance.hpp"
#include <queue>
#include <cplex.h>

void HPLUS_imai_iterative_variable_elimination(const HPLUS_instance& inst, my::BitField& eliminated_variables, my::BitField& fixed_variables, my::BitField& eliminated_actions, my::BitField& fixed_actions, std::vector<my::BitField>& eliminated_fas, std::vector<my::BitField>& fixed_fas, std::vector<int>& fixed_var_timestamps, std::vector<int>& fixed_act_timestamps);

/**
 * Build the model from Imai's paper
 */
void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst);

/**
 * Store the solution found by Imai's model
 */
void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst);

#endif