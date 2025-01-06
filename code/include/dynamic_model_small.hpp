#ifndef MY_MODEL_HPP
#define MY_MODEL_HPP

#include "hplus_instance.hpp"
#include <cplex.h>

void HPLUS_cpx_build_dynamic_small(CPXENVptr& env, CPXLPptr& lp);

void HPLUS_cpx_post_warmstart_dynamic_small(CPXENVptr& env, CPXLPptr& lp);

void HPLUS_store_dynamic_small_sol(const CPXENVptr& env, const CPXLPptr& lp);

#endif