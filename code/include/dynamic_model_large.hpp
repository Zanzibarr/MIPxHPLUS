#ifndef DYNAMIC_LARGE_HPP
#define DYNAMIC_LARGE_HPP

#include "hplus_instance.hpp"
#include <cplex.h>

void HPLUS_cpx_build_dynamic_large(CPXENVptr& env, CPXLPptr& lp);

void HPLUS_cpx_post_warmstart_dynamic_large(CPXENVptr& env, CPXLPptr& lp);

void HPLUS_store_dynamic_large_sol(const CPXENVptr& env, const CPXLPptr& lp);

#endif