#ifndef RANKOOH_MODEL_H
#define RANKOOH_MODEL_H

#include "hplus_instance.hpp"
#include <cplex.h>

void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp);

void HPLUS_cpx_post_warmstart_rankooh(CPXENVptr& env, CPXLPptr& lp);

void HPLUS_store_rankooh_sol(const CPXENVptr& env, const CPXLPptr& lp);

#endif