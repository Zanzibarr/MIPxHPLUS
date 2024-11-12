#ifndef IMAI_MODEL_H
#define IMAI_MODEL_H

#include "hplus_instance.hpp"
#include <cplex.h>

void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp);

void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp);

#endif