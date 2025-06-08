#ifndef HPLUS_RELAX_CALLBACK_HPP
#define HPLUS_RELAX_CALLBACK_HPP

#include <cplex.h>

#include "../domain/hplus_algs.hpp"
#include "../utils/algorithms.hpp"

namespace relax_cuts {

[[nodiscard]]
unsigned int lm(CPXCALLBACKCONTEXTptr context, CPXENVptr& env, CPXLPptr& lp, const hplus::instance& inst, const std::vector<double>& relax_point);

[[nodiscard]]
unsigned int sec(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst,
                 const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights);

}  // namespace relax_cuts

namespace callbacks {

typedef struct {
    unsigned int usercuts_lm, usercuts_sec;
    double cand_time, relax_time;
    CPXENVptr flmdet_env;
    CPXLPptr flmdet_lp;
} thread_data;

void relaxation_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, thread_data& data);

}  // namespace callbacks

#endif
