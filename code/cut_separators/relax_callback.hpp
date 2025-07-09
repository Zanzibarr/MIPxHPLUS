#ifndef HPLUS_RELAX_CALLBACK_HPP
#define HPLUS_RELAX_CALLBACK_HPP

#include <cplex.h>

#include "../domain/hplus_algs.hpp"
#include "../utils/algorithms.hpp"

namespace relax_cuts {
[[nodiscard]]
std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> relaxationpoint_info(const hplus::instance& inst,
                                                                                                  std::vector<double>& relax_point);

[[nodiscard]]
std::pair<bool, std::vector<unsigned int>> get_violated_landmark(CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec,
                                                                 const hplus::instance& inst, const std::vector<double>& relax_point);

[[nodiscard]]
unsigned int lm(CPXCALLBACKCONTEXTptr context, CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec, const hplus::instance& inst,
                const std::vector<double>& relax_point);

[[nodiscard]]
std::vector<unsigned int> get_violated_sec(const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights,
                                           const hplus::instance& inst);

[[nodiscard]]
unsigned int sec(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst,
                 const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights);

}  // namespace relax_cuts

namespace callbacks {

struct thread_data {
    unsigned int usercuts_lm, usercuts_sec;
    double cand_time, relax_time;
    CPXENVptr flmdet_env;
    CPXLPptr flmdet_lp;
};

void relaxation_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, thread_data& data);

}  // namespace callbacks

#endif
