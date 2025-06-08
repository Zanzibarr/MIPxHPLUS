#ifndef HPLUS_PREP_HPP
#define HPLUS_PREP_HPP

#include "../domain/hplus_algs.hpp"

namespace prep {

void landmark_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks);

void first_adders_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks);

void relevance_analysis_backward(hplus::instance& inst, binary_set& relevant_variables);

void relevance_analysis_forward(hplus::instance& inst, binary_set& relevant_variables);

void dominated_actions_extraction(hplus::instance& inst, const std::vector<std::vector<unsigned int>>& landmarks);

void eliminated_facts_removal(hplus::instance& inst, hplus::statistics& stats, std::vector<std::vector<unsigned int>>& landmarks);

void eliminated_actions_removal(hplus::instance& inst, hplus::statistics& stats);

inline void prepare_preprocessing(hplus::instance& inst) {
    inst.fixed_facts = binary_set{inst.n};
    inst.fixed_actions = binary_set{inst.m};
    inst.eliminated_facts = binary_set{inst.n};
    inst.eliminated_actions = binary_set{inst.m};
}

inline void preprocess(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    if (BASIC_VERBOSE()) LOG_INFO << "Preprocessing instance";

    double start_time = GET_TIME();
    stats.preprocessing = static_cast<double>(exec.timelimit) - start_time;

    std::vector<std::vector<unsigned int>> landmarks(inst.n);
    binary_set relevant_variables(inst.n);

    landmark_extraction(inst, landmarks);
    first_adders_extraction(inst, landmarks);
    relevance_analysis_backward(inst, relevant_variables);
    relevance_analysis_forward(inst, relevant_variables);
    eliminated_facts_removal(inst, stats, landmarks);  // Done here since no more facts will be eliminated, and this could make the next steps faster
    dominated_actions_extraction(inst, landmarks);
    eliminated_actions_removal(inst, stats);

    stats.preprocessing = GET_TIME() - start_time;
}

inline void prepare_optimization_helpers(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    if (BASIC_VERBOSE()) LOG_INFO << "Preparing optimization helpers";

    double start_time = GET_TIME();
    double prep_time_save = stats.preprocessing;
    stats.preprocessing += static_cast<double>(exec.timelimit) - start_time;

    inst.fadd_cpx_start.clear();
    inst.fadd_cpx_start.reserve(inst.m);
    unsigned int sum{0};
    for (const auto& act : inst.actions) {
        inst.fadd_cpx_start.push_back(sum);
        sum += act.eff_sparse.size();
    }
    inst.act_with_pre = std::vector<std::vector<unsigned int>>(inst.n);
    inst.act_with_eff = std::vector<std::vector<unsigned int>>(inst.n);
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        for (const auto& pre_i : inst.actions[act_i].pre_sparse) inst.act_with_pre[pre_i].push_back(act_i);
        for (const auto& eff_i : inst.actions[act_i].eff_sparse) inst.act_with_eff[eff_i].push_back(act_i);
    }

    inst.eliminated_facts = binary_set{1};
    inst.eliminated_actions = binary_set{1};

    stats.preprocessing = prep_time_save + GET_TIME() - start_time;
}

}  // namespace prep

#endif