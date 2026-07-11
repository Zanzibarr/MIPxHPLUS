#include "solver.hpp"

void Solver::hplus_separate_cand_lmfront_cut_(CPXCALLBACKCONTEXTptr context) {
    auto _sep_timer = scoped_timer("cand_lm_sep");

    std::vector<unsigned int> landmark;
    for (unsigned int act_i : local_.cand_unused_actions) {
        if (bs_contains(local_.cand_reachable_state, inst_.actions[act_i].pre_sparse)) {
            if (!bs_contains(local_.cand_reachable_state, inst_.actions[act_i].eff_sparse)) {
                landmark.push_back(act_i);
            }
        }
    }

    hplus_reject_cand_lm_(context, {landmark});
}

void Solver::hplus_separate_cand_lmcomp_cut_(CPXCALLBACKCONTEXTptr context) {
    auto _sep_timer = scoped_timer("cand_lm_sep");

    std::vector<unsigned int> landmark(local_.cand_unused_actions.begin(), local_.cand_unused_actions.end());
    landmark_minimalization_(landmark, local_.cand_unreachable_actions, local_.cand_reachable_state);

    hplus_reject_cand_lm_(context, {landmark});
}

void Solver::hplus_separate_relax_lm_cut_(CPXCALLBACKCONTEXTptr /*context*/, bool /*minimization*/) {
    // TODO
    unimplemented();
}

void Solver::hplus_reject_cand_lm_(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& landmarks) {
    for (const auto& landmark : landmarks) {
        stats_.gauge_record<"cand_lm_size">(static_cast<double>(landmark.size()));
        stats_.counter_inc<"cand_lm">();

        std::vector<int> ind(landmark.begin(), landmark.end());
        std::vector<double> val(landmark.size(), 1.0);
        constexpr double rhs{1.0};
        constexpr char sense{'G'};
        constexpr int begin{0};
        call_cplex(CPXcallbackrejectcandidate(context, 1, static_cast<int>(landmark.size()), &rhs, &sense, &begin, ind.data(), val.data()));
    }
}

void Solver::hplus_reject_relax_lm_(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& landmarks) {
    for (const auto& landmark : landmarks) {
        stats_.gauge_record<"relax_lm_size">(static_cast<double>(landmark.size()));
        stats_.counter_inc<"relax_lm">();

        std::vector<int> ind(landmark.size());
        unsigned int nnz{0};
        for (const auto& act_i : landmark) {
            ind[nnz++] = static_cast<int>(act_i);
        }
        std::vector<double> val(landmark.size(), 1.0);
        constexpr double rhs{1};
        constexpr char sense{'G'};
        constexpr int begin{0};
        constexpr int purgeable{CPX_USECUT_FILTER};
        constexpr int local{0};
        call_cplex(CPXcallbackaddusercuts(context, 1, static_cast<int>(nnz), &rhs, &sense, &begin, ind.data(), val.data(), &purgeable, &local));
    }
}
