
#include "int_separators.hpp"

#include <vector>

#include "bs.hxx"
#include "bs_utils.hpp"
#include "landmark_utils.hpp"
#include "lmcut.hpp"
#include "stats_registry.hxx"
#include "timer.hxx"
#include "utils.hpp"

[[nodiscard]]
auto int_lm_sep::get_lmcut_violated_landmarks(const hplus::instance& inst, const std::vector<double>& xstar)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>> {
    auto _cand_lm = make_scoped_timer<"cand_lm_separator">(STATS);
    LMcut lmcut(inst);

    std::vector<unsigned int> used_actions;
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (xstar[act_i] > HPLUS_CPX_INT_ROUNDING) {
            used_actions.push_back(act_i);
        }
    }

    const auto& [found, landmarks] = lmcut.int_separation(used_actions, hmax::hmax_arbitrary);
    for (const auto& landmark : landmarks) {
        STATS.gauge_record<"cand_lm_size">(landmark.size());
    }
    return {found, landmarks};
}

[[nodiscard]]
auto int_lm_sep::get_comp_violated_landmark(const hplus::instance& inst, const std::vector<unsigned int>& unreachable_actions,
                                            const std::vector<unsigned int>& unused_actions, const BinarySet& reachable_state)
    -> std::vector<unsigned int> {
    auto _cand_lm = make_scoped_timer<"cand_lm_separator">(STATS);
    std::vector<unsigned int> landmark(unused_actions.begin(), unused_actions.end());
    STATS.gauge_record<"cand_lm_size">(landmark.size());
    lmutils::landmark_minimalization(inst, landmark, unreachable_actions, reachable_state);
    return landmark;
}

[[nodiscard]]
auto int_lm_sep::get_front_violated_landmark(const hplus::instance& inst, const std::vector<unsigned int>& unused_actions,
                                             const BinarySet& reachable_state) -> std::vector<unsigned int> {
    auto _cand_lm = make_scoped_timer<"cand_lm_separator">(STATS);
    std::vector<unsigned int> landmark;
    for (unsigned int act_i : unused_actions) {
        if (bs_contains(reachable_state, inst.actions[act_i].pre_sparse) && !bs_contains(reachable_state, inst.actions[act_i].eff_sparse)) {
            landmark.push_back(act_i);
        }
    }
    STATS.gauge_record<"cand_lm_size">(landmark.size());
    return landmark;
}