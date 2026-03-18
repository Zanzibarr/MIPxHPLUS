
#include "int_separators.hpp"

#include <vector>

#include "bs_utils.hpp"
#include "lmcut.hpp"
#include "stats_registry.hxx"
#include "timer.hxx"
#include "utils.hpp"

void int_lm_sep::landmark_minimalization(const hplus::instance& inst, std::vector<unsigned int>& landmark, BinarySet unapplicable_actions,
                                         BinarySet reachable_state) {
    BinarySet extension(inst.m);
    const auto& goal{inst.goal};
    for (const auto& act_i : landmark) {
        const auto& action = inst.actions[act_i];
        // If the effects of this action won't change the reachable state, just apply it
        if (bs_contains(reachable_state, action.eff_sparse)) {
            extension.add(act_i);
            continue;
        }

        // If the action is unreachable, then it won't change the set of reachable facts -> I can add it to the extension
        if (!bs_contains(reachable_state, action.pre_sparse)) {
            extension.add(act_i);
            // Add this action to the unapplicable actions, to eventually add its effects when it'd become applicable
            unapplicable_actions.add(act_i);
            continue;
        }

        // Here the action is reachable and it has new effects

        BinarySet state_sim(reachable_state);
        state_sim |= action.eff_sparse;
        // If the effect of this action reaches the goal, we don't add it to the extension
        if (state_sim.superset_of(goal)) {
            continue;
        }

        // Simulate the effects of using this action
        BinarySet new_reachable(inst.m);
        while (true) {
            bool skip{true};
            for (const auto& act_j : unapplicable_actions) {
                if (new_reachable[act_j]) {
                    continue;
                }
                // If now a (previously) unapplicable action is applicable, add its effects to the simulated state
                if (bs_contains(state_sim, inst.actions[act_j].pre_sparse)) {
                    new_reachable.add(act_j);
                    state_sim |= inst.actions[act_j].eff_sparse;
                    if (state_sim.superset_of(goal)) {
                        skip = true;
                        break;
                    }
                    skip = false;
                }
            }
            if (skip) {
                break;
            }
        }

        // If the new state doesn't contain the goal, then we can update the reachable state and remove the applicable actions from the previously
        // unapplicable ones
        if (!state_sim.superset_of(goal)) {
            extension.add(act_i);
            reachable_state = state_sim;
            unapplicable_actions -= new_reachable;
        }
    }

    // Compute the landmark as the set of actions that are unused, but not in the extension
    std::erase_if(landmark, [&extension](const auto& elem) { return extension[elem]; });
}

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
auto int_lm_sep::get_comp_violated_landmark(const hplus::instance& inst, const BinarySet& unreachable_actions,
                                            const std::vector<unsigned int>& unused_actions, const BinarySet& reachable_state)
    -> std::vector<unsigned int> {
    auto _cand_lm = make_scoped_timer<"cand_lm_separator">(STATS);
    std::vector<unsigned int> landmark(unused_actions.begin(), unused_actions.end());
    landmark_minimalization(inst, landmark, unreachable_actions, reachable_state);
    STATS.gauge_record<"cand_lm_size">(landmark.size());
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