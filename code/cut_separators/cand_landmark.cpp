#include <algorithm>

#include "../external/pq.hpp"
#include "../utils/algorithms.hpp"
#include "cand_callback.hpp"

[[nodiscard]]
unsigned int cand_cuts::complementary_lm(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const binary_set& unreachable_actions,
                                         const std::vector<unsigned int>& unused_actions, const binary_set& reachable_state) {
    binary_set unapplicable{unreachable_actions}, extension(inst.m), state{reachable_state};
    const auto& goal{inst.goal};
    for (const auto& act_i : unused_actions) {
        const auto& action = inst.actions[act_i];
        // If the effects of this action won't change the reachable state, just apply it
        if (state.contains(action.eff)) {
            extension.add(act_i);
            continue;
        }

        // If the action is unreachable, then it won't change the set of reachable facts -> I can add it to the extension
        if (!state.contains(action.pre)) {
            extension.add(act_i);
            // Add this action to the unapplicable actions, to eventually add its effects when it'd become applicable
            unapplicable.add(act_i);
            continue;
        }

        // Here the action is reachable and it has new effects

        // If the effect of this action reaches the goal, we don't add it to the extension
        binary_set state_sim{state | action.eff};
        if (state_sim.contains(goal)) continue;

        // Simulate the effects of using this action
        binary_set new_reachable(inst.m);
        while (true) {
            bool skip{true};
            for (const auto& act_j : unapplicable) {
                if (new_reachable[act_j]) continue;
                // If now a (previously) unapplicable action is applicable, add its effects to the simulated state
                if (state_sim.contains(inst.actions[act_j].pre)) {
                    new_reachable.add(act_j);
                    state_sim |= inst.actions[act_j].eff;
                    skip = false;
                    if (state_sim.contains(goal)) {
                        skip = true;
                        break;
                    }
                }
            }
            if (skip) break;
        }

        // If the new state doesn't contain the goal, then we can update the reachable state and remove the applicable actions from the previously
        // unapplicable ones
        if (!state_sim.contains(goal)) {
            extension.add(act_i);
            state = state_sim;
            unapplicable -= new_reachable;
        }
    }

    // Compute the landmark as the set of actions that are unused, but not in the extension
    std::vector<unsigned int> landmark;
    for (const auto& act_i : unused_actions) {
        if (!extension[act_i]) landmark.push_back(act_i);
    }
    reject_with_lm_cut(context, landmark);
    return 1;
}

[[nodiscard]]
unsigned int cand_cuts::frontier_lm(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const std::vector<unsigned int>& unused_actions,
                                    const binary_set& reachable_state) {
    std::vector<unsigned int> landmark;
    for (unsigned int act_i : unused_actions) {
        if (reachable_state.contains(inst.actions[act_i].pre) && !reachable_state.contains(inst.actions[act_i].eff)) landmark.push_back(act_i);
    }
    reject_with_lm_cut(context, landmark);
    return 1;
}