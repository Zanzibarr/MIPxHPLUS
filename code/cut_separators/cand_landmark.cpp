#include <algorithm>

#include "../utils/algorithms.hpp"
#include "cand_callback.hpp"

[[nodiscard]]
unsigned int cand_cuts::complementary_lm(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const binary_set& unreachable_actions,
                                         const std::vector<unsigned int>& unused_actions, const binary_set& reachable_state) {
    std::vector<unsigned int> unapplicable{unreachable_actions.sparse()};
    std::vector<unsigned int> extension;
    binary_set state{reachable_state};
    for (const auto& act_i : unused_actions) {
        ASSERT(!state.contains(inst.goal))
        // If the effects of this action won't change the reachable state, just apply it
        if (state.contains(inst.actions[act_i].eff)) {
            insert_sorted(extension, act_i);
            continue;
        }

        // If the action is unreachable, then it won't change the set of reachable facts -> I can add it to the excention
        if (!state.contains(inst.actions[act_i].pre)) {
            insert_sorted(extension, act_i);
            // Add this action to the unapplicable actions, to eventually add its effects when it'd become applicable
            insert_sorted(unapplicable, act_i);
            continue;
        }

        // Here the action is reachable and it has new effects

        // If the effect of this action reaches the goal, we don't add it to the extension
        binary_set state_sim{state | inst.actions[act_i].eff};
        if (state_sim.contains(inst.goal)) continue;

        // Simulate the effects of using this action
        std::vector<unsigned int> new_reachable;
        while (true) {
            bool skip{true};
            for (const auto& act_j : unapplicable) {
                if (std::binary_search(new_reachable.begin(), new_reachable.end(), act_j)) continue;
                // If now a (previously) unapplicable action is applicable, add its effects to the simulated state
                if (state_sim.contains(inst.actions[act_j].pre)) {
                    insert_sorted(new_reachable, act_j);
                    state_sim |= inst.actions[act_j].eff;
                    skip = false;
                }
            }
            if (skip) [[unlikely]]
                break;
        }

        // If the new state doesn't contain the goal, then we can update the reachable state and remove the applicable actions from the previously
        // unapplicable ones
        if (!state_sim.contains(inst.goal)) {
            insert_sorted(extension, act_i);
            state = state_sim;
            const auto& it{
                std::set_difference(unapplicable.begin(), unapplicable.end(), new_reachable.begin(), new_reachable.end(), unapplicable.begin())};
            unapplicable.resize(it - unapplicable.begin());
        }
    }

    // Compute the landmark as the set of actions that are unused, but not in the extension
    std::vector<unsigned int> landmark;
    std::set_difference(unused_actions.begin(), unused_actions.end(), extension.begin(), extension.end(), std::back_inserter(landmark));
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