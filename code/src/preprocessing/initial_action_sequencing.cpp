#include <binary_set.hxx>

#include "solver.hpp"

auto Solver::prep_initial_action_sequencing_(std::vector<std::vector<unsigned int>>& landmarks) -> bool {
    //  Apply fixed/0-cost actions to the prefix  //
    std::vector<unsigned int> watch_pre(inst_.m, inst_.n);
    std::vector<std::vector<unsigned int>> watching(inst_.n);
    BinarySet state{inst_.n};

    std::vector<unsigned int> candidates;  // List of applicable fixed/0-cost actions
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (!global_.fixed_actions[act_i] && inst_.actions[act_i].cost != 0) {
            continue;
        }

        if (inst_.actions[act_i].pre_sparse.empty()) {  // Initial state is empty
            candidates.push_back(act_i);
        } else {  // We know this action has at least one precondition, since the initial state is empty such precondition isn't achieved
            const auto pre = inst_.actions[act_i].pre_sparse[0];
            watch_pre[act_i] = pre;
            watching[pre].push_back(act_i);
        }
    }

    std::vector<unsigned int> used_actions;  // keep track of used actions, so we can eliminated them at the end
    std::vector<unsigned int> new_effects;   // reused across rounds, holds only the newly reached facts
    std::vector<char> queued(inst_.m, 0);    // dedup marker for the next round's candidates
    while (!candidates.empty()) {
        new_effects.clear();
        for (const auto act_i : candidates) {  // Order is irrelevant since they are all applicable
            myassert(bs_contains(state, inst_.actions[act_i].pre_sparse), "Applying an action whose preconditions are not satisfied");
            used_actions.push_back(act_i);
            global_.cost_prefix += inst_.actions[act_i].cost;
            global_.solution_prefix.push_back(inst_.actions_names[act_i]);
            logger_[DEBUG] << std::format("Added to prefix: A{}", act_i);
            for (const auto eff : inst_.actions[act_i].eff_sparse) {
                if (state[eff]) {
                    continue;
                }
                // Mark reached immediately: dedups new_effects across actions in the same round and
                // folds the old separate `state |= eff_sparse` pass into this single traversal.
                state.add(eff);
                new_effects.push_back(eff);
            }
        }

        // Update watch preconditions and candidates list
        candidates.clear();
        for (const auto eff : new_effects) {
            // eff is now reached, so pre != eff for any unsatisfied pre below; watching[pre] is
            // therefore always a different list than watchers, keeping this reference stable.
            auto& watchers = watching[eff];
            for (std::size_t i = 0; i < watchers.size();) {
                // No filter needed here: only fixed/0-cost actions are ever inserted into the watch lists (see the init loop above)
                const auto act_i = watchers[i];
                myassert(global_.fixed_actions[act_i] || inst_.actions[act_i].cost == 0, "Non fixed/0-cost action found in a watch list");

                // Try to find a new unsatisfied precondition to watch
                bool moved = false;
                for (const auto pre : inst_.actions[act_i].pre_sparse) {
                    if (!state[pre]) {
                        watch_pre[act_i] = pre;
                        watching[pre].push_back(act_i);
                        moved = true;
                        break;
                    }
                }

                // Swap-erase from watching[eff] regardless (this fact is now reached, so no action is watchin it as its watch_pre)
                watchers[i] = watchers.back();
                watchers.pop_back();

                if (moved || queued[act_i] != 0) {
                    continue;
                }
                queued[act_i] = 1;
                candidates.push_back(act_i);
            }
        }
        // Reset the dedup markers for exactly the actions we just queued (O(#new candidates))
        for (const auto act_i : candidates) {
            queued[act_i] = 0;
        }

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("constructing initial prefix", EarlyExit::TIMELIMIT);
        }
    }

    if (state.empty()) {  // No action applied: the instance is unchanged
        return false;
    }

    // The prefix alone reaches the goal: signal optimality to the caller (via its goal-empty check) instead of removing the reached facts
    // The prefix has already been stored, so altering the goal state to exit early is fine
    if (state.superset_of(inst_.goal)) {
        inst_.goal = BinarySet{inst_.n};
        return true;  // caller exits on the empty goal, so this value is only for consistency
    }

    // ~~ Remove used actions (remaps fixed actions and recounts nfadd, the repetition trigger for 'l') ~~ //
    global_.eliminated_actions = BinarySet{inst_.m};
    global_.eliminated_actions |= used_actions;
    prep_eliminated_actions_();

    // ~~ Remove all reached facts (remaps goal, landmarks and fixed facts, and recounts nfadd) ~~ //
    // Reassigned (not cleared) since earlier fact eliminations in this pass may have shrunk inst_.n below the BinarySet capacity
    global_.eliminated_facts = state;
    prep_eliminated_facts_(landmarks);

    return true;  // actions and/or facts were removed: the instance (and thus the symmetry graph) changed
}
