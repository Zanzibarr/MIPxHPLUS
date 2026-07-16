#include <binary_set.hxx>

#include "solver.hpp"

void Solver::prep_initial_action_sequencing_(std::vector<std::vector<unsigned int>>& landmarks) {
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
    while (!candidates.empty()) {
        BinarySet new_effects{inst_.n};
        for (const auto act_i : candidates) {  // Order is irrelevant since they are all applicable
            myassert(bs_contains(state, inst_.actions[act_i].pre_sparse), "Applying an action whose preconditions are not satisfied");
            used_actions.push_back(act_i);
            global_.cost_prefix += inst_.actions[act_i].cost;
            global_.solution_prefix.push_back(inst_.actions_names[act_i]);
            for (const auto eff : inst_.actions[act_i].eff_sparse) {
                if (state[eff]) {
                    continue;
                }
                new_effects.add(eff);
            }
            state |= inst_.actions[act_i].eff_sparse;
        }

        // Update watch preconditions and candidates list
        candidates.clear();
        BinarySet new_candidates{inst_.m};
        for (const auto eff : new_effects) {
            for (unsigned int i = 0; i < watching[eff].size();) {
                // No filter needed here: only fixed/0-cost actions are ever inserted into the watch lists (see the init loop above)
                const auto act_i = watching[eff][i];
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
                watching[eff][i] = watching[eff].back();
                watching[eff].pop_back();

                if (moved || new_candidates[act_i]) {
                    continue;
                }
                new_candidates.add(act_i);
                candidates.push_back(act_i);
            }
        }
    }

    if (state.empty()) {  // No action applied
        return;
    }

    // The prefix alone reaches the goal: signal optimality to the caller (via its goal-empty check) instead of removing the reached facts
    // The prefix has already been stored, so altering the goal state to exit early is fine
    if (state.superset_of(inst_.goal)) {
        inst_.goal = BinarySet{inst_.n};
        return;
    }

    // ~~ Remove used actions (remaps fixed actions and recounts nfadd, the repetition trigger for 'l') ~~ //
    global_.eliminated_actions = BinarySet{inst_.m};
    global_.eliminated_actions |= used_actions;
    prep_eliminated_actions_();

    // ~~ Remove all reached facts (remaps goal, landmarks and fixed facts, and recounts nfadd) ~~ //
    // Reassigned (not cleared) since earlier fact eliminations in this pass may have shrunk inst_.n below the BinarySet capacity
    global_.eliminated_facts = state;
    prep_eliminated_facts_(landmarks);
}
