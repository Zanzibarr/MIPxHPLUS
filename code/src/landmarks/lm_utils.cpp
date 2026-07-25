#include <binary_set.hxx>
#include <timer.hxx>

#include "bs_utils.hpp"
#include "solver.hpp"

auto Solver::check_landmark_(const BinarySet& landmark) -> bool {
    // TODO: Remove
    auto _tmr = make_scoped_timer<"reach_test">(stats_);

    // Goal facts still to be reached. The moment this hits 0 the goal is relaxed-reachable,
    // so the orbit is NOT a landmark and we can bail out without saturating the rest of the
    // reachable set (which is the common case: most orbits aren't landmarks).
    std::size_t goal_remaining = inst_.goal.size();
    if (goal_remaining == 0) {
        return false;
    }

    BinarySet state{inst_.n};
    std::vector<unsigned int> watch_pre(inst_.m, inst_.n);
    std::vector<std::vector<unsigned int>> watching(inst_.n);
    std::vector<unsigned int> candidates;
    std::vector<unsigned int> new_effects;  // reused across rounds, holds only the newly reached facts
    BinarySet queued(inst_.m);              // dedup marker for the next round's candidates

    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        // We don't want to use actions in the landmark
        if (landmark[act_i]) {
            continue;
        }

        // An action with no effects can never change the state: never track it
        if (inst_.actions[act_i].eff_sparse.empty()) {
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

    while (!candidates.empty()) {
        new_effects.clear();
        for (const auto act_i : candidates) {  // Order is irrelevant since they are all applicable
            myassert(bs_contains(state, inst_.actions[act_i].pre_sparse), "Applying an action whose preconditions are not satisfied");
            for (const auto eff : inst_.actions[act_i].eff_sparse) {
                if (state[eff]) {
                    continue;
                }
                // Mark reached immediately: this dedups new_effects across actions in the same round
                // and folds the old separate `state |= eff_sparse` pass into this single traversal.
                state.add(eff);
                new_effects.push_back(eff);
                if (inst_.goal[eff] && --goal_remaining == 0) {
                    return false;  // whole goal reached -> not a landmark
                }
            }
        }

        // Update watch preconditions and candidates list
        candidates.clear();
        for (const auto eff : new_effects) {
            // eff is now reached, so pre != eff for any unsatisfied pre below; watching[pre] is
            // therefore always a different list than watchers, keeping this reference stable.
            auto& watchers = watching[eff];
            for (std::size_t i = 0; i < watchers.size();) {
                // No filter needed here: non-landmark actions are ever inserted into the watch lists (see the init loop above)
                const auto act_i = watchers[i];
                myassert(!landmark[act_i], "Non fixed/0-cost action found in a watch list");

                // Swap-erase from watching[eff] regardless (this fact is now reached, so no action is watching it as its watch_pre)
                watchers[i] = watchers.back();
                watchers.pop_back();

                // If every effect is already reached, applying the action can't change the state anymore. Since an action lives in exactly one watch
                // list (the one of its watch_pre), dropping it here removes it for good: later rounds won't even see it.
                if (bs_contains(state, inst_.actions[act_i].eff_sparse)) {
                    watch_pre[act_i] = inst_.n;
                    continue;
                }

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

                if (moved || queued[act_i]) {
                    continue;
                }
                queued.add(act_i);
                candidates.push_back(act_i);
            }
        }
        // Reset the dedup markers for exactly the actions we just queued (O(#new candidates))
        for (const auto act_i : candidates) {
            queued.remove(act_i);
        }
    }

    return !state.superset_of(inst_.goal);
}

void Solver::landmark_minimalization_greedy_(std::vector<unsigned int>& landmark, const BinarySet& reachable_state) {
    std::unordered_set<unsigned int> removed;
    for (const auto& act_i : landmark) {
        // If an action in the landmark has a precondition that's outside of the pre_goal section, then simply changing that pcf would remove this
        // action from the landmark (while the rest of the graph remains unchanged: given that the pcf is not in the pre_goal, the pre_goal won't be
        // expanded by using this pcf instead... moreover the goal section cannot change, since this can't be a 0-cost action, otherwise the pcf
        // would be in the goal section too, and this action wouldn't be in the landmark)
        if (!bs_contains(reachable_state, inst_.actions[act_i].pre_sparse)) {
            removed.insert(act_i);
        }
    }
    std::erase_if(landmark, [&removed](const auto& elem) { return removed.contains(elem); });
}

void Solver::landmark_minimalization_(std::vector<unsigned int>& landmark, const std::vector<unsigned int>& unapplicable, BinarySet reachable_state) {
    // Watch lists
    std::vector<unsigned int> watch_pre(inst_.m + 1, inst_.n);  // Watch precondition of each action (as default we use a non-existant fact)
    std::vector<std::vector<unsigned int>> watching(inst_.n);   // List of actions that are watching each fact
    std::unordered_set<unsigned int> to_remove;

    const unsigned int goal_act = inst_.m;

    const auto check_update_watch_pre = [&](unsigned int act_i, bool init = false) -> void {
        myassert(init || watch_pre[act_i] != inst_.n, "Wrongly initialized watch precondition");

        const unsigned int previous = watch_pre[act_i];
        // If the current watch_pre isn't reachable, there's no need to change it
        if (!init && !reachable_state[previous]) {
            return;
        }

        for (const auto& pre : inst_.actions[act_i].pre_sparse) {
            if (reachable_state[pre]) {
                continue;
            }

            // Update the watch_pre and the watching list ...
            watch_pre[act_i] = pre;
            watching[pre].push_back(act_i);
            // ... remembering to remove previous occurrences of this action in the previous watching list
            if (!init /*&& watch_pre[act_i] != previous -- this is commented due to the check at the beginning of the function*/) {
                myassert(std::ranges::find(watching[previous], act_i) != watching[previous].end(),
                         "Action was not found among those watching this fact");
                const auto pos = static_cast<unsigned int>(std::ranges::find(watching[previous], act_i) - watching[previous].begin());
                const auto size = static_cast<unsigned int>(watching[previous].size());
                watching[previous][pos] = watching[previous][size - 1];
                watching[previous].resize(size - 1);
            }
            break;
        }

        // If we're in an initialization call and we no unreachable precondition is found (for example when iterating on the landmark) then set its
        // watch_pre to an arbitrary precondition (to avoid out-of-bounds checks triggering)
        if (init && watch_pre[act_i] == inst_.n) {
            watch_pre[act_i] = inst_.actions[act_i].pre_sparse[0];
        }

        myassert(watch_pre[act_i] != inst_.n, "Watch precondition not set");
    };

    const auto check_update_goal_watch_pre = [&](bool init = false) -> void {
        myassert(init || watch_pre[goal_act] != inst_.n, "Wrongly initialized watch precondition in goal facts");
        // If the current watch_pre isn't reachable, there's no need to change it
        if (!init && !reachable_state[watch_pre[goal_act]]) {
            return;
        }
        for (const auto& g_fact : global_.goal_sparse) {
            if (reachable_state[g_fact]) {
                continue;
            }
            watch_pre[goal_act] = g_fact;
            break;
        }

        myassert(watch_pre[goal_act] != inst_.n, "Watch precondition not set for goal action");
    };

    for (const auto& act_i : unapplicable) {
        myassert(!inst_.actions[act_i].pre_sparse.empty(), "Empty preconditions of an unapplicable action");
        check_update_watch_pre(act_i, true);
    }

    // Keep track of the watch_pre of the goal
    check_update_goal_watch_pre(true);

    for (const auto& act_i : landmark) {
        const auto& action = inst_.actions[act_i];

        // If the effects of this action won't change the reachable state, just apply it
        if (bs_contains(reachable_state, action.eff_sparse)) {
            to_remove.insert(act_i);
            continue;
        }

        // Initialize watch_pre lazily: only register this landmark action into the watch lists
        // now, so that unreachable landmark actions enter 'watching' only when discovered
        // (mirroring insert_sorted(unapplicable, act_i) in the WATCH_LIT=0 branch).
        // Registering all landmark actions upfront caused them to chain their effects during
        // the simulation of earlier actions, producing different results than WATCH_LIT=0.
        if (!action.pre_sparse.empty()) {
            check_update_watch_pre(act_i, true);
        }

        // If the action is unreachable, then it won't change the set of reachable facts -> I can remove it
        if (!action.pre_sparse.empty() && !reachable_state[watch_pre[act_i]]) {
            to_remove.insert(act_i);
            continue;
        }

        // Here the action is reachable and it has new effects
        std::vector<unsigned int> new_effects;

        // Look at the chain-effects of each effect
        std::queue<unsigned int> eff_queue;
        for (const auto& eff : action.eff_sparse) {
            if (reachable_state[eff]) {
                continue;
            }
            eff_queue.push(eff);
        }

        while (!eff_queue.empty()) {
            const auto fact = eff_queue.front();
            eff_queue.pop();

            // This is a new fact... is it is achieved in a previous iteration we simply skip it
            if (reachable_state[fact]) {
                continue;
            }

            reachable_state.add(fact);
            new_effects.push_back(fact);

            // Early exit condition
            check_update_goal_watch_pre();
            if (reachable_state[watch_pre[goal_act]]) {
                break;
            }

            // Look at actions whose watch_pre is fact
            // Use index-based loop: check_update_watch_pre may erase act_j from watching[fact],
            // in which case the next element shifts to position i, so we don't advance.
            for (size_t i = 0; i < watching[fact].size();) {
                const auto act_j = watching[fact][i];
                // Update its watch_pre
                check_update_watch_pre(act_j);
                // If act_j was erased from watching[fact], it shifted out of position i
                if (i < watching[fact].size() && watching[fact][i] == act_j) {
                    ++i;
                }
                // If it's reached, then this action is applicable
                if (!reachable_state[watch_pre[act_j]]) {
                    continue;
                }
                for (const auto& new_eff : inst_.actions[act_j].eff_sparse) {
                    if (reachable_state[new_eff]) {
                        continue;
                    }
                    eff_queue.push(new_eff);
                }
            }
        }

        check_update_goal_watch_pre();

        // If after all the chain effects the goal is reachable, we need to revert them... note that we don't need to revert the changes to the
        // watch_pre and watching vectors, because if watch_pre of an (unachievable) action changed, it can't have changed to a fact that was achieved
        // pre-simulation, so the new watch_pre is still not achieved in the pre-simulation envorinment that we are reversing to (same with watching,
        // since they get updated simultaneously)
        if (reachable_state[watch_pre[goal_act]]) {
            for (const auto& fact : new_effects) {
                reachable_state.remove(fact);
            }
        }
        // Otherwise we can remove this action from the landmark
        else {
            to_remove.insert(act_i);
        }
    }

    // Compute the landmark as the set of actions that are unused, but not in the extension
    std::erase_if(landmark, [&to_remove](const auto& elem) { return to_remove.contains(elem); });
}
