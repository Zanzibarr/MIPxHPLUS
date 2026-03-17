#include "lmcut.hpp"

#include <algorithm>
#include <deque>
#include <limits>

#include "bs.hxx"
#include "limits.hxx"
#include "logger.hxx"
#include "utils.hpp"

auto hmax::hmax_arbitrary(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                          const std::vector<double>& /*initial_hmax_values*/) -> std::pair<int, double> {
    int pcf{-1};
    double hmax{-1};
    for (const auto& pre : preconditions) {
        if (hmax_values[pre] > hmax + HPLUS_EPSILON) {
            hmax = hmax_values[pre];
            pcf = static_cast<int>(pre);
        }
    }
    return {pcf, hmax};
}

auto hmax::hmax_inverse(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                        const std::vector<double>& /*initial_hmax_values*/) -> std::pair<int, double> {
    int pcf{-1};
    double hmax{-1};
    for (const auto& pre : preconditions) {
        if (hmax_values[pre] >= hmax - HPLUS_EPSILON) {
            hmax = hmax_values[pre];
            pcf = static_cast<int>(pre);
        }
    }
    return {pcf, hmax};
}

auto hmax::hmax_value_decrease_minimization(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                            const std::vector<double>& initial_hmax_values) -> std::pair<int, double> {
    int pcf{-1};
    int count{0};
    double hmax{-1};
    double min_decrease{std::numeric_limits<double>::infinity()};
    for (const auto& pre : preconditions) {
        if (hmax_values[pre] > hmax + HPLUS_EPSILON) {
            hmax = hmax_values[pre];
            pcf = static_cast<int>(pre);
            min_decrease = initial_hmax_values[pre] - hmax_values[pre];
            count = 1;
        } else if (std::abs(hmax - hmax_values[pre]) <= HPLUS_EPSILON) {
            // First level tie-breaking: pcf with the smallest value decrease since the first hmax iteration
            if (initial_hmax_values[pre] - hmax_values[pre] < min_decrease - HPLUS_EPSILON) {
                pcf = static_cast<int>(pre);
                min_decrease = initial_hmax_values[pre] - hmax_values[pre];
                count = 1;
            } else if (std::abs(initial_hmax_values[pre] - hmax_values[pre] - min_decrease) <= HPLUS_EPSILON) {
                // Second level tie-breaking: random
                count++;
                std::uniform_int_distribution<int> dist(1, count);
                if (dist(g_rng) == 1) {
                    pcf = static_cast<int>(pre);
                }
            }
        }
    }
    return {pcf, hmax};
}

auto hmax::hmax_random(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                       const std::vector<double>& /*initial_hmax_values*/) -> std::pair<int, double> {
    int pcf{-1};
    int count{0};
    double hmax{-1};
    for (const auto& pre : preconditions) {
        if (hmax_values[pre] > hmax + HPLUS_EPSILON) {
            hmax = hmax_values[pre];
            pcf = static_cast<int>(pre);
            count = 1;
        } else if (std::abs(hmax - hmax_values[pre]) <= HPLUS_EPSILON) {
            count++;
            std::uniform_int_distribution<int> dist(1, count);
            if (dist(g_rng) == 1) {
                pcf = static_cast<int>(pre);
            }
        }
    }

    return {pcf, hmax};
}

void LMcut::init() {
    pcf_ = std::vector<int>(inst_->m);
    hmax_values_ = std::vector<double>(inst_->n, std::numeric_limits<double>::infinity());
    pcf_hmax_ = std::vector<double>(inst_->m);
    reduced_costs_ = std::vector<double>(inst_->m);
    goal_ = inst_->goal.sparse();
    initial_actions_.clear();

    for (unsigned int i = 0; i < inst_->m; i++) {
        if (inst_->actions[i].pre_sparse.empty()) {
            initial_actions_.push_back(i);
            pcf_[i] = static_cast<int>(inst_->n);
            pcf_hmax_[i] = 0;
        } else {
            pcf_[i] = -1;
            pcf_hmax_[i] = std::numeric_limits<double>::infinity();
        }
        reduced_costs_[i] = static_cast<double>(inst_->actions[i].cost);
    }
}

void LMcut::update_and_enqueue_effects_values(priority_queue<double>& queue, unsigned int act_i) {
    const double new_cost{pcf_hmax_[act_i] + reduced_costs_[act_i]};
    for (const auto& eff : inst_->actions[act_i].eff_sparse) {
        // h_max(eff) = min_a{cost(a) + max{h_max{pre}}}
        if (new_cost >= hmax_values_[eff] - HPLUS_EPSILON) {
            continue;
        }

        hmax_values_[eff] = new_cost <= HPLUS_EPSILON ? 0 : new_cost;  // Fix numerical issues for close-to-0 values
        if (queue.has(eff)) {
            queue.change(eff, new_cost);
        } else {
            queue.push(eff, new_cost);
        }
    }
}

void LMcut::update_hmax_values(const std::vector<unsigned int>& changed_actions, hmax_function hmax) {
    priority_queue<double> queue{inst_->n};

    for (const auto& act_i : changed_actions) {
        update_and_enqueue_effects_values(queue, act_i);
    }

    while (!queue.empty()) {
        const auto fact{static_cast<int>(queue.top())};
        queue.pop();

        for (const auto& act_i : inst_->act_with_pre[fact]) {
            // If this action's pcf is not 'fact', than since we are lowering the hmax values, the hmax won't change for this action... skip
            if (pcf_[act_i] != -1 && pcf_[act_i] != fact) {
                continue;
            }

            const double old_hmax{pcf_hmax_[act_i]};

            // Compute hmax and the pcf
            const auto& [act_pcf, act_hmax]{hmax(inst_->actions[act_i].pre_sparse, hmax_values_, initial_hmax_values_)};

            // If this action has no pcf or it's infinite, skip
            if (act_pcf == -1 || act_hmax == std::numeric_limits<double>::infinity()) {
                continue;
            }

            // Update the pcf
            pcf_[act_i] = act_pcf;
            pcf_hmax_[act_i] = act_hmax <= HPLUS_EPSILON ? 0 : act_hmax;  // Fix numerical issues for close-to-0 values

            // If the hmax of this action hasnt changed, skip
            if (std::abs(pcf_hmax_[act_i] - old_hmax) <= HPLUS_EPSILON) {
                continue;
            }

            update_and_enqueue_effects_values(queue, act_i);
        }
    }
}

auto LMcut::compute_goal_section(hmax_function hmax) -> binary_set {
    binary_set goal_section(inst_->n);
    std::deque<int> queue;

    // Simulate a 0-cost action with precondition the goal state -> set its pcf as starting goal_section
    int goal_pcf{hmax(goal_, hmax_values_, initial_hmax_values_).first};
    goal_section.add(goal_pcf);
    queue.push_back(goal_pcf);

    binary_set explored(inst_->m);

    // Compute the goal section
    while (!queue.empty()) {
        const auto fact{queue.front()};
        queue.pop_front();

        for (const auto& act_i : inst_->act_with_eff[fact]) {
            // If I already explored this action or if it has no pcf, skip...
            if (explored[act_i] || pcf_[act_i] == -1) {
                continue;
            }

            explored.add(act_i);

            // If it is a 0 reduced-cost (non-initial) action, than its pcf is also in the goal zone
            // non-initial check: I cannot add to the queue the source node, since no action could ever achieve it
            if (reduced_costs_[act_i] <= HPLUS_EPSILON && pcf_[act_i] != static_cast<int>(inst_->n)) {
                if (!goal_section[pcf_[act_i]]) {
                    goal_section.add(pcf_[act_i]);
                    queue.push_back(pcf_[act_i]);
                }
            }
        }
    }

    return goal_section;
}

auto LMcut::compute_cut(hmax_function hmax) -> std::pair<std::vector<unsigned int>, double> {
    const auto& goal_section = compute_goal_section(hmax);

    // Compute the pre_goal section and the cut
    std::vector<unsigned int> cut;
    binary_set pre_goal_section(inst_->n);
    binary_set explored(inst_->m);
    std::deque<int> queue;

    const auto& check_update_cut_pregoal = [&](unsigned int act_i) -> void {
        explored.add(act_i);

        if (reduced_costs_[act_i] > HPLUS_EPSILON) {
            binary_set added_facts(inst_->n);
            for (const auto& eff : inst_->actions[act_i].eff_sparse) {
                if (goal_section[eff]) {
                    cut.push_back(act_i);
                    // Early exit condition... if this action crosses the cut there's no need to add its non-goal_section effects to the
                    // pre_goal_section
                    //
                    // Case 1: This is the only action from the pre_goal_section that achieves a fact p not in the goal_section... actions added to
                    // the cut because of this fact are not necessary for the validity of the landmark (the current action is a landmark for p (with
                    // the current pcf) hence adding actions that generated from p (or from successive iterations generated from p) would only weaken
                    // the landmark)... now pre_goal and goal sections are not summing up to the totality of the facts, but we can consider as
                    // partition (pre_goal_section, P \ pre_goal_section) instead of (pre_goal_section, goal_section), which would still produce a
                    // valid landmark since it's a cut for a valid partition of the facts
                    //
                    // Case 2: There are other actions from the pre_goal_section (that don't cross the cut) that achieve fact p... then they are gonna
                    // add it to the pre_goal_section: the validity of the partition is preserved
                    return;
                }

                if (!pre_goal_section[eff] && !added_facts[eff]) {
                    added_facts.add(eff);
                }
            }

            // If we didn't exit early add the effects to the pre_goal section
            for (const auto& eff : added_facts) {
                pre_goal_section.add(eff);
                queue.push_back(static_cast<int>(eff));
            }

        } else {
            for (const auto& eff : inst_->actions[act_i].eff_sparse) {
                ASSERT(!goal_section[eff]);
                if (!pre_goal_section[eff]) {
                    pre_goal_section.add(eff);
                    queue.push_back(static_cast<int>(eff));
                }
            }
        }
    };

    for (const auto& act_i : initial_actions_) {
        check_update_cut_pregoal(act_i);
    }

    while (!queue.empty()) {
        const auto fact{queue.front()};
        queue.pop_front();

        for (const auto& act_i : inst_->act_with_pre[fact]) {
            if (pcf_[act_i] != fact || explored[act_i] || pre_goal_section.contains(inst_->actions[act_i].eff)) {
                continue;
            }
            check_update_cut_pregoal(act_i);
        }
    }

    // auto size = cut.size();

    // Note that this MUST be done after the cut has been computed 'cause before the pre_goal might change...
    binary_set removed(inst_->m);
    for (const auto& act_i : cut) {
        // If an action in the cut has a precondition that's outside of the pre_goal section, then simply changing that pcf would remove this
        // action from the cut (while the rest of the graph remains unchanged: given that the pcf is not in the pre_goal, the pre_goal won't be
        // expanded by using this pcf instead... moreover the goal section cannot change, since this can't be a 0-cost action, otherwise the pcf
        // would be in the goal section too, and this action wouldn't be in the cut)
        if (!pre_goal_section.contains(inst_->actions[act_i].pre)) {
            removed.add(act_i);
        }
    }
    std::erase_if(cut, [&removed](const auto& elem) { return removed[elem]; });
    explored -= removed;

    // unsigned int removed_first = size - cut.size();

    // binary_set unapplicable_actions(inst_->m);
    // for (unsigned int act_i = 0; act_i < inst_->m; act_i++) {
    //     if (!pre_goal_section.contains(inst_->actions[act_i].pre)) {
    //         unapplicable_actions.add(act_i);
    //     }
    // }

    // // Further try to minimize the landmark
    // int_lm_sep::landmark_minimalization(*inst_, cut, unapplicable_actions, pre_goal_section);

    // LOG_DEBUG << "Removal: " << removed_first << " + " << size - cut.size() - removed_first << " = " << size - cut.size() << " actions over " <<
    // size;

    double min_reduced_cost = std::numeric_limits<double>::infinity();
    for (const auto& act_i : cut) {
        min_reduced_cost = std::min(min_reduced_cost, reduced_costs_[act_i]);
    }

    for (const auto& act_i : cut) {
        reduced_costs_[act_i] -= min_reduced_cost;
        if (reduced_costs_[act_i] <= HPLUS_EPSILON) {
            reduced_costs_[act_i] = 0;  // Fix numerical issues for close-to-0 values
        }
    }

    return {cut, min_reduced_cost};
}

LMcut::LMcut(const hplus::instance& inst) : inst_(&inst) {}

auto LMcut::compute_lmcut_private(hmax_function hmax) -> std::pair<std::vector<std::vector<unsigned int>>, double> {
    double lmcut_value{0};
    std::vector<std::vector<unsigned int>> landmarks;

    update_hmax_values(initial_actions_, hmax);

    initial_hmax_values_ = std::vector<double>(hmax_values_.begin(), hmax_values_.end());

    while (hmax(goal_, hmax_values_, initial_hmax_values_).second > HPLUS_EPSILON) {
        const auto& [cut, val] = compute_cut(hmax);
        check_landmark(cut);  // This is an (expensive) integrity check... //! //FIXME don't use this in runs where performance is measured
        lmcut_value += val;
        update_hmax_values(cut, hmax);
        landmarks.push_back(std::move(cut));

        if (CHECK_STOP()) {
            throw timelimit_exception("Reached time limit.");
        }
    }

    // for (const auto& landmark : landmarks) {
    //     LOG_DEBUG << "Size: " << landmark.size();
    // }

    return {landmarks, lmcut_value};
}

auto LMcut::compute_lmcut(hmax_function hmax) -> std::pair<std::vector<std::vector<unsigned int>>, double> {
    init();

    return compute_lmcut_private(hmax);
}

auto LMcut::int_separation(const std::vector<unsigned int>& used_actions, hmax_function hmax)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>> {
    init();

    // Set reduced costs of used actions to 0
    for (const auto& idx : used_actions) {
        reduced_costs_[idx] = 0;
    }

    const auto& [landmarks, lmcut_val] = compute_lmcut_private(hmax);

    return {!landmarks.empty(), landmarks};
}

auto LMcut::fract_separation(const std::vector<double>& actions_weights, hmax_function hmax)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>> {
    init();

    for (unsigned int i = 0; i < actions_weights.size(); i++) {
        reduced_costs_[i] = reduced_costs_[i] * (1 - actions_weights[i]);
    }

    auto [landmarks, lmcut_val] = compute_lmcut_private(hmax);

    std::erase_if(landmarks, [&actions_weights](const std::vector<unsigned int>& landmark) {
        double sum = 0;
        for (const auto& act_i : landmark) {
            sum += actions_weights[act_i];
        }
        return sum >= 1;
    });

    return {!landmarks.empty(), landmarks};
}

void LMcut::check_landmark(const std::vector<unsigned int>& landmark) {
    // This function is meant to be a check for debugging... it's not optimized to be used as a routine
    bool found = false;
    binary_set state(inst_->n);
    binary_set remaining_actions(inst_->m, true);
    for (const auto& act_i : landmark) {
        remaining_actions.remove(act_i);
    }

    while (!found) {
        auto state_before = state;
        for (const auto& act_i : remaining_actions) {
            const auto& act = inst_->actions[act_i];
            if (state.contains(act.pre)) {
                remaining_actions.remove(act_i);
                state |= act.eff;
            }
        }
        if (state.contains(inst_->goal)) {
            found = true;
        }
        if (state_before == state) {
            break;  // Valid landmark: without its actions we can't reach the goal
        }
    }

    if (found) {  // If this was a valid landmark we wouldn't be able to find a valid plan to the goal
        LOG_WARNING << vtos(landmark, landmark.size());
        LOG_WARNING << "inst_->m: " << inst_->m;
        LOG_WARNING << "GOAL:" << std::string(inst_->goal);
        for (const auto& act_i : landmark) {
            LOG_WARNING << "Act " << act_i;
            LOG_WARNING << "Cost: " << inst_->actions[act_i].cost;
            LOG_WARNING << "PRE: " << std::string(inst_->actions[act_i].pre);
            LOG_WARNING << "EFF: " << std::string(inst_->actions[act_i].eff);
        }
        LOG_ERROR << "Found invalid landmark.";
    }
}