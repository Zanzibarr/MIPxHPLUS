#include <deque>

#include "../external/pq.hpp"
#include "../utils/algorithms.hpp"
#include "preprocessing.hpp"

void mypause() {
    std::string _;
    std::cin >> _;
}

std::pair<int, int> hmax(const std::vector<unsigned int>& preconditions, const std::vector<int>& hmax_values) {
    int pcf{-1}, hmax{-1};
    for (const auto& p : preconditions) {
        if (hmax < hmax_values[p]) {
            hmax = hmax_values[p];
            pcf = p;
        }
    }
    return {pcf, hmax};
}

void update_hmax_values(const hplus::instance& inst, std::vector<int>& hmax_values, std::vector<int>& pcf, std::vector<int>& pcf_hmax,
                        const std::vector<int>& reduced_costs, const std::vector<unsigned int>& modified_actions) {
    priority_queue<int> pq{inst.n};

    for (const auto& act_i : modified_actions) {
        const int new_cost{pcf_hmax[act_i] + reduced_costs[act_i]};
        for (const auto& p : inst.actions[act_i].eff_sparse) {
            if (new_cost >= hmax_values[p]) continue;

            hmax_values[p] = new_cost;
            if (pq.has(p))
                pq.change(p, new_cost);
            else
                pq.push(p, new_cost);
        }
    }

    while (!pq.empty()) {
        const auto p{static_cast<int>(pq.top())};
        pq.pop();

        for (const auto& act_i : inst.act_with_pre[p]) {
            // If this action's pcf is not p, than since we are lowering the hmax values, the hmax won't change for this action... skip
            // ... or if p is act_i's pcf but it's hmax is that same value, nothing would change... skip
            if (pcf[act_i] != -1 && (pcf[act_i] != p || hmax_values[p] == pcf_hmax[act_i])) continue;

            const int old_hmax{pcf_hmax[act_i]};

            // Compute hmax and the pcf
            const auto& [act_pcf, act_hmax]{hmax(inst.actions[act_i].pre_sparse, hmax_values)};

            // If this action has no pcf or it's infinite, skip
            if (act_pcf == -1 || act_hmax == std::numeric_limits<int>::max()) continue;

            // Update the pcf
            pcf[act_i] = act_pcf;
            pcf_hmax[act_i] = act_hmax;

            // If the hmax of this action hasnt changed, skip
            if (pcf_hmax[act_i] == old_hmax) continue;

            // Update all the action's effects and add them to the queue
            const auto& new_cost{act_hmax + reduced_costs[act_i]};
            for (const auto& eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= hmax_values[eff]) continue;

                hmax_values[eff] = new_cost;
                if (pq.has(eff))
                    pq.change(eff, new_cost);
                else
                    pq.push(eff, new_cost);
            }
        }
    }
}

int compute_cut(hplus::instance& inst, const std::vector<int>& hmax_values, const std::vector<int>& pcf, std::vector<int>& reduced_costs,
                const std::vector<unsigned int>& goal_sparse, const std::vector<unsigned int>& initial_actions) {
    binary_set pre_goal_section(inst.n), goal_section(inst.n);
    std::deque<int> section_detect_queue;
    // Simulate a 0-cost action with precondition the goal state -> set its pcf as starting goal_section
    int goal_pcf{hmax(goal_sparse, hmax_values).first};
    goal_section.add(goal_pcf);
    section_detect_queue.push_back(goal_pcf);

    binary_set explored(inst.m);

    // Compute the goal section
    while (!section_detect_queue.empty()) {
        const auto p{section_detect_queue.front()};
        section_detect_queue.pop_front();

        for (const auto& act_i : inst.act_with_eff[p]) {
            // If I already explored this action or if it has no pcf, skip...
            if (explored[act_i] || pcf[act_i] == -1) continue;
            explored.add(act_i);

            // If it is a 0 reduced-cost (non-initial) action, than its pcf is also in the goal zone
            if (reduced_costs[act_i] == 0 && pcf[act_i] != static_cast<int>(inst.n)) {
                if (!goal_section[pcf[act_i]]) {
                    goal_section.add(pcf[act_i]);
                    section_detect_queue.push_back(pcf[act_i]);
                }
            }
        }
    }

    std::vector<unsigned int> cut;
    int min_redcost_cut{std::numeric_limits<int>::max()};

    explored.clear();

    const auto& check_cut = [&inst, &reduced_costs, &cut, &min_redcost_cut, &section_detect_queue, &pre_goal_section, &goal_section,
                             &explored](unsigned int act_i) {
        explored.add(act_i);
        bool added_to_cut{false};
        for (const auto& eff : inst.actions[act_i].eff_sparse) {
            if (goal_section[eff]) {
                if (reduced_costs[act_i] != 0 && !added_to_cut) {
                    cut.push_back(act_i);
                    min_redcost_cut = std::min(min_redcost_cut, reduced_costs[act_i]);
                    added_to_cut = true;
                }
            } else if (!pre_goal_section[eff]) {
                pre_goal_section.add(eff);
                section_detect_queue.push_back(eff);
            }
        }
    };

    // Compute the pre_goal section
    for (const auto& act_i : initial_actions) check_cut(act_i);

    while (!section_detect_queue.empty()) {
        const auto p{section_detect_queue.front()};
        section_detect_queue.pop_front();

        for (const auto& act_i : inst.act_with_pre[p]) {
            if (pcf[act_i] != p || explored[act_i] || pre_goal_section.contains(inst.actions[act_i].eff)) continue;
            check_cut(act_i);
        }
    }

    for (const auto& act_i : cut) reduced_costs[act_i] -= min_redcost_cut;

    inst.landmarks.push_back(std::move(cut));

    return min_redcost_cut;
}

void prep::lmcut_landmarks_extraction(hplus::instance& inst) {
    std::vector<int> hmax_values(inst.n, std::numeric_limits<int>::max()), pcf(inst.m, -1), pcf_hmax(inst.m, std::numeric_limits<int>::max());

    const std::vector<unsigned int> goal_sparse{inst.goal.sparse()};

    std::vector<int> reduced_costs(inst.m);
    for (unsigned int i = 0; i < inst.m; i++) reduced_costs[i] = static_cast<int>(inst.actions[i].cost);

    std::vector<unsigned int> initial_actions;
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            initial_actions.push_back(act_i);
            pcf[act_i] = static_cast<int>(inst.n);
            pcf_hmax[act_i] = 0;
        }
    }

    int lmcut_value{0};

    update_hmax_values(inst, hmax_values, pcf, pcf_hmax, reduced_costs, initial_actions);
    while (hmax(goal_sparse, hmax_values).second > 0) {
        lmcut_value += compute_cut(inst, hmax_values, pcf, reduced_costs, goal_sparse, initial_actions);
        update_hmax_values(inst, hmax_values, pcf, pcf_hmax, reduced_costs, inst.landmarks[inst.landmarks.size() - 1]);

        if (CHECK_STOP()) throw timelimit_exception("Reached time limit.");
    }

    LOG_INFO << "Computed a lm-cut value of: " << lmcut_value;
}