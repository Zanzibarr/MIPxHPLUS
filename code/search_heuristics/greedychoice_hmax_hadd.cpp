#include <algorithm>
#include <vector>

#include "heuristic.hpp"

namespace {

inline void update_hadd_values(const hplus::instance& inst, const std::vector<unsigned int>& new_facts, heur::greedychoice_userhandle& uh) {
    BinarySet trail_flags{inst.n};
    BinarySet action_trail_flags{inst.m};

    for (const auto& fact : new_facts) {
        uh.trail.emplace(fact, uh.values[fact]);
        trail_flags.add(fact);

        const double old_val = uh.values[fact];
        uh.values[fact] = 0;
        uh.pq.push(fact, 0);

        // Propagate the decrease to hadd_pre of dependent actions
        for (const auto& act_j : inst.act_with_pre[fact]) {
            if (!action_trail_flags[act_j]) {
                uh.action_trail.emplace(act_j, uh.hadd_pre[act_j]);
                action_trail_flags.add(act_j);
            }
            if (old_val >= std::numeric_limits<double>::infinity() || uh.hadd_pre[act_j] >= std::numeric_limits<double>::infinity()) {
                // Infinity involved — recompute from scratch with updated values[]
                double sum = 0;
                for (const auto& pre : inst.actions[act_j].pre_sparse) {
                    sum += uh.values[pre];
                }
                uh.hadd_pre[act_j] = sum;
            } else {
                uh.hadd_pre[act_j] -= old_val;  // delta = old_val - 0
            }
        }
    }

    while (!uh.pq.empty()) {
        const auto fact{uh.pq.top()};
        uh.pq.pop();

        for (const auto& act_i : inst.act_with_pre[fact]) {
            if (uh.used_actions.contains(act_i)) {
                continue;
            }

            const double cost_pre{uh.hadd_pre[act_i]};  // O(1) incremental lookup

            if (cost_pre >= std::numeric_limits<double>::infinity()) {
                continue;
            }

            const double new_cost{cost_pre + inst.actions[act_i].cost};
            for (const auto& p_eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= uh.values[p_eff]) {
                    continue;
                }

                const double old_eff_val = uh.values[p_eff];

                if (!trail_flags[p_eff]) {
                    uh.trail.emplace(p_eff, old_eff_val);
                    trail_flags.add(p_eff);
                }
                uh.values[p_eff] = new_cost;
                if (uh.pq.has(p_eff)) {
                    uh.pq.change(p_eff, new_cost);
                } else {
                    uh.pq.push(p_eff, new_cost);
                }

                // Propagate the decrease to hadd_pre of dependent actions
                const double eff_delta = old_eff_val - new_cost;
                for (const auto& act_j : inst.act_with_pre[p_eff]) {
                    if (!action_trail_flags[act_j]) {
                        uh.action_trail.emplace(act_j, uh.hadd_pre[act_j]);
                        action_trail_flags.add(act_j);
                    }
                    if (old_eff_val >= std::numeric_limits<double>::infinity() || uh.hadd_pre[act_j] >= std::numeric_limits<double>::infinity()) {
                        double sum = 0;
                        for (const auto& pre : inst.actions[act_j].pre_sparse) {
                            sum += uh.values[pre];
                        }
                        uh.hadd_pre[act_j] = sum;
                    } else {
                        uh.hadd_pre[act_j] -= eff_delta;
                    }
                }
            }
        }
    }
}

inline void update_hmax_values(const hplus::instance& inst, const std::vector<unsigned int>& new_facts, heur::greedychoice_userhandle& uh) {
    BinarySet trail_flags{inst.n};
    BinarySet action_trail_flags{inst.m};

    for (const auto& fact : new_facts) {
        uh.trail.emplace(fact, uh.values[fact]);
        trail_flags.add(fact);
        uh.values[fact] = 0;
        uh.pq.push(fact, 0);
    }

    while (!uh.pq.empty()) {
        const auto fact{uh.pq.top()};
        uh.pq.pop();

        for (const auto& act_i : inst.act_with_pre[fact]) {
            if (uh.used_actions.contains(act_i)) {
                continue;
            }

            // PCF skip: if we know the argmax precondition and it is not the
            // current fact, the action's hmax cannot have decreased — skip.
            if (uh.pcf[act_i] != -1 && uh.pcf[act_i] != static_cast<int>(fact)) {
                continue;
            }

            // Recompute argmax over preconditions
            int new_pcf = -1;
            double new_pcf_val = -1;
            for (const auto& pre : inst.actions[act_i].pre_sparse) {
                if (uh.values[pre] > new_pcf_val) {
                    new_pcf_val = uh.values[pre];
                    new_pcf = static_cast<int>(pre);
                }
            }

            if (new_pcf == -1 || new_pcf_val >= std::numeric_limits<double>::infinity()) {
                continue;
            }

            // Record old pcf_val in action_trail before updating
            if (!action_trail_flags[act_i]) {
                uh.action_trail.emplace(act_i, uh.pcf_val[act_i]);
                action_trail_flags.add(act_i);
            }
            uh.pcf[act_i] = new_pcf;
            uh.pcf_val[act_i] = new_pcf_val;

            const double new_cost{new_pcf_val + inst.actions[act_i].cost};
            for (const auto& p_eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= uh.values[p_eff]) {
                    continue;
                }

                if (!trail_flags[p_eff]) {
                    uh.trail.emplace(p_eff, uh.values[p_eff]);
                    trail_flags.add(p_eff);
                }
                uh.values[p_eff] = new_cost;
                if (uh.pq.has(p_eff)) {
                    uh.pq.change(p_eff, new_cost);
                } else {
                    uh.pq.push(p_eff, new_cost);
                }
            }
        }
    }
}
}  // namespace

void heur::init_htype_values(const hplus::instance& inst, const std::list<unsigned int>& initial_actions, std::vector<double>& values,
                             priority_queue<double>& pq, double (*h_eqtype)(double, double)) {
    for (const auto& act_i : initial_actions) {
        const unsigned int cost{inst.actions[act_i].cost};
        for (const auto& eff : inst.actions[act_i].eff_sparse) {
            if (cost >= values[eff]) {
                continue;
            }
            values[eff] = cost;
            if (pq.has(eff)) {
                pq.change(eff, cost);
            } else {
                pq.push(eff, cost);
            }
        }
    }

    // Generic Dijkstra relaxation (initialization only — no trail, no incremental state)
    while (!pq.empty()) {
        const auto fact{static_cast<unsigned int>(pq.top())};
        pq.pop();

        for (const auto& act_i : inst.act_with_pre[fact]) {
            double cost_pre{0};
            for (const auto& pre : inst.actions[act_i].pre_sparse) {
                cost_pre = h_eqtype(cost_pre, values[pre]);
            }
            if (cost_pre >= std::numeric_limits<double>::infinity()) {
                continue;
            }
            const double new_cost{cost_pre + inst.actions[act_i].cost};
            for (const auto& p_eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= values[p_eff]) {
                    continue;
                }
                values[p_eff] = new_cost;
                if (pq.has(p_eff)) {
                    pq.change(p_eff, new_cost);
                } else {
                    pq.push(p_eff, new_cost);
                }
            }
        }
    }
}

[[nodiscard]]
auto heur::greedy_choice_hmax(const hplus::instance& inst, const std::list<unsigned int>& candidates, const BinarySet& state,
                              heur::greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    double best_hmax = std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto& act_i : candidates) {
        if (inst.fixed_actions[act_i] || inst.actions[act_i].cost == 0) {
            found = true;
            best_choice = act_i;
            break;
        }

        std::vector<unsigned int> new_eff(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        update_hmax_values(inst, new_eff, userhandle);

        // hmax of goal
        double hmax_value = 0;
        for (const auto& goal_fact : userhandle.goal_sparse) {
            hmax_value = std::max(userhandle.values[goal_fact], hmax_value);
        }

        // Restore fact values via trail
        while (!userhandle.trail.empty()) {
            auto [p, old_value] = userhandle.trail.top();
            userhandle.trail.pop();
            userhandle.values[p] = old_value;
        }
        // Restore pcf from action_trail by recomputing from the restored values[]
        while (!userhandle.action_trail.empty()) {
            const unsigned int act_j = userhandle.action_trail.top().first;
            userhandle.action_trail.pop();
            int restored_pcf = -1;
            double restored_pcf_max = -1;
            for (const auto& pre : inst.actions[act_j].pre_sparse) {
                if (userhandle.values[pre] > restored_pcf_max) {
                    restored_pcf_max = userhandle.values[pre];
                    restored_pcf = static_cast<int>(pre);
                }
            }
            userhandle.pcf[act_j] = restored_pcf;
            userhandle.pcf_val[act_j] = (restored_pcf_max >= 0) ? restored_pcf_max : 0.0;
        }

        if (hmax_value >= best_hmax) {
            continue;
        }

        best_choice = act_i;
        best_hmax = hmax_value;
        found = true;
    }

    // Permanent commit for best_choice (trails discarded, state permanently updated)
    userhandle.used_actions.add(best_choice);
    std::vector<unsigned int> new_eff(inst.actions[best_choice].eff_sparse.begin(), inst.actions[best_choice].eff_sparse.end());
    std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
    update_hmax_values(inst, new_eff, userhandle);
    userhandle.trail = std::stack<std::pair<unsigned int, double>>{};
    userhandle.action_trail = std::stack<std::pair<unsigned int, double>>{};

    return {found, best_choice};
}

[[nodiscard]]
auto heur::greedy_choice_hadd(const hplus::instance& inst, const std::list<unsigned int>& candidates, const BinarySet& state,
                              heur::greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    double best_hadd = std::numeric_limits<double>::infinity();
    bool found = false;
    double current_hadd = 0;
    for (const auto& goal_fact : userhandle.goal_sparse) {
        current_hadd += userhandle.values[goal_fact];
    }

    for (const auto& act_i : candidates) {
        if (inst.fixed_actions[act_i] || inst.actions[act_i].cost == 0) {
            found = true;
            best_choice = act_i;
            break;
        }

        std::vector<unsigned int> new_eff(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        update_hadd_values(inst, new_eff, userhandle);

        double hadd_value = current_hadd;

        // Restore fact values via trail; accumulate delta on goal facts for hadd_value
        while (!userhandle.trail.empty()) {
            const auto [p, old_value] = userhandle.trail.top();
            userhandle.trail.pop();
            if (inst.goal[p]) {
                hadd_value -= (old_value - userhandle.values[p]);
            }
            userhandle.values[p] = old_value;
        }
        // Restore hadd_pre via action_trail
        while (!userhandle.action_trail.empty()) {
            const auto [act_j, old_hadd_pre] = userhandle.action_trail.top();
            userhandle.action_trail.pop();
            userhandle.hadd_pre[act_j] = old_hadd_pre;
        }

        if (hadd_value >= best_hadd) {
            continue;
        }

        best_choice = act_i;
        best_hadd = hadd_value;
        found = true;
    }

    // Permanent commit for best_choice (trails discarded, state permanently updated)
    userhandle.used_actions.add(best_choice);
    std::vector<unsigned int> new_eff(inst.actions[best_choice].eff_sparse.begin(), inst.actions[best_choice].eff_sparse.end());
    std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
    update_hadd_values(inst, new_eff, userhandle);
    userhandle.trail = std::stack<std::pair<unsigned int, double>>{};
    userhandle.action_trail = std::stack<std::pair<unsigned int, double>>{};

    return {found, best_choice};
}
