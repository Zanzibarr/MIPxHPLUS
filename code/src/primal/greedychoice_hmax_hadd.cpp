#include "solver.hpp"

void Solver::primal_hmaxadd_init_values_(const primal_hmax_function& h_eqtype) {
    for (const auto& act_i : global_.initial_actions) {
        const unsigned int cost{inst_.actions[act_i].cost};
        for (const auto& eff : inst_.actions[act_i].eff_sparse) {
            if (cost >= global_.primal_h_values[eff]) {
                continue;
            }
            global_.primal_h_values[eff] = cost;
            if (global_.primal_pq.has(eff)) {
                global_.primal_pq.change(eff, cost);
            } else {
                global_.primal_pq.push(eff, cost);
            }
        }
    }

    // Generic Dijkstra relaxation (initialization only — no trail, no incremental state)
    while (!global_.primal_pq.empty()) {
        const auto fact{static_cast<unsigned int>(global_.primal_pq.top())};
        global_.primal_pq.pop();

        for (const auto& act_i : global_.act_with_pre[fact]) {
            double cost_pre{0};
            for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                cost_pre = h_eqtype(cost_pre, global_.primal_h_values[pre]);
            }
            if (cost_pre >= std::numeric_limits<double>::infinity()) {
                continue;
            }
            const double new_cost{cost_pre + inst_.actions[act_i].cost};
            for (const auto& p_eff : inst_.actions[act_i].eff_sparse) {
                if (new_cost >= global_.primal_h_values[p_eff]) {
                    continue;
                }
                global_.primal_h_values[p_eff] = new_cost;
                if (global_.primal_pq.has(p_eff)) {
                    global_.primal_pq.change(p_eff, new_cost);
                } else {
                    global_.primal_pq.push(p_eff, new_cost);
                }
            }
        }
    }
}

void Solver::primal_update_hmax_values_(const std::vector<unsigned int>& new_facts) {
    for (const auto& fact : new_facts) {
        global_.primal_trail.emplace(fact, global_.primal_h_values[fact]);
        global_.primal_trail_flags.add(fact);
        global_.primal_h_values[fact] = 0;
        global_.primal_pq.push(fact, 0);
    }

    while (!global_.primal_pq.empty()) {
        const auto fact{global_.primal_pq.top()};
        global_.primal_pq.pop();

        for (const auto& act_i : global_.act_with_pre[fact]) {
            if (global_.primal_used_actions.contains(act_i)) {
                continue;
            }

            // PCF skip: if we know the argmax precondition and it is not the
            // current fact, the action's hmax cannot have decreased — skip.
            if (global_.primal_hmax_pcf[act_i] != -1 && global_.primal_hmax_pcf[act_i] != static_cast<int>(fact)) {
                continue;
            }

            // Recompute argmax over preconditions
            int new_pcf = -1;
            double new_pcf_val = -1;
            for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                if (global_.primal_h_values[pre] > new_pcf_val) {
                    new_pcf_val = global_.primal_h_values[pre];
                    new_pcf = static_cast<int>(pre);
                }
            }

            if (new_pcf == -1 || new_pcf_val >= std::numeric_limits<double>::infinity()) {
                continue;
            }

            // Record old pcf in action_trail before updating
            if (!global_.primal_action_trail_flags[act_i]) {
                global_.primal_action_trail.emplace(act_i, global_.primal_hmax_pcf[act_i], global_.primal_pcf_hmax[act_i]);
                global_.primal_action_trail_flags.add(act_i);
            }
            global_.primal_hmax_pcf[act_i] = new_pcf;
            global_.primal_pcf_hmax[act_i] = new_pcf_val;

            const double new_cost{new_pcf_val + inst_.actions[act_i].cost};
            for (const auto& p_eff : inst_.actions[act_i].eff_sparse) {
                if (new_cost >= global_.primal_h_values[p_eff]) {
                    continue;
                }

                if (!global_.primal_trail_flags[p_eff]) {
                    global_.primal_trail.emplace(p_eff, global_.primal_h_values[p_eff]);
                    global_.primal_trail_flags.add(p_eff);
                }
                global_.primal_h_values[p_eff] = new_cost;
                if (global_.primal_pq.has(p_eff)) {
                    global_.primal_pq.change(p_eff, new_cost);
                } else {
                    global_.primal_pq.push(p_eff, new_cost);
                }
            }
        }
    }
}

auto Solver::primal_greedychoice_hmax_(const std::list<unsigned int>& candidates, const BinarySet& state) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    double best_hmax = std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto& act_i : candidates) {
        if (global_.fixed_actions[act_i] || inst_.actions[act_i].cost == 0) {
            found = true;
            best_choice = act_i;
            break;
        }

        std::vector<unsigned int> new_eff(inst_.actions[act_i].eff_sparse.begin(), inst_.actions[act_i].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        primal_update_hmax_values_(new_eff);

        // hmax of goal
        double hmax_value = 0;
        for (const auto& goal_fact : global_.goal_sparse) {
            hmax_value = std::max(global_.primal_h_values[goal_fact], hmax_value);
        }

        // Restore fact values via trail
        while (!global_.primal_trail.empty()) {
            const auto [p, old_value] = global_.primal_trail.top();
            global_.primal_trail.pop();
            global_.primal_h_values[p] = old_value;
        }
        global_.primal_trail_flags.clear();
        // Restore pcf from action_trail
        while (!global_.primal_action_trail.empty()) {
            const auto [act_j, old_pcf, old_pcf_val] = global_.primal_action_trail.top();
            global_.primal_action_trail.pop();
            global_.primal_hmax_pcf[act_j] = old_pcf;
            global_.primal_pcf_hmax[act_j] = old_pcf_val;
        }
        global_.primal_action_trail_flags.clear();

        if (hmax_value >= best_hmax) {
            continue;
        }

        best_choice = act_i;
        best_hmax = hmax_value;
        found = true;
    }

    // Permanent commit for best_choice (keep updated pcf values, discard trails)
    global_.primal_used_actions.add(best_choice);
    std::vector<unsigned int> new_eff(inst_.actions[best_choice].eff_sparse.begin(), inst_.actions[best_choice].eff_sparse.end());
    std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
    primal_update_hmax_values_(new_eff);
    global_.primal_trail = std::stack<std::pair<unsigned int, double>>{};
    global_.primal_trail_flags.clear();
    global_.primal_action_trail = std::stack<std::tuple<unsigned int, int, double>>{};
    global_.primal_action_trail_flags.clear();

    return {found, best_choice};
}

inline void Solver::primal_update_hadd_values_(const std::vector<unsigned int>& new_facts) {
    for (const auto& fact : new_facts) {
        global_.primal_trail.emplace(fact, global_.primal_h_values[fact]);
        global_.primal_trail_flags.add(fact);
        global_.primal_h_values[fact] = 0;
        global_.primal_pq.push(fact, 0);
    }

    while (!global_.primal_pq.empty()) {
        const auto fact{global_.primal_pq.top()};
        global_.primal_pq.pop();

        for (const auto& act_i : global_.act_with_pre[fact]) {
            if (global_.primal_used_actions.contains(act_i)) {
                continue;
            }

            double cost_pre = 0;
            for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                cost_pre += global_.primal_h_values[pre];
            }

            if (cost_pre >= std::numeric_limits<double>::infinity()) {
                continue;
            }

            const double new_cost{cost_pre + inst_.actions[act_i].cost};
            for (const auto& p_eff : inst_.actions[act_i].eff_sparse) {
                if (new_cost >= global_.primal_h_values[p_eff]) {
                    continue;
                }

                if (!global_.primal_trail_flags[p_eff]) {
                    global_.primal_trail.emplace(p_eff, global_.primal_h_values[p_eff]);
                    global_.primal_trail_flags.add(p_eff);
                }
                global_.primal_h_values[p_eff] = new_cost;
                if (global_.primal_pq.has(p_eff)) {
                    global_.primal_pq.change(p_eff, new_cost);
                } else {
                    global_.primal_pq.push(p_eff, new_cost);
                }
            }
        }
    }
}

auto Solver::primal_greedychoice_hadd_(const std::list<unsigned int>& candidates, const BinarySet& state) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    double best_hadd = std::numeric_limits<double>::infinity();
    bool found = false;
    double current_hadd = 0;
    for (const auto& goal_fact : global_.goal_sparse) {
        current_hadd += global_.primal_h_values[goal_fact];
    }

    for (const auto& act_i : candidates) {
        if (global_.fixed_actions[act_i] || inst_.actions[act_i].cost == 0) {
            found = true;
            best_choice = act_i;
            break;
        }

        std::vector<unsigned int> new_eff(inst_.actions[act_i].eff_sparse.begin(), inst_.actions[act_i].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        primal_update_hadd_values_(new_eff);

        double hadd_value = current_hadd;

        // Restore fact values via trail; accumulate delta on goal facts for hadd_value
        while (!global_.primal_trail.empty()) {
            const auto [p, old_value] = global_.primal_trail.top();
            global_.primal_trail.pop();
            if (inst_.goal[p]) {
                hadd_value -= (old_value - global_.primal_h_values[p]);
            }
            global_.primal_h_values[p] = old_value;
        }
        global_.primal_trail_flags.clear();
        if (hadd_value >= best_hadd) {
            continue;
        }

        best_choice = act_i;
        best_hadd = hadd_value;
        found = true;
    }

    // Permanent commit for best_choice (keep updated values, discard trail)
    global_.primal_used_actions.add(best_choice);
    std::vector<unsigned int> new_eff(inst_.actions[best_choice].eff_sparse.begin(), inst_.actions[best_choice].eff_sparse.end());
    std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
    primal_update_hadd_values_(new_eff);
    global_.primal_trail = std::stack<std::pair<unsigned int, double>>{};
    global_.primal_trail_flags.clear();

    return {found, best_choice};
}
