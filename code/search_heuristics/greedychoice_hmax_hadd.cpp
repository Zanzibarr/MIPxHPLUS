#include <vector>

#include "heuristic.hpp"

[[nodiscard]]
static auto htype(const std::vector<unsigned int>& state, const std::vector<double>& values, double (*h_eqtype)(double, double)) -> double {
    double hvalue{0};
    for (const auto& fact : state) {
        hvalue = h_eqtype(hvalue, values[fact]);
    }
    return hvalue;
}

static void update_htype_values(const hplus::instance& inst, const std::vector<unsigned int>& new_facts, std::vector<double>& values,
                                priority_queue<double>& pq, std::stack<std::pair<unsigned int, double>>& trail, const binary_set& used_actions,
                                double (*h_eqtype)(double, double)) {
    binary_set trail_flags{inst.n};
    for (const auto& fact : new_facts) {
        trail.emplace(fact, values[fact]);
        trail_flags.add(fact);
        values[fact] = 0;
        pq.push(fact, 0);
    }

    while (!pq.empty()) {
        const auto& fact{pq.top()};
        pq.pop();

        for (const auto& act_i : inst.act_with_pre[fact]) {
            if (used_actions.contains(act_i)) {
                continue;
            }

            const double cost_pre{htype(inst.actions[act_i].pre_sparse, values, h_eqtype)};

            if (cost_pre >= std::numeric_limits<double>::infinity()) {
                continue;
            }

            const double new_cost{cost_pre + inst.actions[act_i].cost};
            for (const auto& p_eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= values[p_eff]) {
                    continue;
                }

                if (!trail_flags[p_eff]) {
                    trail.emplace(p_eff, values[p_eff]);
                    trail_flags.add(p_eff);
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
    std::stack<std::pair<unsigned int, double>> _{};
    update_htype_values(inst, std::vector<unsigned int>(), values, pq, _, binary_set(inst.m), h_eqtype);
}

[[nodiscard]]
auto heur::greedy_choice_hmax(const hplus::instance& inst, const std::list<unsigned int>& candidates, const binary_set& state,
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
        update_htype_values(inst, new_eff, userhandle.values, userhandle.pq, userhandle.trail, userhandle.used_actions, hmax);

        double hmax_value = htype(userhandle.goal_sparse, userhandle.values, hmax);

        while (!userhandle.trail.empty()) {
            auto [p, old_value] = userhandle.trail.top();
            userhandle.trail.pop();
            userhandle.values[p] = old_value;
        }

        if (hmax_value >= best_hmax) {
            continue;
        }

        best_choice = act_i;
        best_hmax = hmax_value;
        found = true;
    }

    std::stack<std::pair<unsigned int, double>> trail;
    userhandle.used_actions.add(best_choice);
    std::vector<unsigned int> new_eff(inst.actions[best_choice].eff_sparse.begin(), inst.actions[best_choice].eff_sparse.end());
    std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
    update_htype_values(inst, new_eff, userhandle.values, userhandle.pq, trail, userhandle.used_actions, hmax);

    return {found, best_choice};
}

[[nodiscard]]
auto heur::greedy_choice_hadd(const hplus::instance& inst, const std::list<unsigned int>& candidates, const binary_set& state,
                              heur::greedychoice_userhandle& userhandle) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    double best_hadd = std::numeric_limits<double>::infinity();
    bool found = false;
    double current_hadd = htype(userhandle.goal_sparse, userhandle.values, hadd);

    for (const auto& act_i : candidates) {
        if (inst.fixed_actions[act_i] || inst.actions[act_i].cost == 0) {
            found = true;
            best_choice = act_i;
            break;
        }

        std::vector<unsigned int> new_eff(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        update_htype_values(inst, new_eff, userhandle.values, userhandle.pq, userhandle.trail, userhandle.used_actions, hadd);

        double hadd_value = current_hadd;

        while (!userhandle.trail.empty()) {
            const auto [p, old_value] = userhandle.trail.top();
            userhandle.trail.pop();
            if (inst.goal[p]) {
                hadd_value -= (old_value - userhandle.values[p]);
            }
            userhandle.values[p] = old_value;
        }

        if (hadd_value >= best_hadd) {
            continue;
        }

        best_choice = act_i;
        best_hadd = hadd_value;
        found = true;
    }

    std::stack<std::pair<unsigned int, double>> trail;
    userhandle.used_actions.add(best_choice);
    std::vector<unsigned int> new_eff(inst.actions[best_choice].eff_sparse.begin(), inst.actions[best_choice].eff_sparse.end());
    std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
    update_htype_values(inst, new_eff, userhandle.values, userhandle.pq, trail, userhandle.used_actions, hadd);

    return {found, best_choice};
}