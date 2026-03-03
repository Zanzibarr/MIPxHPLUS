#include "lmcut.hpp"

#include <deque>
#include <map>

#include "../external/pq.hxx"
#include "utils.hpp"

std::pair<int, double> lmcut::hmax_arbitrary(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                             [[maybe_unused]] const std::vector<double>& _) {
    int pcf{-1};
    double hmax{-1};
    for (const auto& p : preconditions) {
        if (hmax < hmax_values[p] - HPLUS_EPSILON) {
            hmax = hmax_values[p];
            pcf = p;
        }
    }
    return {pcf, hmax};
}

std::pair<int, double> lmcut::hmax_inverse(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                           [[maybe_unused]] const std::vector<double>& _) {
    int pcf{-1};
    double hmax{-1};
    for (const auto& p : preconditions) {
        if (hmax <= hmax_values[p] + HPLUS_EPSILON) {
            hmax = hmax_values[p];
            pcf = p;
        }
    }
    return {pcf, hmax};
}

std::pair<int, double> lmcut::hmax_value_decrease_minimization(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                                               const std::vector<double>& initial_hmax_values) {
    int pcf{-1}, count{0};
    double hmax{-1}, min_decrease{std::numeric_limits<double>::infinity()};
    for (const auto& p : preconditions) {
        if (hmax < hmax_values[p] - HPLUS_EPSILON) {
            hmax = hmax_values[p];
            pcf = p;
            min_decrease = initial_hmax_values[p] - hmax_values[p];
            count = 1;
        } else if (std::abs(hmax - hmax_values[p]) <
                   HPLUS_EPSILON) {  // First level tie-breaking: pcf with the smallest value decrease since the first hmax iteration
            if (initial_hmax_values[p] - hmax_values[p] < min_decrease - HPLUS_EPSILON) {
                pcf = p;
                min_decrease = initial_hmax_values[p] - hmax_values[p];
                count = 1;
            } else if (std::abs(initial_hmax_values[p] - hmax_values[p] - min_decrease) < HPLUS_EPSILON) {  // Second level tie-breaking: random
                count++;
                std::uniform_int_distribution<int> dist(1, count);
                if (dist(g_rng) == 1) pcf = p;
            }
        }
    }
    return {pcf, hmax};
}

std::pair<int, double> lmcut::hmax_random(const std::vector<unsigned int>& preconditions, const std::vector<double>& hmax_values,
                                          [[maybe_unused]] const std::vector<double>& _) {
    int pcf{-1}, count{0};
    double hmax{-1};
    for (const auto& p : preconditions) {
        if (hmax < hmax_values[p] - HPLUS_EPSILON) {
            hmax = hmax_values[p];
            pcf = p;
            count = 1;
        } else if (std::abs(hmax - hmax_values[p]) < HPLUS_EPSILON) {
            count++;
            std::uniform_int_distribution<int> dist(1, count);
            if (dist(g_rng) == 1) pcf = p;
        }
    }

    return {pcf, hmax};
}

void lmcut::init_hmax(const hplus::instance& inst, std::vector<double>& hmax_values, std::vector<int>& pcf, std::vector<double>& pcf_hmax,
                      std::vector<double>& reduced_costs, std::vector<unsigned int>& initial_actions) {
    for (unsigned int i = 0; i < inst.n; i++) hmax_values[i] = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < inst.m; i++) {
        pcf[i] = -1;
        pcf_hmax[i] = std::numeric_limits<double>::infinity();
        reduced_costs[i] = static_cast<double>(inst.actions[i].cost);
    }

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            initial_actions.push_back(act_i);
            pcf[act_i] = static_cast<int>(inst.n);
            pcf_hmax[act_i] = 0;
        }
    }
}

void update_hmax_values(const hplus::instance& inst, std::vector<double>& hmax_values, std::vector<int>& pcf, std::vector<double>& pcf_hmax,
                        const std::vector<double>& reduced_costs, const std::vector<unsigned int>& modified_actions,
                        const lmcut::hmax_function_type& hmax_function, const std::vector<double>& initial_hmax_values) {
    priority_queue<double> pq{inst.n};

    for (const auto& act_i : modified_actions) {
        const double new_cost{pcf_hmax[act_i] + reduced_costs[act_i]};
        for (const auto& p : inst.actions[act_i].eff_sparse) {
            if (new_cost >= hmax_values[p] - HPLUS_EPSILON) continue;

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
            if (pcf[act_i] != -1 && (pcf[act_i] != p || std::abs(hmax_values[p] - pcf_hmax[act_i])) < HPLUS_EPSILON) continue;

            const double old_hmax{pcf_hmax[act_i]};

            // Compute hmax and the pcf
            const auto& [act_pcf, act_hmax]{hmax_function(inst.actions[act_i].pre_sparse, hmax_values, initial_hmax_values)};

            // If this action has no pcf or it's infinite, skip
            if (act_pcf == -1 || act_hmax == std::numeric_limits<double>::infinity()) continue;

            // Update the pcf
            pcf[act_i] = act_pcf;
            pcf_hmax[act_i] = act_hmax;

            // If the hmax of this action hasnt changed, skip
            if (std::abs(pcf_hmax[act_i] - old_hmax) < HPLUS_EPSILON) continue;

            // Update all the action's effects and add them to the queue
            const double new_cost{act_hmax + reduced_costs[act_i]};
            for (const auto& eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= hmax_values[eff] + HPLUS_EPSILON) continue;

                hmax_values[eff] = new_cost;
                if (pq.has(eff))
                    pq.change(eff, new_cost);
                else
                    pq.push(eff, new_cost);
            }
        }
    }
}

double compute_and_store_cut(const hplus::instance& inst, const std::vector<double>& hmax_values, const std::vector<int>& pcf,
                             std::vector<double>& reduced_costs, const std::vector<unsigned int>& goal_sparse,
                             const std::vector<unsigned int>& initial_actions, const lmcut::hmax_function_type& hmax_function,
                             const std::vector<double>& initial_hmax_values, std::vector<std::vector<unsigned int>>& landmarks) {
    binary_set pre_goal_section(inst.n), goal_section(inst.n);
    std::deque<int> section_detect_queue;
    // Simulate a 0-cost action with precondition the goal state -> set its pcf as starting goal_section
    int goal_pcf{hmax_function(goal_sparse, hmax_values, initial_hmax_values).first};
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
            // non-initial check: I cannot add to the queue the source node, since no action could ever achieve it
            if (reduced_costs[act_i] <= HPLUS_EPSILON && pcf[act_i] != static_cast<int>(inst.n)) {
                if (!goal_section[pcf[act_i]]) {
                    goal_section.add(pcf[act_i]);
                    section_detect_queue.push_back(pcf[act_i]);
                }
            }
        }
    }

    // Compute the pre_goal section and the cut
    std::vector<unsigned int> cut;
    double min_redcost_cut{std::numeric_limits<double>::infinity()};
    explored.clear();

    const auto& check_cut = [&inst, &reduced_costs, &cut, &min_redcost_cut, &section_detect_queue, &pre_goal_section, &goal_section,
                             &explored](unsigned int act_i) {
        explored.add(act_i);
        bool added_to_cut{false};
        for (const auto& eff : inst.actions[act_i].eff_sparse) {
            if (goal_section[eff]) {
                if (reduced_costs[act_i] > HPLUS_EPSILON && !added_to_cut) {
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

    landmarks.push_back(std::move(cut));

    return min_redcost_cut;
}

void lmcut::compute_lmcut(const hplus::instance& inst, std::vector<double> hmax_values, std::vector<int> pcf, std::vector<double> pcf_hmax,
                          std::vector<double> reduced_costs, const std::vector<unsigned int>& goal_sparse,
                          const std::vector<unsigned int>& initial_actions, const hmax_function_type& hmax_function,
                          std::vector<std::vector<unsigned int>>& landmarks, bool print) {
    double lmcut_value{0};
    update_hmax_values(inst, hmax_values, pcf, pcf_hmax, reduced_costs, initial_actions, hmax_function, hmax_values);
    const std::vector<double> initial_hmax_values{hmax_values.begin(), hmax_values.end()};
    while (hmax_function(goal_sparse, hmax_values, initial_hmax_values).second > 0) {
        lmcut_value +=
            compute_and_store_cut(inst, hmax_values, pcf, reduced_costs, goal_sparse, initial_actions, hmax_function, initial_hmax_values, landmarks);
        update_hmax_values(inst, hmax_values, pcf, pcf_hmax, reduced_costs, landmarks[landmarks.size() - 1], hmax_function, initial_hmax_values);

        if (CHECK_STOP()) throw timelimit_exception("Reached time limit.");
    }
    if (print) LOG_INFO << "Computed a lm-cut value of: " << lmcut_value;
}

void prep::lmcut_landmarks_extraction(const hplus::execution& exec, hplus::instance& inst) {
    std::vector<int> pcf(inst.m);
    std::vector<double> hmax_values(inst.n), reduced_costs(inst.m), pcf_hmax(inst.m);
    const std::vector<unsigned int> goal_sparse{inst.goal.sparse()};
    std::vector<unsigned int> initial_actions;

    lmcut::init_hmax(inst, hmax_values, pcf, pcf_hmax, reduced_costs, initial_actions);

    std::map<char, lmcut::hmax_function_type> hmax_functions{
        {'a', lmcut::hmax_arbitrary}, {'i', lmcut::hmax_inverse}, {'v', lmcut::hmax_value_decrease_minimization}, {'r', lmcut::hmax_random}};

    for (auto x : exec.prep_lmcut)
        lmcut::compute_lmcut(inst, hmax_values, pcf, pcf_hmax, reduced_costs, goal_sparse, initial_actions, hmax_functions.at(x), inst.landmarks,
                             VERBOSE_BASIC());
}