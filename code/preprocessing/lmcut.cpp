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
    // LOG_DEBUG << "Debugging hmax function:";
    // LOG_DEBUG << "Preconditions:";
    // for (const auto& x : preconditions) std::cout << "(" << x << ";" << hmax_values[x] << ")";
    // std::cout << "\n";
    for (const auto& p : preconditions) {
        if (hmax < hmax_values[p]) {
            hmax = hmax_values[p];
            pcf = p;
        }
    }
    // LOG_DEBUG << "Returned: (" << pcf << ";" << hmax << ")";
    return {pcf, hmax};
}

void update_hmax_values(const hplus::instance& inst, std::vector<int>& hmax_values, std::vector<int>& pcf, std::vector<int>& pcf_hmax,
                        const std::vector<int>& reduced_costs, const std::vector<unsigned int>& modified_actions) {
    priority_queue<int> pq{inst.n};

    for (const auto& act_i : modified_actions) {
        const int new_cost{pcf_hmax[act_i] + reduced_costs[act_i]};
        for (const auto& p : inst.actions[act_i].eff_sparse) {
            if (new_cost >= hmax_values[p]) continue;

            ASSERT(new_cost < std::numeric_limits<int>::max());

            // std::cout << "(" << p << ";" << hmax_values[p] << "->" << new_cost << ")\n";

            hmax_values[p] = new_cost;
            if (pq.has(p))
                pq.change(p, new_cost);
            else
                pq.push(p, new_cost);
        }
    }

    // LOG_DEBUG << "\n" + std::string(pq);

    while (!pq.empty()) {
        const auto p{static_cast<int>(pq.top())};
        pq.pop();

        // LOG_DEBUG << "Var " << p << " with hmax " << hmax_values[p];

        for (const auto& act_i : inst.act_with_pre[p]) {
            // LOG_DEBUG << "--- Act " << act_i << " has " << p << " among the preconditions: reduced cost: " << reduced_costs[act_i]
            //           << "; current pcf: " << pcf[act_i] << " with hmax " << pcf_hmax[act_i]
            //           << " (check if it's updated: " << (pcf[act_i] >= 0 ? hmax_values[pcf[act_i]] : -1) << ")";

            // If this action's pcf is not p, than since we are lowering the hmax values, the hmax won't change for this action... skip
            // ... or if p is act_i's pcf but it's hmax is that same value, nothing would change... skip
            if (pcf[act_i] != -1 && (pcf[act_i] != p || hmax_values[p] == pcf_hmax[act_i])) continue;

            // LOG_DEBUG << "The pcf either is -1 or is " << p << " and its value isn't updated.";

            // LOG_DEBUG << "Preconditions of " << act_i << ":";
            // for (const auto& pre : inst.actions[act_i].pre_sparse) std::cout << "(" << pre << ";" << hmax_values[pre] << ")\n";

            // LOG_DEBUG << "Effects of " << act_i << ":";
            // for (const auto& eff : inst.actions[act_i].eff_sparse) std::cout << "(" << eff << ";" << hmax_values[eff] << ")\n";

            const int old_hmax{pcf_hmax[act_i]};

            // Compute hmax and the pcf
            const auto& [act_pcf, act_hmax]{hmax(inst.actions[act_i].pre_sparse, hmax_values)};

            // LOG_DEBUG << "Computed hmax: (" << act_pcf << ";" << act_hmax << ")";

            ASSERT(inst.actions[act_i].pre[act_pcf]);

            // If this action has no pcf or it's infinite, skip
            if (act_pcf == -1 || act_hmax == std::numeric_limits<int>::max()) continue;

            // LOG_DEBUG << "The hmax computed isn't infinite and it's an improvement to the previous";

            ASSERT(act_hmax <= pcf_hmax[act_i]);

            // Update the pcf
            pcf[act_i] = act_pcf;
            pcf_hmax[act_i] = act_hmax;

            // If the hmax of this action hasnt changed, skip
            if (pcf_hmax[act_i] == old_hmax) continue;  // FIXME: Removing this fixes the issue... why???

            // LOG_DEBUG << "Updating the effects:";

            // Update all the action's effects and add them to the queue
            const auto& new_cost{act_hmax + reduced_costs[act_i]};
            for (const auto& eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= hmax_values[eff]) continue;

                // std::cout << "(" << eff << ";" << hmax_values[eff] << "->" << new_cost << ")\n";

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

    // LOG_DEBUG << "\n\n\n\nDEBUGGING GOAL SECTION BUILDUP\n\n\n\n";
    // LOG_DEBUG << "Goal section:";
    // for (const auto& x : goal_section) std::cout << "(" << x << ";" << hmax_values[x] << ");";
    // std::cout << "\n";

    // Compute the goal section
    while (!section_detect_queue.empty()) {
        const auto p{section_detect_queue.front()};
        section_detect_queue.pop_front();

        // LOG_DEBUG << "Expanding on " << p;

        for (const auto& act_i : inst.act_with_eff[p]) {
            // LOG_DEBUG << "Action " << act_i << " preconditions:";
            // for (const auto& x : inst.actions[act_i].pre_sparse) std::cout << "(" << x << ";" << hmax_values[x] << ")";
            // std::cout << "\n";
            // LOG_DEBUG << "Action's pcf: " << pcf[act_i];

            // If I already explored this action or if it has no pcf, skip...
            if (explored[act_i] || pcf[act_i] == -1) continue;
            explored.add(act_i);

            // LOG_DEBUG << "Action's reduced cost: " << reduced_costs[act_i];
            // LOG_DEBUG << "Expanding...";

            // If it is a 0 reduced-cost (non-initial) action, than its pcf is also in the goal zone
            if (reduced_costs[act_i] == 0 && pcf[act_i] != static_cast<int>(inst.n)) {
                if (!goal_section[pcf[act_i]]) {
                    // LOG_DEBUG << "Adding " << pcf[act_i] << " to the goal section";
                    goal_section.add(pcf[act_i]);
                    section_detect_queue.push_back(pcf[act_i]);
                }
            }
            // mypause();
        }
        // LOG_DEBUG << "\n\n\n\n\n\n\n\n\n\n\n\n";
    }

    std::vector<unsigned int> cut;
    int min_redcost_cut{std::numeric_limits<int>::max()};

    explored.clear();

    const auto& check_cut = [&inst, &reduced_costs, &cut, &min_redcost_cut, &section_detect_queue, &pre_goal_section, &goal_section, &explored, &pcf,
                             &hmax_values](unsigned int act_i) {
        explored.add(act_i);
        // LOG_DEBUG << "Exploring action " << act_i << " with reduced cost " << reduced_costs[act_i];
        // LOG_DEBUG << "pcf: " << pcf[act_i] << " - " << ((pcf[act_i] >= 0 && pcf[act_i] != static_cast<int>(inst.n)) ? hmax_values[pcf[act_i]] :
        // -1); LOG_DEBUG << "Effects:"; for (const auto& eff : inst.actions[act_i].eff_sparse)
        //     std::cout << "(" << eff << ";" << hmax_values[eff] << "(" << (goal_section[eff] ? "G))" : "N))");
        // std::cout << "\n";
        bool added_to_cut{false};
        for (const auto& eff : inst.actions[act_i].eff_sparse) {
            if (goal_section[eff]) {
                if (reduced_costs[act_i] != 0 && !added_to_cut) {
                    // LOG_DEBUG << "Found " << eff << " in the goal with " << pcf[act_i] << " not in the goal.";
                    // LOG_DEBUG << "Adding " << act_i << " to the cut";
                    cut.push_back(act_i);
                    min_redcost_cut = std::min(min_redcost_cut, reduced_costs[act_i]);
                    added_to_cut = true;
                } else {
                    ASSERT(added_to_cut || inst.actions[act_i].pre_sparse.empty());
                }
            } else if (!pre_goal_section[eff]) {
                pre_goal_section.add(eff);
                section_detect_queue.push_back(eff);
            }
        }

        // LOG_DEBUG << "Pre goal section:";
        // for (const auto& x : pre_goal_section) std::cout << "(" << x << ";" << hmax_values[x] << ");";
        // std::cout << "\n";
        // mypause();
    };

    // LOG_DEBUG << "\n\n\n\nDEBUGGING PRE GOAL SECTION BUILDUP\n\n\n\n";
    // LOG_DEBUG << "Goal section:";
    // for (const auto& x : goal_section) std::cout << "(" << x << ";" << hmax_values[x] << ");";
    // std::cout << "\n";

    // // mypause();

    // LOG_DEBUG << "Initial actions:";
    // for (const auto& act_i : initial_actions) std::cout << "(" << act_i << ";" << reduced_costs[act_i] << ")";
    // std::cout << "\n";

    // Compute the pre_goal section
    for (const auto& act_i : initial_actions) check_cut(act_i);

    // LOG_DEBUG << "Size of queue: " << section_detect_queue.size();

    while (!section_detect_queue.empty()) {
        const auto p{section_detect_queue.front()};
        section_detect_queue.pop_front();

        // LOG_DEBUG << "!!! P: " << p << " - " << hmax_values[p];

        for (const auto& act_i : inst.act_with_pre[p]) {
            // LOG_DEBUG << "--- ACT: " << act_i << " ---";
            // const auto& [pcf_check, hmax_check]{hmax(inst.actions[act_i].pre_sparse, hmax_values)};
            // LOG_DEBUG << "pcf: " << pcf[act_i] << " (check: " << pcf_check << ")";
            // LOG_DEBUG << "hmax: " << hmax_values[pcf[act_i]] << " (check: " << hmax_check << ")";
            // LOG_DEBUG << "explored: " << explored[act_i];
            // ASSERT(pcf[act_i] == pcf_check);  // FIXME: THIS IS WHERE ITS FAILING
            // ASSERT(hmax_values[pcf[act_i]] == hmax_check);
            if (pcf[act_i] != p || explored[act_i] || pre_goal_section.contains(inst.actions[act_i].eff)) continue;
            check_cut(act_i);
        }
        // LOG_DEBUG << "Size of queue: " << section_detect_queue.size();
    }

    // LOG_DEBUG << "SEPARATOR\n\n\n\n\n";

    // LOG_DEBUG << "PREGOAL: " << std::string(pre_goal_section);
    // LOG_DEBUG << "GOAL:    " << std::string(goal_section);

    ASSERT(!pre_goal_section.intersects(goal_section));

    // for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
    //     // if (reduced_costs[act_i] == 0) continue;

    //     for (const auto& eff : inst.actions[act_i].eff_sparse) {
    //         if (goal_section[eff]) {
    //             if (pcf[act_i] == static_cast<int>(inst.n))
    //                 LOG_DEBUG << "Initial action " << act_i << " crossing the cut (x -> " << eff << ") with reduced cost " << reduced_costs[act_i];
    //             else if (pcf[act_i] >= 0 && pre_goal_section[pcf[act_i]])
    //                 LOG_DEBUG << "Action " << act_i << " crossing the cut (" << pcf[act_i] << " -> " << eff << " with reduced cost "
    //                           << reduced_costs[act_i];
    //         }
    //     }
    // }

    // LOG_DEBUG << "PRE GOAL SECTION: ";
    // for (const auto& x : pre_goal_section) std::cout << x << ";";
    // std::cout << "\n";

    // LOG_DEBUG << "GOAL SECTION: ";
    // for (const auto& x : goal_section) std::cout << x << ";";
    // std::cout << "\n";

    // for (const auto& act_i : initial_actions)
    //     if (reduced_costs[act_i] != 0) {
    //         LOG_DEBUG << "act: " << act_i << "; reduced cost: " << reduced_costs[act_i];
    //         for (const auto& eff : inst.actions[act_i].eff_sparse)
    //             if (goal_section[eff]) LOG_DEBUG << "This action has eff " << eff << " in the goal section.";
    //     }

    // LOG_DEBUG << "CHAOS:";
    // for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
    //     if (reduced_costs[act_i] == 0) continue;
    //     for (const auto& eff : inst.actions[act_i].eff_sparse) {
    //         if (pcf[act_i] < 0) continue;
    //         if (pcf[act_i] == static_cast<int>(inst.n))
    //             LOG_DEBUG << -1 << " -> " << eff;
    //         else
    //             LOG_DEBUG << pcf[act_i] << " -> " << eff;
    //     }
    // }

    ASSERT(!cut.empty());

    for (const auto& act_i : cut) reduced_costs[act_i] -= min_redcost_cut;

    inst.landmarks.push_back(std::move(cut));

    return min_redcost_cut;
}

void prep::lmcut_landmarks_extraction(hplus::instance& inst) {
    std::vector<int> hmax_values(inst.n, std::numeric_limits<int>::max()), pcf(inst.m, -1), pcf_hmax(inst.m, std::numeric_limits<int>::max());

    const std::vector<unsigned int> goal_sparse{inst.goal.sparse()};

    std::vector<int> reduced_costs(inst.m);
    for (unsigned int i = 0; i < inst.m; i++) reduced_costs[i] = static_cast<int>(inst.actions[i].cost);
    for (unsigned int i = 0; i < inst.m; i++) ASSERT(reduced_costs[i] >= 0);

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
    while (int val = hmax(goal_sparse, hmax_values).second > 0) {
        LOG_DEBUG << "goal hmax: " << val;
        lmcut_value += compute_cut(inst, hmax_values, pcf, reduced_costs, goal_sparse, initial_actions);
        update_hmax_values(inst, hmax_values, pcf, pcf_hmax, reduced_costs, inst.landmarks[inst.landmarks.size() - 1]);

        for (unsigned int i = 0; i < inst.m; i++) ASSERT(reduced_costs[i] >= 0);
        if (CHECK_STOP()) throw timelimit_exception("Reached time limit.");
    }

    LOG_INFO << "Computed a lm-cut value of: " << lmcut_value;
}