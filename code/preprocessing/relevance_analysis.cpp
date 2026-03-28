#include <queue>

#include "bs_utils.hpp"
#include "limits.hxx"
#include "preprocessing.hpp"

void prep::relevance_analysis_backward(hplus::instance& inst, BinarySet& relevant_variables) {
    BinarySet relevant_actions(inst.m);
    std::queue<unsigned int> relevant_facts_queue;

    // compute first round of relevand variables and actions
    for (const auto& g_fact : inst.goal) {
        relevant_variables.add(g_fact);
        relevant_facts_queue.push(g_fact);
    }
    if (CHECK_STOP()) {
        [[unlikely]] throw timelimit_exception("Reached time limit.");
    }

    std::vector<std::vector<unsigned int>> act_with_eff(inst.n);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        for (const auto eff : inst.actions[act_i].eff_sparse) {
            act_with_eff[eff].push_back(act_i);
        }
    }

    while (!relevant_facts_queue.empty()) {
        const auto fact = relevant_facts_queue.front();
        relevant_facts_queue.pop();

        for (const auto& act_i : act_with_eff[fact]) {
            if (relevant_actions[act_i]) {
                continue;
            }
            relevant_actions.add(act_i);
            for (const auto& pre : inst.actions[act_i].pre_sparse) {
                // relevant_variables is empty at the beginning of this function, so this is equivalent to a check of "pre" already being in the queue
                if (relevant_variables[pre]) {
                    continue;
                }
                relevant_variables.add(pre);
                relevant_facts_queue.push(pre);
            }
        }
    }

    // eliminate actions and variables that are not relevant (or landmarks)
    inst.eliminated_facts |= !(relevant_variables | inst.fixed_facts);  // eliminating variables will be done at once, later
    inst.eliminated_actions |= !relevant_actions;                       // eliminating actions will be done at once, later
}

void prep::relevance_analysis_forward(hplus::instance& inst, BinarySet& relevant_variables) {
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (inst.eliminated_actions[act_i] || inst.fixed_actions[act_i]) {
            continue;
        }
        if (!bs_intersects(relevant_variables, inst.actions[act_i].eff_sparse)) {
            inst.eliminated_actions.add(act_i);
        }
    }
}