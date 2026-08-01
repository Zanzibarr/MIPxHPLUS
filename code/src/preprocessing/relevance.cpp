#include <binary_set.hxx>
#include <queue>

#include "bs_utils.hpp"
#include "cli_descriptions.hpp"
#include "solver.hpp"

void Solver::prep_relevance_backward_(BinarySet& relevant_variables) {
    logger_[DEBUG] << "PREP: backwards relevance analysis";

    BinarySet relevant_actions(inst_.m);
    std::queue<unsigned int> relevant_facts_queue;

    // compute first round of relevand variables and actions
    for (const auto& g_fact : inst_.goal) {
        relevant_variables.add(g_fact);
        relevant_facts_queue.push(g_fact);
    }

    std::vector<std::vector<unsigned int>> act_with_eff(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto eff : inst_.actions[act_i].eff_sparse) {
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
            for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                // relevant_variables is empty at the beginning of this function, so this is equivalent to a check of "pre" already being in the
                // queue
                if (relevant_variables[pre]) {
                    continue;
                }
                relevant_variables.add(pre);
                relevant_facts_queue.push(pre);
            }
        }

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing backwards relevance analysis", EarlyExit::TIMELIMIT);
        }
    }

    // eliminate actions and variables that are not relevant (or landmarks)
    global_.eliminated_facts |= !(relevant_variables | global_.fixed_facts);  // eliminating variables will be done at once, later
    global_.eliminated_actions |= !relevant_actions;                          // eliminating actions will be done at once, later

    // Eliminate non-goal facts that aren't precondition of any action
    // list of actions that have as precondition variable p
    if (params_.get<cli_desc::preprocess, std::string>().find('p') != std::string::npos) {
        std::vector<std::vector<unsigned int>> act_with_pre(inst_.n);
        for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
            if (global_.eliminated_actions[act_i]) {
                continue;
            }
            for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                if (global_.eliminated_facts[pre]) {
                    continue;
                }
                act_with_pre[pre].push_back(act_i);
            }
        }

        for (unsigned int i = 0; i < inst_.n; i++) {
            if (!inst_.goal[i] && !global_.eliminated_facts[i] && act_with_pre[i].empty()) {
                global_.eliminated_facts.add(i);
            }
        }
    }
}

void Solver::prep_relevance_forward_(BinarySet& relevant_variables) {
    logger_[DEBUG] << "PREP: forward relevance analysis";

    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (global_.eliminated_actions[act_i] || global_.fixed_actions[act_i]) {
            continue;
        }
        // Instead of using (relevant | fixed) to check if an action should be eliminated or not, we just use relevant since we might have fixed facts
        // that are not relevant... moreover, using only relevant we have a smaller set with which an actions effects needs to intersect with to not
        // be eliminated -> we eliminate more actions
        if (!bs_intersects(relevant_variables, inst_.actions[act_i].eff_sparse)) {
            global_.eliminated_actions.add(act_i);
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing forward relevance analysis", EarlyExit::TIMELIMIT);
        }
    }
}
