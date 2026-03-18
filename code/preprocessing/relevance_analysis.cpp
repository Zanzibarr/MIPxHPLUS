#include "bs_utils.hpp"
#include "limits.hxx"
#include "preprocessing.hpp"

void prep::relevance_analysis_backward(hplus::instance& inst, BinarySet& relevant_variables) {
    BinarySet relevant_actions{inst.m};

    // compute first round of relevand variables and actions
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (bs_intersects(inst.goal, inst.actions[act_i].eff_sparse)) {
            relevant_variables |= inst.actions[act_i].pre_sparse;
            relevant_actions.add(act_i);
        }
    }
    if (CHECK_STOP()) {
        [[unlikely]] throw timelimit_exception("Reached time limit.");
    }

    // list of actions yet to check
    auto cand_actions_sparse{(!relevant_actions).sparse()};

    // keep looking for other relevant actions/variables until no more can be found
    bool new_act{true};
    while (new_act) {
        new_act = false;
        std::vector<unsigned int> new_relevant_actions;
        new_relevant_actions.reserve(inst.m);
        for (const auto& act_i : cand_actions_sparse) {
            if (!bs_intersects(relevant_variables, inst.actions[act_i].eff_sparse)) {
                continue;
            }

            relevant_actions.add(act_i);
            relevant_variables |= inst.actions[act_i].pre_sparse;
            new_relevant_actions.push_back(act_i);
            new_act = true;
        }
        const auto it{std::set_difference(cand_actions_sparse.begin(), cand_actions_sparse.end(), new_relevant_actions.begin(),
                                          new_relevant_actions.end(), cand_actions_sparse.begin())};
        cand_actions_sparse.resize(it - cand_actions_sparse.begin());
        if (CHECK_STOP()) {
            [[unlikely]] throw timelimit_exception("Reached time limit.");
        }
    }
    relevant_variables |= inst.goal;

    // eliminate actions and variables that are not relevant (or landmarks)
    inst.eliminated_facts |= !(relevant_variables | inst.fixed_facts);  // eliminating variables will be done at once, later
    inst.eliminated_actions |= !relevant_actions;                       // eliminating actions will be done at once, later
}

void prep::relevance_analysis_forward(hplus::instance& inst, BinarySet& relevant_variables) {
    std::vector<unsigned int> rem_act = ((!inst.eliminated_actions) & (!inst.fixed_actions)).sparse();

    for (const auto& act_i : rem_act) {
        if (!bs_intersects(relevant_variables, inst.actions[act_i].eff_sparse)) {
            inst.eliminated_actions.add(act_i);
        }
    }
}