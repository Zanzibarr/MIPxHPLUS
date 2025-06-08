#include "preprocessing.hpp"

void prep::dominated_actions_extraction(hplus::instance& inst, const std::vector<std::vector<unsigned int>>& landmarks) {
    const auto& rem_act{(!inst.eliminated_actions).sparse()};

    std::vector<binary_set> act_flm(inst.m, binary_set(inst.n));

    // compute the landmarks for each action remaining
    for (const auto& act_i : rem_act) {
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            for (const auto& i : landmarks[var_i]) act_flm[act_i].add(i);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

    bs_searcher candidates{inst.n};
    for (const auto& act_i : rem_act) {
        if (inst.fixed_actions[act_i]) continue;
        candidates.add(act_i, inst.actions[act_i].eff);
    }

    binary_set dominated_actions{inst.m};

    // find all dominated actions and eliminate them
    for (const auto& dominant_act : rem_act) {
        if (dominated_actions[dominant_act]) continue;

        for (const auto& dominated_act : candidates.find_subsets(inst.actions[dominant_act].eff)) {
            if (dominant_act == dominated_act || inst.actions[dominant_act].cost > inst.actions[dominated_act].cost ||
                !act_flm[dominated_act].contains(inst.actions[dominant_act].pre)) [[likely]]
                continue;

            dominated_actions.add(dominated_act);
            inst.eliminated_actions.add(dominated_act);
            candidates.remove(dominated_act, inst.actions[dominated_act].eff);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
}