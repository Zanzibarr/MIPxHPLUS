#include <binary_set.hxx>

#include "bs_utils.hpp"
#include "solver.hpp"

auto Solver::prep_dominated_actions_(std::vector<std::vector<unsigned int>>& landmarks) -> bool {
    logger_[DEBUG] << "PREP: dominated actions extraction";

    const auto& rem_act{(!global_.eliminated_actions).sparse()};

    myassert(global_.eliminated_facts.size() < inst_.n, "Every fact is marked for elimination");
    std::vector<unsigned int> local_idx(inst_.n, 0);
    unsigned int n_local{0};
    for (const auto fact : !global_.eliminated_facts) {
        local_idx[fact] = n_local++;
    }

    std::vector<BinarySet> act_flm(inst_.m, BinarySet(n_local));
    for (const auto& act_i : rem_act) {
        for (const auto& var_i : inst_.actions[act_i].pre_sparse) {
            if (global_.eliminated_facts[var_i]) {
                continue;
            }
            for (const auto& fact : landmarks[var_i]) {
                if (global_.eliminated_facts[fact]) {
                    continue;
                }
                act_flm[act_i].add(local_idx[fact]);
            }
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing dominated actions removal (first loop)", EarlyExit::TIMELIMIT);
        }
    }

    bs_searcher candidates{n_local};
    std::vector<BinarySet> actions_effects(inst_.m, BinarySet(n_local));
    for (const auto& act_i : rem_act) {
        for (const auto& val : inst_.actions[act_i].eff_sparse) {
            if (global_.eliminated_facts[val]) {
                continue;
            }
            actions_effects[act_i].add(local_idx[val]);
        }
        if (global_.fixed_actions[act_i]) {
            continue;
        }
        candidates.add(act_i, actions_effects[act_i]);
    }

    // pre(dominant) is a subset of the fact landmarks of the dominated action, restricted to the surviving facts
    const auto flm_covers_pre = [&](unsigned int dominated_act, unsigned int dominant_act) -> bool {
        for (const auto& pre : inst_.actions[dominant_act].pre_sparse) {
            if (global_.eliminated_facts[pre]) {
                continue;
            }
            if (!act_flm[dominated_act][local_idx[pre]]) {
                return false;
            }
        }
        return true;
    };

    // rem_act excludes everything marked before this step, so within it "already marked" and "dominated here" are the same thing: eliminated_actions
    // doubles as the record of what this loop found, and only the count has to be tracked separately
    unsigned int dominated{0};

    for (const auto& dominant_act : rem_act) {
        if (global_.eliminated_actions[dominant_act]) {
            continue;
        }

        for (const auto& dominated_act : candidates.find_subsets(actions_effects[dominant_act])) {
            if (dominant_act == dominated_act || inst_.actions[dominant_act].cost > inst_.actions[dominated_act].cost ||
                !flm_covers_pre(dominated_act, dominant_act)) {
                continue;
            }

            global_.eliminated_actions.add(dominated_act);
            candidates.remove(dominated_act, actions_effects[dominated_act]);
            dominated++;
        }

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing dominated actions removal (main loop)", EarlyExit::TIMELIMIT);
        }
    }
    return dominated > 0;
}
