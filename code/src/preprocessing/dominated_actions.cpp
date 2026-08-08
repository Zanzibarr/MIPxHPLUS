#include <binary_set.hxx>

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

    std::vector<std::vector<unsigned int>> act_with_eff = std::vector<std::vector<unsigned int>>(n_local);
    std::vector<BinarySet> actions_effects_bs(inst_.m, BinarySet(n_local));
    for (unsigned int act_i = 0; act_i < inst_.m; ++act_i) {
        if (global_.eliminated_actions[act_i]) {
            continue;
        }
        for (const auto& eff : inst_.actions[act_i].eff_sparse) {
            if (global_.eliminated_facts[eff]) {
                continue;
            }
            act_with_eff[local_idx[eff]].push_back(act_i);
            actions_effects_bs[act_i].add(local_idx[eff]);
        }
    }

    // pre(dominant) is a subset of the fact landmarks of the dominated action, restricted to the surviving facts
    const auto flm_covers_pre = [&](unsigned int dominated_act, unsigned int dominant_act) -> bool {
        for (const auto& pre : inst_.actions[dominant_act].pre_sparse) {
            if (!global_.eliminated_facts[pre] && !act_flm[dominated_act][local_idx[pre]]) {
                return false;
            }
        }
        return true;
    };

    const auto eff_contains_wlim = [&](unsigned int dominant_act, unsigned int dominated_act) -> bool {
        for (const auto& eff : inst_.actions[dominated_act].eff_sparse) {
            if (!global_.eliminated_facts[eff] && !actions_effects_bs[dominant_act][local_idx[eff]]) {
                return false;
            }
        }
        return true;
    };

    // rem_act excludes everything marked before this step, so within it "already marked" and "dominated here" are the same thing:
    // eliminated_actions doubles as the record of what this loop found, and only the count has to be tracked separately
    unsigned int dominated_count{0};

    // Reversing the loop, iterating through possible dominated actions, looking for a dominant one
    for (const auto dominated_act : rem_act) {
        if (global_.fixed_actions[dominated_act]) {
            continue;
        }

        unsigned int best_fact{inst_.n};  // sentinel: "not found"
        unsigned int best_count{inst_.m + 1};

        for (const auto& eff : inst_.actions[dominated_act].eff_sparse) {
            if (global_.eliminated_facts[eff]) {
                continue;
            }
            const auto count = act_with_eff[local_idx[eff]].size();
            if (count < best_count) {
                best_count = count;
                best_fact = eff;
            }
        }

        // If dominated_act has no effect, it can be eliminated by relevance
        if (best_fact == inst_.n) {
            global_.eliminated_actions.add(dominated_act);
            dominated_count++;
            continue;
        }

        for (const auto& dominant_act : act_with_eff[local_idx[best_fact]]) {
            // act_with_eff/rem_act are snapshots from before this loop started, so actions eliminated by earlier
            // iterations of this same pass can still show up here and must be skipped explicitly
            if (dominant_act == dominated_act || global_.eliminated_actions[dominant_act]) {
                continue;
            }
            if (!eff_contains_wlim(dominant_act, dominated_act)) {
                continue;
            }
            if (!flm_covers_pre(dominated_act, dominant_act)) {
                continue;
            }
            if (inst_.actions[dominant_act].cost > inst_.actions[dominated_act].cost) {
                continue;
            }

            global_.eliminated_actions.add(dominated_act);
            dominated_count++;
            break;
        }
    }

    return dominated_count > 0;
}