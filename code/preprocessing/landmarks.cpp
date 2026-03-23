#include <deque>

#include "bs_utils.hpp"
#include "limits.hxx"
#include "preprocessing.hpp"

void prep::landmark_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks_ret) {
    // use the last bit as a flag to tell the BinarySet is full
    std::vector<BinarySet> landmarks(inst.n, BinarySet{inst.n + 1});
    for (unsigned int fact = 0; fact < inst.n; fact++) {
        landmarks[fact].add(inst.n);
    }

    BinarySet s_set{inst.n};

    // add to the queue all initial actions...
    std::deque<unsigned int> actions_queue;
    std::unordered_set<unsigned int> acts_in_queue;
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        // ... and since the initial state is empty, the initial actions are those with no preconditions (pre.empty() is O(n) while
        // pre_sparse.empty() is O(1))
        if (inst.actions[act_i].pre_sparse.empty()) {
            actions_queue.push_back(act_i);
            acts_in_queue.insert(act_i);
        }
    }

    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(inst.n);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        for (const auto& pre : inst.actions[act_i].pre_sparse) {
            act_with_pre[pre].push_back(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const auto act_i = actions_queue.front();
        const hplus::action& act{inst.actions[act_i]};
        actions_queue.pop_front();
        acts_in_queue.erase(act_i);

        BinarySet x_a{inst.n + 1};
        for (const auto& eff : act.eff_sparse) {
            x_a.add(eff);
        }
        for (const auto& eff : act.eff_sparse) {
            s_set.add(eff);

            BinarySet x{x_a};
            for (const auto& pre : act.pre_sparse) {
                // if variable eff' has the "full" flag then the unification generates a "full" bitfield -> no need to unificate, just set the flag
                if (landmarks[pre][inst.n]) {
                    x.add(inst.n);
                    // if x is now full we can exit, since all further unions won't change x
                    break;
                }
                x |= landmarks[pre];
            }

            // we then check if L[eff] != X, and if they are the same we skip... since X will be the intersection between L[eff] and (the current) x,
            // if x is full, then X = L[P], so we can already skip
            if (x[inst.n]) {
                continue;
            }

            // if the set for variable eff is the full set of variables, the intersection generates back x -> we can skip the intersection
            if (!landmarks[eff][inst.n]) {
                x &= landmarks[eff];
            }

            // we already know that x is not the full set now, so if the set for variable eff is the full set, we know that x is not equal to the
            // set for variable eff -> we can skip the check
            if (!landmarks[eff][inst.n] && x == landmarks[eff]) {
                continue;
            }

            landmarks[eff] = x;
            for (const auto& act_j : act_with_pre[eff]) {
                if (bs_contains(s_set, inst.actions[act_j].pre_sparse) && !acts_in_queue.contains(act_j)) {
                    actions_queue.push_back(act_j);
                }
            }
        }
        if (CHECK_STOP()) {
            [[unlikely]] throw timelimit_exception("Reached time limit.");
        }
    }

    // list of fact landmarks for each variable
    for (unsigned int fact_p = 0; fact_p < inst.n; fact_p++) {
        for (unsigned int fact_q = 0; fact_q < inst.n; fact_q++) {
            if (landmarks[fact_p][fact_q] || landmarks[fact_p][inst.n]) {
                landmarks_ret[fact_p].push_back(fact_q);
            }
        }
    }

    // list of actions that have as effect variable p
    std::vector<std::vector<unsigned int>> act_with_eff(inst.n);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        for (const auto& eff : inst.actions[act_i].eff_sparse) {
            act_with_eff[eff].push_back(act_i);
        }
    }

    for (const auto& g_fact : inst.goal) {
        for (const auto& g_factlm : landmarks_ret[g_fact]) {
            if (inst.fixed_facts[g_factlm]) {
                continue;
            }
            inst.fixed_facts.add(g_factlm);
            unsigned int count{0};
            unsigned int cand_act = 0;
            for (const auto& act_i : act_with_eff[g_factlm]) {
                cand_act = act_i;
                count++;
                if (count > 1) {
                    break;
                }
            }
            if (count == 1) {
                inst.fixed_actions.add(cand_act);
            }
        }
        if (CHECK_STOP()) {
            [[unlikely]] throw timelimit_exception("Reached time limit.");
        }
    }
}