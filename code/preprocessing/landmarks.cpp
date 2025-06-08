#include <deque>  // std::deque

#include "preprocessing.hpp"

void prep::landmark_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks_ret) {
    // use the last bit as a flag to tell the binary_set is full
    std::vector<binary_set> landmarks(inst.n, binary_set{inst.n + 1});
    for (unsigned int p = 0; p < inst.n; p++) landmarks[p].add(inst.n);

    binary_set s_set{inst.n};

    // add to the queue all initial actions...
    std::deque<unsigned int> actions_queue;
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        // ... and since the initial state is empty, the initial actions are those with no preconditions (pre.empty() is O(n) while
        // pre_sparse.empty() is O(1))
        if (inst.actions[act_i].pre_sparse.empty()) actions_queue.push_back(act_i);
    }

    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(inst.n);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        for (const auto& p : inst.actions[act_i].pre_sparse) act_with_pre[p].push_back(act_i);
    }

    while (!actions_queue.empty()) {
        const hplus::action& a{inst.actions[actions_queue.front()]};
        actions_queue.pop_front();

        binary_set x_a{inst.n + 1};
        for (const auto& p : a.eff_sparse) x_a.add(p);
        for (const auto& p : a.eff_sparse) {
            s_set.add(p);

            binary_set x{x_a};
            for (const auto& q : a.pre_sparse) {
                // if variable p' has the "full" flag then the unification generates a "full" bitfield -> no need to unificate, just set the flag
                if (landmarks[q][inst.n]) {
                    x.add(inst.n);
                    // if x is now full we can exit, since all further unions won't change x
                    break;
                } else
                    x |= landmarks[q];
            }

            // we then check if L[p] != X, and if they are the same we skip... since X will be the intersection between L[p] and (the current) x,
            // if x is full, then X = L[P], so we can already skip
            if (x[inst.n]) continue;

            // if the set for variable p is the full set of variables, the intersection generates back x -> we can skip the intersection
            if (!landmarks[p][inst.n]) x &= landmarks[p];

            // we already know that x is not the full set now, so if the set for variable p is the full set, we know that x is not equal to the
            // set for variable p -> we can skip the check
            if (!landmarks[p][inst.n] && x == landmarks[p]) continue;

            landmarks[p] = x;
            for (const auto& act_i : act_with_pre[p]) {
                if (s_set.contains(inst.actions[act_i].pre) && std::find(actions_queue.begin(), actions_queue.end(), act_i) == actions_queue.end())
                    actions_queue.push_back(act_i);
            }
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

    // list of fact landmarks for each variable
    for (unsigned int p = 0; p < inst.n; p++) {
        for (unsigned int q = 0; q < inst.n; q++) {
            if (landmarks[p][q] || landmarks[p][inst.n]) landmarks_ret[p].push_back(q);
        }
    }

    // list of actions that have as effect variable p
    std::vector<std::vector<unsigned int>> act_with_eff(inst.n);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        for (const auto& p : inst.actions[act_i].eff_sparse) act_with_eff[p].push_back(act_i);
    }

    for (const auto& p : inst.goal) {
        for (const auto& q : landmarks_ret[p]) {
            if (inst.fixed_facts[q]) continue;
            inst.fixed_facts.add(q);
            unsigned int count{0}, cand_act;
            for (const auto& act_i : act_with_eff[q]) {
                cand_act = act_i;
                count++;
                if (count > 1) break;
            }
            if (count == 1) inst.fixed_actions.add(cand_act);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
}