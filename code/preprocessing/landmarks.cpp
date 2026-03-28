#include <deque>

#include "limits.hxx"
#include "preprocessing.hpp"

void prep::landmark_extraction(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks_ret) {
    // use the last bit as a flag to tell the BinarySet is full
    std::vector<BinarySet> landmarks(inst.n, BinarySet{inst.n + 1});
    for (unsigned int fact = 0; fact < inst.n; fact++) {
        landmarks[fact].add(inst.n);
    }

    BinarySet s_set{inst.n};

    // add to the queue all initial actions (those with no preconditions)
    std::deque<unsigned int> actions_queue;
    BinarySet acts_in_queue{inst.m};
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            actions_queue.push_back(act_i);
            acts_in_queue.add(act_i);
        }
    }

    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(inst.n);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        for (const auto& pre : inst.actions[act_i].pre_sparse) {
            act_with_pre[pre].push_back(act_i);
        }
    }

    // Watch preconditions for O(1) applicability checks against s_set.
    // watch_s[act] = one precondition of act not yet in s_set (inst.n = sentinel: action is applicable)
    // watching_s[fact] = actions that are currently watching 'fact'
    // applicable[act] = true once all preconditions of act are in s_set
    std::vector<unsigned int> watch_s(inst.m, inst.n);
    std::vector<std::vector<unsigned int>> watching_s(inst.n);
    BinarySet applicable{inst.m};
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            applicable.add(act_i);
        } else {
            // s_set is empty at init — first precondition is always unsatisfied
            const unsigned int watched = inst.actions[act_i].pre_sparse[0];
            watch_s[act_i] = watched;
            watching_s[watched].push_back(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const auto act_i = actions_queue.front();
        const auto& act = inst.actions[act_i];
        actions_queue.pop_front();
        acts_in_queue.remove(act_i);

        BinarySet x_a{inst.n + 1};
        for (const auto& eff : act.eff_sparse) {
            x_a.add(eff);
        }

        // Compute the union of precondition landmarks once for all effects of this action.
        BinarySet x_union{x_a};
        bool x_union_full = false;
        for (const auto& pre : act.pre_sparse) {
            // if variable pre has the "full" flag then the union generates a "full" bitfield
            if (landmarks[pre][inst.n]) {
                x_union_full = true;
                break;
            }
            x_union |= landmarks[pre];
        }

        for (const auto& eff : act.eff_sparse) {
            s_set.add(eff);

            // Update watch preconditions for actions watching this newly reached fact.
            // Swap-erase pattern: we always remove act_j from watching_s[eff] since eff is
            // now in s_set and should never be watched again.
            for (size_t wi = 0; wi < watching_s[eff].size();) {
                const unsigned int act_j = watching_s[eff][wi];
                bool moved = false;
                for (const auto& pre : inst.actions[act_j].pre_sparse) {
                    if (!s_set[pre]) {
                        watch_s[act_j] = pre;
                        watching_s[pre].push_back(act_j);
                        moved = true;
                        break;
                    }
                }
                watching_s[eff][wi] = watching_s[eff].back();
                watching_s[eff].pop_back();
                if (!moved) {
                    applicable.add(act_j);
                    watch_s[act_j] = inst.n;  // sentinel: no unsatisfied precondition
                }
            }

            // if x_union is already full, intersection with landmarks[eff] cannot shrink it —
            // no useful landmark update is possible for this effect
            if (x_union_full) {
                continue;
            }

            BinarySet lm_new{x_union};

            // if the set for variable eff is the full set of variables, the intersection
            // generates back lm_new -> we can skip the intersection
            if (!landmarks[eff][inst.n]) {
                lm_new &= landmarks[eff];
            }

            // we already know that lm_new is not the full set now, so if the set for variable
            // eff is the full set, we know that lm_new is not equal to the set for variable eff
            if (!landmarks[eff][inst.n] && lm_new == landmarks[eff]) {
                continue;
            }

            landmarks[eff] = lm_new;
            for (const auto& act_j : act_with_pre[eff]) {
                if (applicable[act_j] && !acts_in_queue[act_j]) {
                    actions_queue.push_back(act_j);
                    acts_in_queue.add(act_j);
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

    for (const auto g_fact : inst.goal) {
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
