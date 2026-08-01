#include <binary_set.hxx>
#include <deque>
#include <utility>  // std::move

#include "solver.hpp"

auto Solver::prep_fact_landmarks_(std::vector<std::vector<unsigned int>>& landmarks_ret) -> bool {
    logger_[DEBUG] << "PREP: fact landmarks extraction";

    // use the last bit as a flag to tell the BinarySet is full
    std::vector<BinarySet> landmarks(inst_.n, BinarySet{inst_.n + 1});
    for (unsigned int fact = 0; fact < inst_.n; fact++) {
        landmarks[fact].add(inst_.n);
    }

    BinarySet s_set{inst_.n};

    // add to the queue all initial actions (those with no preconditions)
    std::deque<unsigned int> actions_queue;
    BinarySet acts_in_queue{inst_.m};
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (inst_.actions[act_i].pre_sparse.empty()) {
            actions_queue.push_back(act_i);
            acts_in_queue.add(act_i);
        }
    }

    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto& pre : inst_.actions[act_i].pre_sparse) {
            act_with_pre[pre].push_back(act_i);
        }
    }

    // Watch preconditions for O(1) applicability checks against s_set.
    // watch_s[act] = one precondition of act not yet in s_set (inst_.n = sentinel: action is applicable)
    // watching_s[fact] = actions that are currently watching 'fact'
    // applicable[act] = true once all preconditions of act are in s_set
    std::vector<unsigned int> watch_s(inst_.m, inst_.n);
    std::vector<std::vector<unsigned int>> watching_s(inst_.n);
    BinarySet applicable{inst_.m};
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (inst_.actions[act_i].pre_sparse.empty()) {
            applicable.add(act_i);
        } else {
            // s_set is empty at init — first precondition is always unsatisfied
            const unsigned int watched = inst_.actions[act_i].pre_sparse[0];
            watch_s[act_i] = watched;
            watching_s[watched].push_back(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const auto act_i = actions_queue.front();
        const auto& act = inst_.actions[act_i];
        actions_queue.pop_front();
        acts_in_queue.remove(act_i);

        BinarySet x_a{inst_.n + 1};
        for (const auto& eff : act.eff_sparse) {
            x_a.add(eff);
        }

        // Compute the union of precondition landmarks once for all effects of this action.
        BinarySet x_union{x_a};
        bool x_union_full = false;
        for (const auto& pre : act.pre_sparse) {
            // if variable pre has the "full" flag then the union generates a "full" bitfield
            if (landmarks[pre][inst_.n]) {
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
                for (const auto& pre : inst_.actions[act_j].pre_sparse) {
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
                    watch_s[act_j] = inst_.n;  // sentinel: no unsatisfied precondition
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
            if (!landmarks[eff][inst_.n]) {
                lm_new &= landmarks[eff];
            }

            // we already know that lm_new is not the full set now, so if the set for variable
            // eff is the full set, we know that lm_new is not equal to the set for variable eff
            if (!landmarks[eff][inst_.n] && lm_new == landmarks[eff]) {
                continue;
            }

            landmarks[eff] = std::move(lm_new);
            for (const auto& act_j : act_with_pre[eff]) {
                if (applicable[act_j] && !acts_in_queue[act_j]) {
                    actions_queue.push_back(act_j);
                    acts_in_queue.add(act_j);
                }
            }
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing landmark extraction (main loop)", EarlyExit::TIMELIMIT);
        }
    }

    // list of fact landmarks for each variable
    std::vector<std::vector<unsigned int>> new_landmarks(inst_.n);
    for (unsigned int fact_p = 0; fact_p < inst_.n; fact_p++) {
        for (unsigned int fact_q = 0; fact_q < inst_.n; fact_q++) {
            if (landmarks[fact_p][fact_q] || landmarks[fact_p][inst_.n]) {
                new_landmarks[fact_p].push_back(fact_q);
            }
        }
    }

    // list of actions that have as effect variable p
    std::vector<std::vector<unsigned int>> act_with_eff(inst_.n);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto& eff : inst_.actions[act_i].eff_sparse) {
            act_with_eff[eff].push_back(act_i);
        }
    }

    // Accumulated across runs: a fixed fact/action is a deduction that stays valid (landmark sets only grow, achiever sets only shrink), and both
    // sets are remapped by the elimination steps. Clearing them here would also drop what other steps derived, e.g. the actions fixed by 'o'
    BinarySet visited_lms(inst_.n);
    for (const auto g_fact : inst_.goal) {
        for (const auto& g_factlm : new_landmarks[g_fact]) {
            // Dedup within this run only: fixed_facts may already hold facts derived elsewhere, whose unique achiever was never looked for
            if (visited_lms[g_factlm]) {
                continue;
            }
            visited_lms.add(g_factlm);
            global_.fixed_facts.add(g_factlm);
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
                global_.fixed_actions.add(cand_act);
            }
        }
        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing landmark extraction (goal loop)", EarlyExit::TIMELIMIT);
        }
    }

    const bool changed = new_landmarks != landmarks_ret;
    landmarks_ret = std::move(new_landmarks);
    return changed;
}
