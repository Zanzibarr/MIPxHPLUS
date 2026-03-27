#include <algorithm>

#include "bs.hxx"
#include "bs_utils.hpp"
#include "heuristic.hpp"
#include "hplus_algs.hpp"
#include "limits.hxx"

void heur::greedy(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
                  std::pair<bool, unsigned int> (*greedy_choice)(const hplus::instance& inst, const std::list<unsigned int>&, const BinarySet&,
                                                                 greedychoice_userhandle&)) {
    hplus::solution sol;
    sol.sequence.reserve(inst.m);
    sol.cost = 0;

    // Initialize watch preconditions
    std::vector<unsigned int> watch_pre(inst.m, inst.n);
    std::vector<std::vector<unsigned int>> watching(inst.n);

    BinarySet state{inst.n};

    std::list<unsigned int> candidates;
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            candidates.push_back(act_i);
        } else {
            // Find one unsatisfied precondition to watch
            for (const auto& pre : inst.actions[act_i].pre_sparse) {
                if (!state[pre]) {
                    watch_pre[act_i] = pre;
                    watching[pre].push_back(act_i);
                    break;
                }
            }
        }
    }

    greedychoice_userhandle userhandle;
    if (exec.ws >= hplus::warmstart::GHM) {
        // Userhandle for hmax-hadd heuristic
        userhandle.values.clear();
        userhandle.values.resize(inst.n, std::numeric_limits<double>::infinity());
        userhandle.goal_sparse = inst.goal.sparse();
        userhandle.trail = std::stack<std::pair<unsigned int, double>>{};
        userhandle.action_trail = std::stack<std::pair<unsigned int, double>>{};
        userhandle.pq = priority_queue<double>{inst.n};
        userhandle.used_actions = BinarySet(inst.m);
        heur::init_htype_values(inst, candidates, userhandle.values, userhandle.pq, exec.ws == hplus::warmstart::GHM ? heur::hmax : heur::hadd);

        if (exec.ws == hplus::warmstart::GHA) {
            // hadd optimization: initialize incremental precondition sums
            userhandle.hadd_pre.resize(inst.m, 0.0);
            for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
                double sum = 0;
                for (const auto& pre : inst.actions[act_i].pre_sparse) {
                    sum += userhandle.values[pre];
                }
                userhandle.hadd_pre[act_i] = sum;
            }
        } else {
            // hmax optimization: initialize PCF (argmax precondition) per action
            userhandle.pcf.assign(inst.m, -1);
            userhandle.pcf_val.resize(inst.m, 0.0);
            for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
                double max_val = -1;
                for (const auto& pre : inst.actions[act_i].pre_sparse) {
                    if (userhandle.values[pre] > max_val) {
                        max_val = userhandle.values[pre];
                        userhandle.pcf[act_i] = static_cast<int>(pre);
                    }
                }
                userhandle.pcf_val[act_i] = (max_val >= 0) ? max_val : 0.0;
            }
        }
    }

    while (!state.superset_of(inst.goal)) {
        if (candidates.empty()) [[unlikely]] {
            inst.sol_s = hplus::solution_status::INFEAS;
            stats.status = HPLUS_STATUS_INFEAS;
            return;
        }

        const auto [found, choice]{greedy_choice(inst, candidates, state, userhandle)};
        if (!found) [[unlikely]] {
            inst.sol_s = hplus::solution_status::INFEAS;
            stats.status = HPLUS_STATUS_INFEAS;
            return;
        }

        candidates.remove(choice);

        // Compute new effects
        std::vector<unsigned int> new_eff(inst.actions[choice].eff_sparse.begin(), inst.actions[choice].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        state |= new_eff;

        // Update candidates via watch preconditions
        for (const auto& eff : new_eff) {
            for (size_t i = 0; i < watching[eff].size();) {
                const auto act_i = watching[eff][i];

                // Try to find a new unsatisfied precondition to watch
                bool moved = false;
                for (const auto& pre : inst.actions[act_i].pre_sparse) {
                    if (!state[pre]) {
                        watch_pre[act_i] = pre;
                        watching[pre].push_back(act_i);
                        moved = true;
                        break;
                    }
                }

                // Swap-erase from watching[eff] regardless (this fact is now reached, so no action is watchin it as its watch_pre)
                watching[eff][i] = watching[eff].back();
                watching[eff].pop_back();

                if (!moved) {
                    // All preconditions satisfied -> add to candidates if not already there
                    // and if its effects are not already subsumed by the current state
                    if (std::ranges::find(candidates, act_i) == candidates.end() && !bs_contains(state, inst.actions[act_i].eff_sparse)) {
                        candidates.push_back(act_i);
                    }
                }
                // Don't increment i: swap-erase shifts the next element into position i
            }
        }

        // Purge candidates whose effects are now fully subsumed by the state
        candidates.remove_if([&](unsigned int act_i) { return bs_contains(state, inst.actions[act_i].eff_sparse) && !inst.fixed_actions[act_i]; });

        sol.sequence.push_back(choice);
        sol.cost += inst.actions[choice].cost;

        if (CHECK_STOP()) {
            [[unlikely]] throw timelimit_exception("Reached time limit.");
        }
    }

    hplus::update_sol(inst, sol, stats);
    inst.sol_s = hplus::solution_status::FEAS;
    stats.heur_cost = sol.cost;
    stats.status = HPLUS_STATUS_FEAS;
}