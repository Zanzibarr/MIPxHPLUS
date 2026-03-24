#include <algorithm>

#include "bs.hxx"
#include "bs_utils.hpp"
#include "heuristic.hpp"
#include "hplus_algs.hpp"
#include "limits.hxx"

// TODO: Consider using watch preconditions here aswell
void heur::greedy(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
                  std::pair<bool, unsigned int> (*greedy_choice)(const hplus::instance& inst, const std::list<unsigned int>&, const BinarySet&,
                                                                 greedychoice_userhandle&)) {
    hplus::solution sol;
    sol.sequence.reserve(inst.m);
    sol.cost = 0;

    std::list<unsigned int> candidates;
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            candidates.push_back(act_i);
        }
    }

    BinarySet state{inst.n};
    greedychoice_userhandle userhandle;
    if (exec.ws >= hplus::warmstart::GHM) {
        // Userhandle for hmax-hadd heuristic
        userhandle.values.clear();
        userhandle.values.resize(inst.n, std::numeric_limits<double>::infinity());
        userhandle.goal_sparse = inst.goal.sparse();
        userhandle.trail = std::stack<std::pair<unsigned int, double>>{};
        userhandle.pq = priority_queue<double>{inst.n};
        userhandle.used_actions = BinarySet(inst.m);
        heur::init_htype_values(inst, candidates, userhandle.values, userhandle.pq, exec.ws == hplus::warmstart::GHM ? heur::hmax : heur::hadd);
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

        // add new actions to the candidates
        std::vector<unsigned int> new_eff(inst.actions[choice].eff_sparse.begin(), inst.actions[choice].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        state |= new_eff;

        for (const auto& eff : new_eff) {
            for (const auto& act_i : inst.act_with_pre[eff]) {
                if (bs_contains(state, inst.actions[act_i].pre_sparse) && std::ranges::find(candidates, act_i) == candidates.end()) {
                    candidates.push_back(act_i);
                }
            }
        }

        // purge unnecessary actions from candidates
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