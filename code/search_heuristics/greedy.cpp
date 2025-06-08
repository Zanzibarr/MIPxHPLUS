#include "heuristic.hpp"

void heur::greedy(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats,
                  std::pair<bool, unsigned int> (*greedy_choice)(const hplus::instance& inst, const std::list<unsigned int>&, const binary_set&,
                                                                 greedychoice_userhandle&)) {
    hplus::solution sol;
    sol.sequence.reserve(inst.m);
    sol.cost = 0;

    std::list<unsigned int> candidates;
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (inst.actions[act_i].pre_sparse.empty()) candidates.push_back(act_i);
    }

    binary_set state{inst.n};
    greedychoice_userhandle userhandle;
    if (exec.ws >= hplus::warmstart::GHM) {
        // Userhandle for hmax-hadd heuristic
        userhandle.values.clear(); 
        userhandle.values.resize(inst.n, std::numeric_limits<double>::infinity());
        userhandle.goal_sparse = inst.goal.sparse();
        userhandle.trail = std::stack<std::pair<unsigned int, double>>{};
        userhandle.pq = priority_queue<double>{inst.n};
        userhandle.used_actions = binary_set(inst.m);
        heur::init_htype_values(inst, candidates, userhandle.values, userhandle.pq, exec.ws == hplus::warmstart::GHM ? heur::hmax : heur::hadd);
    }

    while (!state.contains(inst.goal)) {
        if (candidates.empty()) [[unlikely]] {
            inst.sol_s = hplus::solution_status::INFEAS;
            stats.status = 1;
            return;
        }

        const auto [found, choice]{greedy_choice(inst, candidates, state, userhandle)};
        if (!found) [[unlikely]] {
            inst.sol_s = hplus::solution_status::INFEAS;
            stats.status = 1;
            return;
        }

        candidates.remove(choice);

        // add new actions to the candidates
        const auto& new_state = state | inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff_sparse) {
            if (state[p]) continue;
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (new_state.contains(inst.actions[act_i].pre) && std::find(candidates.begin(), candidates.end(), act_i) == candidates.end())
                    candidates.push_back(act_i);
            }
        }

        // purge unnecessary actions from candidates
        candidates.remove_if([&](unsigned int act_i) { return new_state.contains(inst.actions[act_i].eff) && !inst.fixed_actions[act_i]; });

        sol.sequence.push_back(choice);
        sol.cost += inst.actions[choice].cost;
        state = new_state;

        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

    hplus::update_sol(exec, inst, sol, stats);
    inst.sol_s = hplus::solution_status::FEAS;
    stats.heur_cost = sol.cost;
    stats.status = 2;
}