#include "greedycost.hpp"

#include <list>

void greedycost::run(hplus::instance& inst, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Running greedycost algorithm.");

    unsigned int timestamp{0};
    // greedy choice
    const auto find_best_act = [&inst, &timestamp](const std::list<size_t>& candidates) {
        size_t choice{0};
        int best_cost{std::numeric_limits<int>::max()};
        for (const auto& act_i : candidates) {
            if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp) return std::pair(true, act_i);
            if (inst.act_f[act_i]) {
                choice = act_i;
                best_cost = -1;
                continue;
            }

            if (best_cost < 0) continue;

            if (inst.actions[act_i].cost >= static_cast<unsigned int>(best_cost)) continue;

            choice = act_i;
            best_cost = inst.actions[act_i].cost;
        }
        return std::pair(true, choice);
    };

    hplus::solution heur_sol;
    heur_sol.plan.reserve(inst.m_opt);
    heur_sol.cost = 0;

    binary_set state{inst.n};  // initial state is empty

    std::list<size_t> candidates;
    for (const auto& act_i : inst.act_rem) {
        if (inst.actions[act_i].pre_sparse.empty()) candidates.push_back(act_i);
    }

    while (!state.contains(inst.goal)) {
        if (candidates.empty()) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        const auto [found, choice]{find_best_act(candidates)};
        if (!found) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        candidates.remove(choice);

        // add new actions to the candidates
        const auto& new_state = state | inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff - state) {
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (new_state.contains(inst.actions[act_i].pre) && std::find(candidates.begin(), candidates.end(), act_i) == candidates.end())
                    candidates.push_back(act_i);
            }
        }

        // purge unnecessary actions from candidates
        std::vector<size_t> purged_actions;
        for (const auto& act_i : candidates) {
            if (new_state.contains(inst.actions[act_i].eff) && !inst.act_f[act_i]) purged_actions.push_back(act_i);
        }
        for (const auto& act_i : purged_actions) candidates.remove(act_i);
        for (const auto& act_i : inst.act_inv[choice]) candidates.remove(act_i);

        heur_sol.plan.push_back(choice);
        heur_sol.cost += inst.actions[choice].cost;
        state = new_state;
        timestamp++;

        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

    hplus::update_sol(inst, heur_sol, log);
    env.sol_s = solution_status::FEAS;
}