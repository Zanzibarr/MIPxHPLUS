#include "solver.hpp"

auto Solver::primal_greedychoice_cost_(const std::list<unsigned int>& candidates, const BinarySet& /*state*/) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    unsigned int best_cost = std::numeric_limits<unsigned int>::max();
    bool found = false;

    for (const auto& act_i : candidates) {
        if (global_.fixed_actions[act_i]) {
            return {true, act_i};
        }
        if (inst_.actions[act_i].cost == 0) {
            return {true, act_i};
        }

        if (inst_.actions[act_i].cost >= best_cost) {
            continue;
        }

        best_cost = inst_.actions[act_i].cost;
        best_choice = act_i;
        found = true;
    }

    return {found, best_choice};
}
