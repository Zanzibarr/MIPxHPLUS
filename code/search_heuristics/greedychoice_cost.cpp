#include "heuristic.hpp"

[[nodiscard]]
auto heur::greedy_choice_cost(const hplus::instance& inst, const std::list<unsigned int>& candidates, const BinarySet& /*state*/,
                              heur::greedychoice_userhandle& /*userhandle*/) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    unsigned int best_cost = std::numeric_limits<unsigned int>::max();
    bool found = false;

    for (const auto& act_i : candidates) {
        if (inst.fixed_actions[act_i]) {
            return {true, act_i};
        }
        if (inst.actions[act_i].cost == 0) {
            return {true, act_i};
        }

        if (inst.actions[act_i].cost >= best_cost) {
            continue;
        }

        best_cost = inst.actions[act_i].cost;
        best_choice = act_i;
        found = true;
    }

    return {found, best_choice};
}