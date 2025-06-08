#include "heuristic.hpp"

[[nodiscard]]
std::pair<bool, unsigned int> heur::greedy_choice_cost(const hplus::instance& inst, const std::list<unsigned int>& candidates,
                                                       [[maybe_unused]] const binary_set& state,
                                                       [[maybe_unused]] heur::greedychoice_userhandle& userhandle) {
    unsigned int best_choice = 0;
    unsigned int best_cost = std::numeric_limits<unsigned int>::max();
    bool found = false;

    for (const auto& act_i : candidates) {
        if (inst.fixed_actions[act_i]) return {true, act_i};
        if (inst.actions[act_i].cost == 0) return {true, act_i};

        if (inst.actions[act_i].cost >= best_cost) continue;

        best_cost = inst.actions[act_i].cost;
        best_choice = act_i;
        found = true;
    }

    return {found, best_choice};
}