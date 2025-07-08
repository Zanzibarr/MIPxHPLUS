#include "heuristic.hpp"

[[nodiscard]]
std::pair<bool, unsigned int> heur::greedy_choice_cxe(const hplus::instance& inst, const std::list<unsigned int>& candidates, const binary_set& state,
                                                      [[maybe_unused]] heur::greedychoice_userhandle& userhandle) {
    unsigned int best_choice = 0;
    double best_cxe = std::numeric_limits<double>::max();
    bool found = false;

    for (const auto& act_i : candidates) {
        if (inst.fixed_actions[act_i]) return {true, act_i};
        if (inst.actions[act_i].cost == 0) return {true, act_i};

        unsigned int neffects{0};
        for (const auto& eff : inst.actions[act_i].eff_sparse) {
            if (state[eff]) continue;
            neffects++;
        }
        if (neffects == 0) continue;

        const double cxe{static_cast<double>(inst.actions[act_i].cost) / neffects};

        if (cxe >= best_cxe) continue;

        best_cxe = cxe;
        best_choice = act_i;
        found = true;
    }

    return {found, best_choice};
}