#include "solver.hpp"

auto Solver::primal_greedychoice_cxe_(const std::list<unsigned int>& candidates, const BinarySet& state) -> std::pair<bool, unsigned int> {
    unsigned int best_choice = 0;
    double best_cxe = std::numeric_limits<double>::max();
    bool found = false;

    for (const auto& act_i : candidates) {
        if (global_.fixed_actions[act_i]) {
            return {true, act_i};
        }
        if (inst_.actions[act_i].cost == 0) {
            return {true, act_i};
        }

        unsigned int neffects{0};
        for (const auto& eff : inst_.actions[act_i].eff_sparse) {
            if (state[eff]) {
                continue;
            }
            neffects++;
        }
        if (neffects == 0) {
            continue;
        }

        const double cxe{static_cast<double>(inst_.actions[act_i].cost) / neffects};

        if (cxe >= best_cxe) {
            continue;
        }

        best_cxe = cxe;
        best_choice = act_i;
        found = true;
    }

    return {found, best_choice};
}
