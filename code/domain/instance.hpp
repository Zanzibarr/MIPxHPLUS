/**
 * Methods and objects for data structures related to this project execution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

#include <cstdint>

#include "bs.hxx"
#include "execution.hpp"
#include "logger.hxx"

namespace hplus {

enum class solution_status : std::uint16_t { OPT = 0, FEAS = 1, INFEAS = 2, NOTFOUND = 404, LOST = 500 };

struct solution {
    std::vector<unsigned int> sequence;
    unsigned int cost;
    bool updating = false;
};

struct action {
    std::vector<unsigned int> pre_sparse, eff_sparse;
    unsigned int cost;
};

struct instance {
    // Instance
    bool equal_costs;
    unsigned int n, m, nfadd;
    std::vector<action> actions;
    std::vector<std::string> actions_names;
    // Preprocessing
    BinarySet eliminated_facts, eliminated_actions;  // used only for preprocessing
    BinarySet fixed_facts, fixed_actions;
    std::vector<unsigned int> fadd_cpx_start;
    std::vector<std::vector<unsigned int>> act_with_pre, act_with_eff;
    std::vector<std::vector<unsigned int>> landmarks;
    // Vertex elimination graph
    std::vector<unsigned int> veg_starts;
    std::vector<std::vector<unsigned int>> veg_cumulative_graph;
    // Goal
    BinarySet goal;
    // Solution
    solution_status sol_s;
    solution sol;
};

inline void init(instance& inst) {
    inst = instance{.equal_costs = false,
                    .n = 0,
                    .m = 0,
                    .nfadd = 0,
                    .actions = std::vector<action>(),
                    .actions_names = std::vector<std::string>(),
                    .eliminated_facts = BinarySet(),
                    .eliminated_actions = BinarySet(),
                    .fixed_facts = BinarySet(),
                    .fixed_actions = BinarySet(),
                    .fadd_cpx_start = std::vector<unsigned int>(),
                    .act_with_pre = std::vector<std::vector<unsigned int>>(),
                    .act_with_eff = std::vector<std::vector<unsigned int>>(),
                    .landmarks = std::vector<std::vector<unsigned int>>(),
                    .veg_starts = std::vector<unsigned int>(),
                    .veg_cumulative_graph = std::vector<std::vector<unsigned int>>(),
                    .goal = BinarySet(),
                    .sol_s = solution_status::NOTFOUND,
                    .sol = solution{.sequence = std::vector<unsigned int>(), .cost = std::numeric_limits<unsigned int>::max(), .updating = false}};
}

inline void print(const instance& inst) {
    LOG_S("----------------- Info on the instance -----------------");
    if (!inst.actions.empty()) {
        const std::string cost_type = inst.actions[0].cost == 1 ? "unitary costs" : "constant costs";
        LOG << "Metric:                             " << std::setw(20) << (inst.equal_costs ? cost_type : "integer costs");
    }
    LOG << "# facts:                                      " << std::setw(10) << inst.n;
    LOG << "# actions:                                    " << std::setw(10) << inst.m;
    LOG << "# first adders:                               " << std::setw(10) << inst.nfadd;
    LOG_S("--------------------------------------------------------");
}

inline void print_sol(const execution& exec, instance& inst) {
    if (inst.sol.updating) {
        LOG_WARN_S("Execution terminated while updating the solution and the solution got lost");
        inst.sol_s = solution_status::LOST;
    }
    if (exec.alg == algorithm::LMCUT && inst.sol_s == solution_status::FEAS) {
        LOG_S("Only a lower bound has been computed through LM-Cut");
        return;
    }
    switch (inst.sol_s) {
        case solution_status::LOST:
            break;
        case solution_status::INFEAS:
            LOG_S("The problem is infeasible");
            break;
        case solution_status::NOTFOUND:
            LOG_S("No solution found within memory and time limits");
            break;
        case solution_status::FEAS:
            LOG_S("The solution found has not been proven optimal");
            [[fallthrough]];
        case solution_status::OPT:
            LOG_S("Solution cost: " + std::to_string(inst.sol.cost));
            // for (const auto& act_idx : inst.sol.sequence) {
            //     LOG_S("(" + inst.actions_names[act_idx] + ")");
            // }
            break;
        default:
            LOG_ERROR_S("Unhandled solution status in hplus::print_sol(hplus::instance): " + std::to_string(static_cast<int>(inst.sol_s)));
    }
}
}  // namespace hplus
