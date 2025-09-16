/**
 * Methods and objects for data structures related to this project execution
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_INST_HPP
#define HPLUS_INST_HPP

#include "../external/bs.hpp"

#define INFBOUND_DBL 1e20
#define INFBOUND_INT std::numeric_limits<unsigned int>::max()

namespace hplus {

enum class solution_status { OPT = 0, FEAS = 1, INFEAS = 2, NOTFOUND = 404, LOST = 500 };

struct solution {
    std::vector<unsigned int> sequence;
    unsigned int cost;
    bool updating = false;
};

struct action {
    binary_set pre, eff;
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
    binary_set eliminated_facts, eliminated_actions;  // temporary, for preprocessing
    binary_set fixed_facts, fixed_actions;
    std::vector<unsigned int> fadd_cpx_start;
    std::vector<std::vector<unsigned int>> act_with_pre, act_with_eff;
    std::vector<std::vector<unsigned int>> landmarks;
    // Vertex elimination graph
    std::vector<unsigned int> veg_starts;
    std::vector<std::vector<unsigned int>> veg_cumulative_graph;
    // Goal
    binary_set goal;
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
                    .eliminated_facts = binary_set(),
                    .eliminated_actions = binary_set(),
                    .fixed_facts = binary_set(),
                    .fixed_actions = binary_set(),
                    .fadd_cpx_start = std::vector<unsigned int>(),
                    .act_with_pre = std::vector<std::vector<unsigned int>>(),
                    .act_with_eff = std::vector<std::vector<unsigned int>>(),
                    .landmarks = std::vector<std::vector<unsigned int>>(),
                    .veg_starts = std::vector<unsigned int>(),
                    .veg_cumulative_graph = std::vector<std::vector<unsigned int>>(),
                    .goal = binary_set(),
                    .sol_s = solution_status::NOTFOUND,
                    .sol = solution{.sequence = std::vector<unsigned int>(), .cost = std::numeric_limits<unsigned int>::max(), .updating = false}};
}

inline void print(const instance& inst) {
    LOG << "----------------- Info on the instance -----------------";
    if (!inst.actions.empty())
        LOG << "Metric:                             " << std::setw(20)
            << (inst.equal_costs ? (inst.actions[0].cost == 1 ? "unitary costs" : "constant costs") : "integer costs");
    LOG << "# facts:                                      " << std::setw(10) << inst.n;
    LOG << "# actions:                                    " << std::setw(10) << inst.m;
    LOG << "# first adders:                               " << std::setw(10) << inst.nfadd;
    LOG << "--------------------------------------------------------";
}

inline void print_sol(instance& inst) {
    if (inst.sol.updating) {
        LOG_WARNING << "Execution terminated while updating the solution and the solution got lost";
        inst.sol_s = solution_status::LOST;
    }
    switch (inst.sol_s) {
        case solution_status::LOST:
            break;
        case solution_status::INFEAS:
            LOG << "The problem is infeasible";
            break;
        case solution_status::NOTFOUND:
            LOG << "No solution found within memory and time limits";
            break;
        case solution_status::FEAS:
            LOG << "The solution found has not been proven optimal";
            [[fallthrough]];
        case solution_status::OPT:
            LOG << "Solution cost: " << inst.sol.cost;
            // for (const auto& act_idx : inst.sol.sequence) LOG << "(" << inst.actions_names[act_idx] << ")";
            break;
        default:
            LOG_ERROR << "Unhandled solution status in hplus::print_sol(hplus::instance): " << static_cast<int>(inst.sol_s);
    }
}
}  // namespace hplus

#endif