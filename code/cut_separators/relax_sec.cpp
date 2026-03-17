#include <algorithm>

#include "../utils/cycle_det.hpp"
#include "relax_callback.hpp"

[[nodiscard]]
static auto build_graph(const hplus::instance& inst, const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights)
    -> std::tuple<std::vector<std::vector<unsigned int>>, std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash>,
                  std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>> {
    std::vector<std::vector<unsigned int>> graph;
    std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash> edge_labels;
    std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> edge_weights;

    graph = std::vector<std::vector<unsigned int>>(inst.n);
    for (unsigned int pre = 0; pre < inst.n; ++pre) {
        for (const auto& act_i : inst.act_with_pre[pre]) {
            for (unsigned int i = 0; i < inst.actions[act_i].eff_sparse.size(); ++i) {
                unsigned int eff = inst.actions[act_i].eff_sparse[i];
                if (fadd_weights.at({act_i, eff}) == 0) {
                    continue;
                }
                graph[pre].push_back(eff);
                double weight = 1 - fadd_weights.at({act_i, eff});
                if (!edge_weights.contains({pre, eff}) || weight < edge_weights.at({pre, eff})) {
                    edge_labels[{pre, eff}] = inst.m + inst.fadd_cpx_start[act_i] + i;
                    edge_weights[{pre, eff}] = weight;
                }
            }
        }
    }

    return {graph, edge_labels, edge_weights};
}

auto relax_cuts::get_violated_sec(const hplus::instance& inst,
                                  const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>> {
    const auto& [graph, edge_labels, edge_weights] = build_graph(inst, fadd_weights);
    // Find cycles in the (weighted) causal relation graph using a DFS approach
    const auto& cycles{find_cycles_weighted_lessthan1(graph, edge_labels, edge_weights)};
    return {!cycles.empty(), cycles};
}

[[nodiscard]]
auto relax_cuts::add_sec_cut(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst,
                             const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights) -> unsigned int {
    const auto& [found, cycles]{get_violated_sec(inst, fadd_weights)};
    if (!found) {
        return 0;
    }
    std::vector<int> ind;
    std::vector<int> begin;
    std::vector<double> val;
    std::vector<double> rhs;
    std::vector<char> sense(cycles.size(), 'L');
    std::vector<int> purgeable(cycles.size(), CPX_USECUT_FORCE);
    std::vector<int> local(cycles.size(), 0);
    for (const auto& cycle : cycles) {
        begin.push_back(static_cast<int>(ind.size()));
        rhs.push_back(static_cast<double>(cycle.size() - 1));
        std::ranges::copy(cycle,
                          std::back_inserter(ind));  // labels in the cycle are the indexes for the first adders in the cplex model
        val.insert(val.end(), cycle.size(), 1.0);
    }
    CPX_HANDLE_CALL(CPXcallbackaddusercuts(context, cycles.size(), static_cast<int>(ind.size()), rhs.data(), sense.data(), begin.data(), ind.data(),
                                           val.data(), purgeable.data(), local.data()));
    return static_cast<unsigned int>(cycles.size());
}
