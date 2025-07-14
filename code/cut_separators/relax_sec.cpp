#include "../utils/algorithms.hpp"
#include "relax_callback.hpp"

[[nodiscard]]
static std::tuple<std::vector<std::vector<unsigned int>>, std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash>,
                  std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>>
build_graph(const hplus::instance& inst, const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights) {
    std::vector<std::vector<unsigned int>> graph;
    std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash> edge_labels;
    std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> edge_weights;

    graph = std::vector<std::vector<unsigned int>>(inst.n);
    for (unsigned int p = 0; p < inst.n; ++p) {
        for (const auto& act_i : inst.act_with_pre[p]) {
            for (unsigned int i = 0; i < inst.actions[act_i].eff_sparse.size(); ++i) {
                unsigned int q = inst.actions[act_i].eff_sparse[i];
                if (fadd_weights.at({act_i, q}) == 0) continue;
                graph[p].push_back(q);
                edge_labels[{p, q}] = inst.m + inst.fadd_cpx_start[act_i] + i;
                edge_weights[{p, q}] = fadd_weights.at({act_i, q});
            }
        }
    }

    return {graph, edge_labels, edge_weights};
}

std::pair<bool, std::vector<std::vector<unsigned int>>> relax_cuts::get_violated_sec(
    const hplus::instance& inst, const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights) {
    const auto& [graph, edge_labels, edge_weights] = build_graph(inst, fadd_weights);
    const auto& cycles{find_cycles_weighted_lessthan1(graph, edge_labels, edge_weights)};
    return {!cycles.empty(), cycles};
}

[[nodiscard]]
unsigned int relax_cuts::sec(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst,
                             const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& fadd_weights) {
    const auto& [found, cycles]{get_violated_sec(inst, fadd_weights)};
    if (!found) return 0;
    std::vector<int> ind, begin;
    std::vector<double> val;
    std::vector<double> rhs;
    std::vector<char> sense(cycles.size(), 'L');
    std::vector<int> purgeable(cycles.size(), CPX_USECUT_FORCE), local(cycles.size(), 0);
    for (const auto& cycle : cycles) {
        begin.push_back(static_cast<int>(ind.size()));
        rhs.push_back(static_cast<double>(cycle.size() - 1));
        std::copy(cycle.begin(), cycle.end(),
                  std::back_inserter(ind));  // labels in the cycle are the indexes for the first adders in the cplex model
        val.insert(val.end(), cycle.size(), 1.0);
    }
    CPX_HANDLE_CALL(CPXcallbackaddusercuts(context, cycles.size(), static_cast<int>(ind.size()), rhs.data(), sense.data(), begin.data(), ind.data(),
                                           val.data(), purgeable.data(), local.data()));
    return static_cast<unsigned int>(cycles.size());
}
