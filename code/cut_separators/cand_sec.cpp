#include "../utils/algorithms.hpp"
#include "cand_callback.hpp"

[[nodiscard]]
static std::tuple<std::vector<std::vector<unsigned int>>, std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash>>
build_graph(const hplus::instance& inst, const binary_set& unreachable_actions, const std::vector<std::vector<unsigned int>>& used_first_achievers) {
    std::vector<std::vector<unsigned int>> graph;
    std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash> edge_labels;

    graph = std::vector<std::vector<unsigned int>>(inst.n);
    for (unsigned int p = 0; p < inst.n; ++p) {
        for (const auto& act_i : inst.act_with_pre[p]) {
            if (!unreachable_actions[act_i]) continue;
            for (const auto& q : used_first_achievers[act_i]) {
                graph[p].push_back(q);
                // There are few enough items in eff_sparse so that a linear search is the fastest option
                edge_labels[{p, q}] = inst.m + inst.fadd_cpx_start[act_i] +
                                      std::distance(inst.actions[act_i].eff_sparse.begin(),
                                                    std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), q));
            }
        }
    }

    // Idea: sort in decreasing order of out_degree... once we get into a hub of nodes we likely will find a cycle in there, since high degree nodes
    // (hence those in the hub) are explored earlier
    for (const auto& neighbors : graph)
        std::sort(neighbors.begin(), neighbors.end(), [&](unsigned int a, unsigned int b) { return graph[a].size() > graph[b].size(); });
    std::sort(graph.begin(), graph.end(),
              [&](const std::vector<unsigned int>& a, const std::vector<unsigned int>& b) { return a.size() > b.size(); });

    return std::tuple(graph, edge_labels);
}

[[nodiscard]]
unsigned int cand_cuts::sec(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const binary_set& unreachable_actions,
                            const std::vector<std::vector<unsigned int>>& used_first_achievers) {
    const auto& [graph, edge_labels] = build_graph(inst, unreachable_actions, used_first_achievers);
    // Find cycles in the giustification graph using a DFS approach
    auto cycles = find_cycles_unweighted(graph, edge_labels);
    // std::sort(cycles.begin(), cycles.end(),
    //           [&](const std::vector<unsigned int>& a, const std::vector<unsigned int>& b) { return a.size() < b.size(); });
    // cycles.resize(std::min(static_cast<size_t>(5), cycles.size()));
    reject_with_sec_cut(context, cycles);
    return static_cast<unsigned int>(cycles.size());
}
