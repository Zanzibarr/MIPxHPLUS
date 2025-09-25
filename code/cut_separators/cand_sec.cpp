#include "../utils/algorithms.hpp"
#include "cand_callback.hpp"

[[nodiscard]]
static std::tuple<std::vector<std::vector<unsigned int>>, std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash>>
build_graph(const hplus::instance& inst, const binary_set& unreachable_actions, const std::vector<binary_set>& used_first_achievers) {
    std::vector<std::vector<unsigned int>> graph;
    std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash> edge_labels;

    graph = std::vector<std::vector<unsigned int>>(inst.n);
    for (unsigned int p = 0; p < inst.n; ++p) {
        for (const auto& act_i : inst.act_with_pre[p]) {
            if (!unreachable_actions[act_i]) continue;
            for (const auto& q : used_first_achievers[act_i]) {
                graph[p].push_back(q);
                edge_labels[{p, q}] = inst.m + inst.fadd_cpx_start[act_i] +
                                      std::distance(inst.actions[act_i].eff_sparse.begin(),
                                                    std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), q));
            }
        }
    }

    return std::tuple(graph, edge_labels);
}

[[nodiscard]]
unsigned int cand_cuts::sec(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const binary_set& unreachable_actions,
                            const std::vector<binary_set>& used_first_achievers) {
    const auto& [graph, edge_labels] = build_graph(inst, unreachable_actions, used_first_achievers);
    // Find cycles in the giustification graph using a DFS approach
    const auto& cycles = find_cycles_unweighted(graph, edge_labels);
    LOG_DEBUG << "SEC size: " << cycles.size() << " // "
              << (std::accumulate(cycles.begin(), cycles.end(), 0, [](const unsigned int sum, const auto& v) { return sum + v.size(); }) /
                  cycles.size());
    reject_with_sec_cut(context, cycles);
    return static_cast<unsigned int>(cycles.size());
}
