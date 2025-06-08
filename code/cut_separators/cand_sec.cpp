#include "../utils/algorithms.hpp"
#include "cand_callback.hpp"

[[nodiscard]]
static std::tuple<std::vector<std::vector<unsigned int>>, std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash>>
build_graph(const hplus::execution& exec, const hplus::instance& inst, const binary_set& unreachable_actions,
            const std::vector<binary_set>& used_first_achievers) {
    std::vector<std::vector<unsigned int>> graph;
    std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int, pair_hash> edge_labels;

    // TODO: Test which of these two methods works best

    // Facts as nodes
    if (exec.testing) {
        graph = std::vector<std::vector<unsigned int>>(inst.n);
        for (unsigned int p = 0; p < inst.n; ++p) {
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (!unreachable_actions[act_i]) continue;
                for (unsigned int i = 0; i < inst.actions[act_i].eff_sparse.size(); ++i) {
                    unsigned int q = inst.actions[act_i].eff_sparse[i];
                    if (!used_first_achievers[act_i][q]) continue;
                    graph[p].push_back(q);
                    edge_labels[{p, q}] = inst.m + inst.fadd_cpx_start[act_i] + i;
                }
            }
        }
    }

    // Actions as nodes
    if (!exec.testing) {
        graph = std::vector<std::vector<unsigned int>>(inst.m);
        for (const auto& act_i : unreachable_actions) {
            for (const auto& p : used_first_achievers[act_i]) {
                for (const auto& act_j : inst.act_with_pre[p]) {
                    if (!unreachable_actions[act_j]) continue;
                    graph[act_i].push_back(act_j);
                    edge_labels[{act_i, act_j}] =
                        inst.m + inst.fadd_cpx_start[act_i] +
                        std::distance(inst.actions[act_i].eff_sparse.begin(),
                                      std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), p));
                }
            }
        }
    }

    return std::tuple(graph, edge_labels);
}

[[nodiscard]]
unsigned int cand_cuts::sec(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst,
                            const binary_set& unreachable_actions, const std::vector<binary_set>& used_first_achievers) {
    const auto& [graph, edge_labels] = build_graph(exec, inst, unreachable_actions, used_first_achievers);
    const auto& cycles = find_cycles_unweighted(graph, edge_labels);
    // [[DEBUG]]
    // int counter = 0;
    // for (const auto& adj : graph) {
    //     if (!adj.empty()) std::cout << counter << ": ";
    //     for (const auto x : adj) std::cout << x << " ";
    //     if (!adj.empty()) std::cout << std::endl;
    //     counter += adj.size();
    // }
    // LOG_DEBUG << edge_labels.size() << " " << cycles.size() << " " << static_cast<double>(counter) / cycles.size();
    // int a;
    // std::cin >> a;
    reject_with_sec_cut(context, cycles);
    return static_cast<unsigned int>(cycles.size());
}
