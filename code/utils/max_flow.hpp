/**
 * Max flow algorithms
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_MAX_FLOW_HPP
#define HPLUS_MAX_FLOW_HPP

#include <queue>

#include "../external/bs.hxx"
#include "utils.hpp"

using network_edge = struct {
    unsigned int to;   // Destination node
    unsigned int rev;  // Index of reverse edge in the destination's adjacency list
    double c;          // Remaining capacity of the edge
    bool is_reverse;
};

// ##################################################################### //
// ########################## GRAPH DEBUGGING ########################## //
// ##################################################################### //

/**
 * @brief Write to file a graph in dot style (only edges with positive flow are displayed)
 *
 * @param file_path Path of where to save the dot graph
 * @param graph adjacency list
 */
[[maybe_unused]]
static inline void write_graph(const std::string& file_path, const std::vector<std::vector<network_edge>>& graph) {
    std::ofstream file(file_path);

    file << "digraph MyGraph {\n";
    file << "   rankdir=LR;\n";
    file << "   node [shape=circle];\n\n";

    for (unsigned int from = 0; from < graph.size(); from++) {
        for (const auto& [to, rev, c, is_rev] : graph[from]) {
            if (is_rev) {
                continue;
            }
            double flow = graph[to][rev].c;
            double res_cap = c;

            if (flow > HPLUS_EPSILON) {
                file << "   " << from << " -> " << to << " [label=\"" << res_cap << "/" << flow << "\"];\n";
            }
        }
    }

    file << "}\n";
    file.close();
}

/**
 * @brief Check whether the current flow in a network is conservative
 *
 * @param graph adjacency list
 * @param source Source node
 * @param sink Sink node
 * @return true/false based on whether the flow conservation is respected or not
 */
[[nodiscard]] [[maybe_unused]]
static inline auto is_flow_conservative(const std::vector<std::vector<network_edge>>& graph, unsigned int source, unsigned int sink) -> bool {
    std::vector<double> flow_at(graph.size(), 0);
    for (unsigned int u = 0; u < graph.size(); u++) {
        for (const auto& [v, rev, flow, is_rev] : graph[u]) {
            if (!is_rev) {
                continue;
            }
            flow_at[u] += flow;
            flow_at[v] -= flow;
        }
    }

    bool result = true;

    for (unsigned int u = 0; u < graph.size(); u++) {
        if (u == sink || u == source) {
            continue;
        }
        if (std::abs(flow_at[u]) > HPLUS_EPSILON) {
            result = false;
            LOG_WARNING << "Flow is not conserved at node " << u;
        }
    }

    if (std::abs(flow_at[source] + flow_at[sink]) > HPLUS_EPSILON) {
        result = false;
        LOG_WARNING << "Flow outoging from source is not the same going into the sink";
    }

    return result;
}

// ##################################################################### //
// ######################### STANDARD MAX FLOW ######################### //
// ##################################################################### //

/** @brief Look for an augmenting path */
[[nodiscard]]
static inline auto max_flow_augmenting_path(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int n,
                                            std::vector<int>& level) -> bool {
    level.assign(n, -1);  // Initialize all levels to -1 (unvisited)
    std::queue<int> q;
    level[s] = 0;
    q.push(s);

    while (!q.empty()) {
        const auto v{q.front()};
        q.pop();
        for (const auto& e : graph[v]) {
            // If this edge still has capacity and it hasn't been visited
            if (e.c > HPLUS_EPSILON && level[e.to] < -HPLUS_EPSILON) {
                level[e.to] = level[v] + 1;  // Set the level of this neighbor
                q.push(e.to);
            }
        }
    }

    return level[t] >= 0;  // If the sink is reachable, return true
}

/** @brief Push flow through the augmenting path */
[[nodiscard]]
static inline auto max_flow_push_flow(std::vector<std::vector<network_edge>>& graph, unsigned int v, unsigned int t, double f,
                                      std::vector<int>& level, std::vector<int>& iter) -> double {
    if (v == t) {
        return f;  // Reached sink, return the flow
    }

    // iter[v] remembers which edge to start from (optimization to avoid revisiting dead ends)
    for (int& i = iter[v]; i < static_cast<int>(graph[v].size()); i++) {
        auto& e = graph[v][i];

        // Only use edges with capacity that go to next level
        if (e.c > HPLUS_EPSILON && level[v] < level[e.to]) {
            double d{max_flow_push_flow(graph, e.to, t, std::min(f, e.c), level, iter)};
            if (d > HPLUS_EPSILON) {
                // Update residual capacities
                e.c -= d;
                graph[e.to][e.rev].c += d;
                return d;
            }
        }
    }

    return 0;
}

/**
 * @brief Compute the max flow on a graph
 *
 * @param graph adjacency list (modified in place)
 * @param source Source node
 * @param sink Sink node
 *
 * @return The computed max flow
 */
[[nodiscard]]
static inline auto compute_max_flow(std::vector<std::vector<network_edge>>& graph, unsigned int source, unsigned int sink) -> double {
    double flow{0};
    const unsigned int n{static_cast<unsigned int>(graph.size())};
    std::vector<int> level(n);
    std::vector<int> iter(n);

    // Repeat while there's an augmenting path from source to sink
    while (max_flow_augmenting_path(graph, source, sink, n, level)) {
        iter.assign(n, 0);
        double f;

        // Keep finding blocking flows until no more paths exist in this level graph
        while ((f = max_flow_push_flow(graph, source, sink, std::numeric_limits<double>::infinity(), level, iter)) > HPLUS_EPSILON) {
            flow += f;
        }
    }

    return flow;
}

/**
 * @brief Compute the left partition of a graph based on flow and capacities
 *
 * @param graph adjacency list
 * @param source Source node
 *
 * @return The left partition of the graph
 */
[[nodiscard]]
static inline auto get_min_cut_lpartition(const std::vector<std::vector<network_edge>>& graph, unsigned int source) -> binary_set {
    binary_set reachable(graph.size());
    std::queue<unsigned int> q;
    q.push(source);
    reachable.add(source);

    while (!q.empty()) {
        const auto v = q.front();
        q.pop();

        for (const auto& e : graph[v]) {
            if (e.c > HPLUS_EPSILON && !reachable[e.to]) {
                reachable.add(e.to);
                q.push(e.to);
            }
        }
    }

    return reachable;
}

// ##################################################################### //
// ############## INCREMENTAL MAX FLOW ON CHANGING GRAPHS ############## //
// ##################################################################### //

/** @brief BFS algorithm to remove flow from source -> target path (if any) */
[[nodiscard]]
static auto bfs_remove_flow(std::vector<std::vector<network_edge>>& graph, unsigned int source, unsigned int target, double max_flow,
                            std::vector<size_t>& parent_edge, std::vector<unsigned int>& parent_node, binary_set& visited) -> double {
    // Reset visited set
    visited.clear();

    std::queue<std::pair<unsigned int, double>> queue;
    queue.push({source, max_flow});
    visited.add(source);
    parent_node[source] = UINT_MAX;

    while (!queue.empty()) {
        auto [node, flow] = queue.front();
        queue.pop();

        if (node == target) {
            // Found path - backtrack and update edges
            double pushed = flow;
            unsigned int curr = target;

            while (curr != source) {
                unsigned int prev = parent_node[curr];
                size_t edge_idx = parent_edge[curr];
                auto& edge = graph[prev][edge_idx];
                graph[edge.to][edge.rev].c -= pushed;  // Reduce flow
                edge.c += pushed;                      // Increase residual capacity
                curr = prev;
            }
            return pushed;
        }

        for (size_t i = 0; i < graph[node].size(); i++) {
            auto& edge = graph[node][i];
            if (!visited[edge.to] && !edge.is_reverse) {
                double available_flow = graph[edge.to][edge.rev].c;
                if (available_flow > HPLUS_EPSILON) {
                    visited.add(edge.to);
                    parent_node[edge.to] = node;
                    parent_edge[edge.to] = i;
                    queue.push({edge.to, std::min(flow, available_flow)});
                }
            }
        }
    }

    return 0;  // No path found
}

/**
 * @brief Remove flow in the path from -> to
 *
 * @param graph adjacency list (modified in place)
 * @param from Start node
 * @param to End node
 * @param flow_to_remove Flow to remove
 *
 * @return Amount of flow successfully removed
 */
[[nodiscard]]
static inline auto flow_removal(std::vector<std::vector<network_edge>>& graph, unsigned int from, unsigned int to, const double flow_to_remove)
    -> double {
    // Pre-allocate structures to avoid repeated allocations
    std::vector<size_t> parent_edge(graph.size());
    std::vector<unsigned int> parent_node(graph.size());
    binary_set visited(graph.size());

    double remaining = flow_to_remove;

    // Keep finding paths and removing flow until we've removed enough
    while (remaining > HPLUS_EPSILON) {
        double removed = bfs_remove_flow(graph, from, to, remaining, parent_edge, parent_node, visited);
        remaining -= removed;

        if (removed <= HPLUS_EPSILON) {
            // No more flow can be removed (isolated loop or no path)
            return flow_to_remove - remaining;
        }
    }

    return flow_to_remove;
}

#endif