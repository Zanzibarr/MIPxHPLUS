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

typedef struct {
    unsigned int to;   // Destination node
    unsigned int rev;  // Index of reverse edge in the destination's adjacency list
    double c;          // Remaining capacity of the edge
    bool is_reverse;
} network_edge;

// ##################################################################### //
// ########################## GRAPH DEBUGGING ########################## //
// ##################################################################### //

/**
 * @brief Write to file a graph in dot style (only edges with positive flow are displayed)
 *
 * @param file_path Path of where to save the dot graph
 * @param graph adjacency list
 */
static inline void write_graph(const std::string& file_path, const std::vector<std::vector<network_edge>>& graph) {
    std::ofstream file(file_path);

    file << "digraph MyGraph {\n";
    file << "   rankdir=LR;\n";
    file << "   node [shape=circle];\n\n";

    for (unsigned int from = 0; from < graph.size(); from++) {
        for (const auto& [to, rev, c, is_rev] : graph[from]) {
            if (is_rev) continue;
            double flow = graph[to][rev].c, res_cap = c;

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
[[nodiscard]]
static inline bool is_flow_conservative(const std::vector<std::vector<network_edge>>& graph, unsigned int source, unsigned int sink) {
    std::vector<double> flow_at(graph.size(), 0);
    for (unsigned int u = 0; u < graph.size(); u++) {
        for (const auto& [v, rev, flow, is_rev] : graph[u]) {
            if (!is_rev) continue;
            flow_at[u] += flow;
            flow_at[v] -= flow;
        }
    }

    bool result = true;

    for (unsigned int u = 0; u < graph.size(); u++) {
        if (u == sink || u == source) continue;
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
static inline bool max_flow_augmenting_path(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int n,
                                            std::vector<int>& level) {
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
static inline double max_flow_push_flow(std::vector<std::vector<network_edge>>& graph, unsigned int v, unsigned int t, double f,
                                        std::vector<int>& level, std::vector<int>& iter) {
    if (v == t) return f;  // Reached sink, return the flow

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
static inline double compute_max_flow(std::vector<std::vector<network_edge>>& graph, unsigned int source, unsigned int sink) {
    double flow{0};
    const unsigned int n{static_cast<unsigned int>(graph.size())};
    std::vector<int> level(n), iter(n);

    // Repeat while there's an augmenting path from source to sink
    while (max_flow_augmenting_path(graph, source, sink, n, level)) {
        iter.assign(n, 0);
        double f;

        // Keep finding blocking flows until no more paths exist in this level graph
        while ((f = max_flow_push_flow(graph, source, sink, std::numeric_limits<double>::infinity(), level, iter)) > HPLUS_EPSILON) flow += f;
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
static inline binary_set get_min_cut_lpartition(const std::vector<std::vector<network_edge>>& graph, unsigned int source) {
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

/** @brief Iterative DFS algorithm to remove flow from a u -> target path (if any) */
[[nodiscard]]
static double dfs_remove_flow(std::vector<std::vector<network_edge>>& graph, unsigned int u, const unsigned int target, double flow_to_remove,
                              binary_set& visited) {
    struct stack_frame {
        unsigned int node;
        double flow;
        size_t edge_index;
    };

    std::vector<stack_frame> stack;
    std::vector<size_t> parent_edge(graph.size(), SIZE_MAX);
    std::vector<unsigned int> parent_node(graph.size(), UINT_MAX);

    stack.push_back({u, flow_to_remove, 0});
    visited.add(u);

    while (!stack.empty()) {
        auto& frame = stack.back();

        // Found target - backtrack and update edges
        if (frame.node == target) {
            double pushed = frame.flow;
            unsigned int curr = target;

            // Backtrack from target to source, updating edges
            while (curr != u) {
                unsigned int prev = parent_node[curr];
                size_t edge_idx = parent_edge[curr];

                auto& edge = graph[prev][edge_idx];
                graph[edge.to][edge.rev].c -= pushed;  // Reduce flow
                edge.c += pushed;                      // Increase residual capacity

                curr = prev;
            }

            return pushed;
        }

        // Try to find an unexplored edge
        bool found_edge = false;
        while (frame.edge_index < graph[frame.node].size()) {
            auto& edge = graph[frame.node][frame.edge_index];

            if (!visited[edge.to] && !edge.is_reverse) {
                double available_flow = graph[edge.to][edge.rev].c;

                if (available_flow > HPLUS_EPSILON) {
                    // Found a valid edge - explore it
                    visited.add(edge.to);
                    parent_node[edge.to] = frame.node;
                    parent_edge[edge.to] = frame.edge_index;

                    stack.push_back({edge.to, std::min(frame.flow, available_flow), 0});
                    found_edge = true;
                    break;
                }
            }

            frame.edge_index++;
        }

        // No more edges to explore from this node - backtrack
        if (!found_edge) {
            stack.pop_back();
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
 * @return Amount of flow succesfully removed
 */
[[nodiscard]]
static inline double flow_removal(std::vector<std::vector<network_edge>>& graph, unsigned int from, unsigned int to, const double flow_to_remove) {
    double remaining = flow_to_remove;

    // Keep finding paths and removing flow until we've removed enough
    while (remaining > HPLUS_EPSILON) {
        binary_set visited(graph.size());
        double removed = dfs_remove_flow(graph, from, to, remaining, visited);
        remaining -= removed;

        if (removed <= HPLUS_EPSILON) {
            // Loop detected, no flow removed: this happens when we try to remove flow from an edge a->b inside a loop that has no connections with
            // the rest of the graph... since we usually call flow_remove(b, sink) AND flow_remove(source, a), if a->b is in a isolated loop the first
            // call will find a loop while the other will find no path...
            return flow_to_remove - remaining;
        }
    }

    return flow_to_remove;
}

#endif