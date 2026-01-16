/**
 * Utility functions used in various parts of this project
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_UTILS_ALGORITHMS_HPP
#define HPLUS_UTILS_ALGORITHMS_HPP

#include <stdlib.h>  // size_t

#include <algorithm>  // std::lower_bound
#include <numeric>    // std::iota
#include <queue>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

#include "../external/bs.hxx"
#include "utils.hpp"

// ##################################################################### //
// ##################### VECTOR SORTING AND SEARCH ##################### //
// ##################################################################### //

template <typename T>
static inline void insert_sorted(std::vector<T>& vec, T value) {
    auto it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end() || *it != value) {
        vec.insert(it, value);
    }
}

template <typename T>
[[nodiscard]]
static inline size_t sorted_find(const std::vector<T>& vec, T value) {
    auto it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it != vec.end() && *it == value) {
        return static_cast<size_t>(it - vec.begin());
    }
    return static_cast<size_t>(-1);  // Not found
}

// ##################################################################### //
// #################### CYCLE DETECTION, UNWEIGHTED #################### //
// ##################################################################### //

struct pair_hash {
    std::size_t operator()(const std::pair<unsigned int, unsigned int>& p) const {
        return std::hash<unsigned int>()(p.first) ^ (std::hash<unsigned int>()(p.second) << 1);
    }
};

struct dfs_state {
    unsigned int vertex;
    unsigned int neighbor_idx;
    bool started;

    dfs_state(unsigned int v) : vertex(v), neighbor_idx(0), started(false) {}
};

/**
 * @brief Find a cycle starting (not necessarily containing) from a start node, in a graph, returning the cycle as the labels of the edges that
 * compose the found cycle, skipping removed edges and removed nodes
 */
template <typename T>
static inline void cycle_dfs(const std::vector<std::vector<unsigned int>>& graph,
                             const std::unordered_map<std::pair<unsigned int, unsigned int>, T, pair_hash>& edge_labels, unsigned int start_vertex,
                             std::vector<std::vector<bool>>& removed_edges, std::vector<bool>& removed_nodes, std::vector<std::vector<T>>& cycles) {
    std::vector<dfs_state> stack;
    std::vector<unsigned int> current_path;
    std::vector<std::pair<unsigned int, unsigned int>> current_path_edges;
    std::vector<bool> visited(graph.size(), false);   // Per-call visited
    std::vector<bool> in_stack(graph.size(), false);  // Tracks recursion path

    stack.emplace_back(start_vertex);

    while (!stack.empty()) {
        // This gets the latest "opened" node
        auto& state = stack.back();

        // If this is the first time opening this node, initialize its stack values
        if (!state.started) {
            state.started = true;
            visited[state.vertex] = true;
            in_stack[state.vertex] = true;
            current_path.push_back(state.vertex);
        }

        bool advanced = false;
        // Resume to whichever state it was when we left it (or from 0 if it has just been opened)
        while (state.neighbor_idx < graph[state.vertex].size()) {
            // Get info on the neighbor we need to open next
            unsigned int neighbor_idx = state.neighbor_idx;
            unsigned int neighbor = graph[state.vertex][neighbor_idx];
            ++state.neighbor_idx;

            // If we already used this edge we skip (we don't care about a cycle where one of its edges was already in a cycle)
            if (removed_edges[state.vertex][neighbor_idx]) continue;

            // If we reached an element that was already in the stack, we found a cycle
            if (in_stack[neighbor]) {
                // Found a back edge -> cycle detected
                auto cycle_start_it = std::find(current_path.begin(), current_path.end(), neighbor);
                unsigned int start_index = std::distance(current_path.begin(), cycle_start_it);

                std::vector<T> cycle_labels;
                std::vector<std::pair<unsigned int, unsigned int>> cycle_edges;

                // Build back the path from the edge labels in the path
                for (unsigned int i = start_index; i < current_path_edges.size(); ++i) {
                    auto [from, to_idx] = current_path_edges[i];
                    cycle_labels.push_back(edge_labels.at({from, graph[from][to_idx]}));
                    cycle_edges.emplace_back(from, to_idx);
                }

                // Close the cycle with the last edge
                cycle_labels.push_back(edge_labels.at({state.vertex, graph[state.vertex][neighbor_idx]}));
                cycle_edges.emplace_back(state.vertex, neighbor_idx);

                // Store cycles and mark each of these edges as already used
                cycles.push_back(std::move(cycle_labels));
                for (const auto& [from, to_idx] : cycle_edges) removed_edges[from][to_idx] = true;

                return;
            }

            // Otherwise, if the neighbor wasn't in the stack, and we haven't seen it yet, add it to the stack as the next one to visit (dfs logic)
            // Also, if we already COMPLETELY explored this node in another DFS call, we can ignore it
            if (!visited[neighbor] && !removed_nodes[neighbor]) {
                current_path_edges.emplace_back(state.vertex, neighbor_idx);
                stack.emplace_back(neighbor);
                advanced = true;
                break;
            }
        }

        // we explored all paths from this node, so it cannot be part of any new cycle in subsequent DFS calls
        // IDEA: we are exploring the graph in a dfs logic, so we get here only if I (completely) explored all of this node's neighbors and I didn't
        // find any I could explore next (!advanced) -> this means that there is no cycle in the remaining graph that uses the current node, otherwise
        // we would have found it by exploring all nodes that can be reached by the current node
        if (!advanced) {
            in_stack[state.vertex] = false;
            removed_nodes[state.vertex] = true;  // Mark as fully explored
            if (!current_path.empty()) current_path.pop_back();
            if (!current_path_edges.empty()) current_path_edges.pop_back();
            stack.pop_back();
        }
    }
}

/**
 * @brief Find (some) cycles in a directed unweighted graph -> an edge can't be in more than one cycle
 * NOTE! IF A CYCLE IS PRESENT IN THE GRAPH, THIS ALGORITHM WILL RETURN AT LEAST ONE OF THEM (it never happens that it returns no cycle even if
 * there is at least one)
 *
 * @tparam T The type of the edge labels
 * @param graph An adjacency list for the graph -> an edge is a pair (p, q) so that q is in graph[p]
 * @param edge_labels Labels to be associated to each edge -> edge_labels.at({p,q}) = label({p,q})
 * @return std::vector<std::vector<T>> The list of cycles expressed by the labels of the edges that would compose that cycle
 */
template <typename T>
[[nodiscard]] static inline std::vector<std::vector<T>> find_cycles_unweighted(
    const std::vector<std::vector<unsigned int>>& graph, const std::unordered_map<std::pair<unsigned int, unsigned int>, T, pair_hash>& edge_labels) {
    if (graph.empty()) return {};

    std::vector<std::vector<T>> cycles;

    // Track edge usage per (from, to_index)
    std::vector<std::vector<bool>> used_edges(graph.size());
    for (unsigned int from = 0; from < graph.size(); ++from) used_edges[from].resize(graph[from].size(), false);
    // Data structure to prune completely visited nodes -> if a node has been completely visited (and no (new) cycle has been found -> that node won't
    // appear in any cycle)
    std::vector<bool> globally_visited(graph.size(), false);

    for (unsigned int v = 0; v < graph.size(); v++) {
        if (globally_visited[v]) continue;

        bool has_free_edges{false};
        for (unsigned int i = 0; i < graph[v].size(); ++i) {
            if (!used_edges[v][i]) {
                has_free_edges = true;
                break;
            }
        }

        if (has_free_edges) cycle_dfs(graph, edge_labels, v, used_edges, globally_visited, cycles);
    }

    return std::move(cycles);
}

// ##################################################################### //
// ##################### CYCLE DETECTION, WEIGHTED ##################### //
// ##################################################################### //

/**
 * @brief Find the shortest path in a weighted graph, returning both the path and edge indices
 * @param max_edge_weight Maximum allowed weight for any single edge in the path (for pruning)
 */
[[nodiscard]]
static inline std::pair<std::vector<std::pair<unsigned int, unsigned int>>, std::vector<unsigned int>> dijkstra_with_path_info(
    const std::vector<std::vector<unsigned int>>& graph,
    const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& edge_weights, unsigned int source, unsigned int destination,
    const std::vector<std::vector<bool>>& removed_edges, double max_edge_weight = 1.0) {
    const unsigned int n = graph.size();
    std::vector<double> distance(n, std::numeric_limits<double>::infinity());
    // previous[v] = (from, to_idx) -> edge = { from -> graph[from][to_idx] = v }
    std::vector<std::pair<unsigned int, unsigned int>> previous(n, {n, n});

    distance[source] = 0.0;

    // Priority queue of [distance, node] pairs
    std::priority_queue<std::pair<double, unsigned int>, std::vector<std::pair<double, unsigned int>>, std::greater<>> pq;
    pq.emplace(0.0, source);

    while (!pq.empty()) {
        auto [dist_u, u] = pq.top();
        pq.pop();

        // Skip if this is an outdated entry (we found a better path already)
        if (dist_u > distance[u]) continue;

        // Early termination when we reach destination
        if (u == destination) break;

        for (unsigned int i = 0; i < graph[u].size(); ++i) {
            if (removed_edges[u][i]) continue;

            unsigned int v = graph[u][i];
            double weight = edge_weights.at({u, v});

            // Skip edges that are too heavy for our cycle constraint (weight(cycle) < 1)
            if (weight >= max_edge_weight) continue;

            // If we found a better path to v
            if (distance[u] + weight < distance[v]) {
                // Update the distance source -> v
                distance[v] = distance[u] + weight;
                // Store the previous of v -> prev[v] = (u,i) -> edge = { u, graph[u][i] = v }
                previous[v] = {u, i};
                pq.emplace(distance[v], v);
            }
        }
    }

    // No path found
    if (distance[destination] == std::numeric_limits<double>::infinity()) return {{}, {}};

    // Reconstruct path
    std::vector<std::pair<unsigned int, unsigned int>> vertex_path;
    std::vector<unsigned int> edge_indices;

    // Build the reverse path (destination -> source)
    for (unsigned int v = destination; previous[v].first != n; v = previous[v].first) {
        unsigned int prev_vertex = previous[v].first;
        unsigned int edge_idx = previous[v].second;

        vertex_path.emplace_back(prev_vertex, v);
        edge_indices.push_back(edge_idx);
    }

    // Reverse the path (source -> destination)
    std::reverse(vertex_path.begin(), vertex_path.end());
    std::reverse(edge_indices.begin(), edge_indices.end());

    return {vertex_path, edge_indices};
}

/**
 * @brief Find (some) cycles in a directed weighted graph with weight < 1 -> an edge can't be in more than one cycle
 * NOTE! IF SUCH A CYCLE IS PRESENT IN THE GRAPH, THIS ALGORITHM WILL RETURN AT LEAST ONE OF THEM (it never happens that it returns no cycle even if
 * there is at least one)
 *
 * @tparam T The type of the edge labels
 * @param graph An adjacency list for the graph -> an edge is a pair (p, q) so that q is in graph[p]
 * @param edge_labels Labels to be associated to each edge -> edge_labels.at({p,q}) = label({p,q})
 * @param edge_weights Weights to be associated to each edge -> edge_labels.at({p,q}) = label({p,q})
 * @return std::vector<std::vector<T>> The list of cycles expressed by the labels of the edges that would compose that cycle
 */
template <typename T>
[[nodiscard]]
static inline std::vector<std::vector<T>> find_cycles_weighted_lessthan1(
    const std::vector<std::vector<unsigned int>>& graph, const std::unordered_map<std::pair<unsigned int, unsigned int>, T, pair_hash>& edge_labels,
    const std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash>& edge_weights) {
    std::vector<std::vector<T>> cycles;
    if (graph.empty()) return cycles;

    // Track edge usage per (from, to_index)
    std::vector<std::vector<bool>> used_edges(graph.size());
    for (unsigned int from = 0; from < graph.size(); ++from) used_edges[from].resize(graph[from].size(), false);

    for (unsigned int v = 0; v < graph.size(); v++) {
        // Skip if vertex has no unused outgoing edges
        bool has_unused_edges = false;
        for (unsigned int i = 0; i < graph[v].size(); ++i) {
            if (!used_edges[v][i]) {
                has_unused_edges = true;
                break;
            }
        }
        if (!has_unused_edges) continue;

        for (unsigned int w_idx = 0; w_idx < graph[v].size(); ++w_idx) {
            if (used_edges[v][w_idx]) continue;

            // For each unused edge outgoing

            unsigned int w = graph[v][w_idx];
            double vw_weight = edge_weights.at({v, w});

            // Early pruning: if this edge alone has weight >= 1, skip
            if (vw_weight >= 1.0) {
                used_edges[v][w_idx] = true;  // Mark as used since it can't be in any valid cycle
                continue;
            }

            // Find shortest path from w back to v, with edge weight constraint
            double max_allowed_edge_weight = 1.0 - vw_weight;
            const auto& [path_vertices, path_edges_indices] = dijkstra_with_path_info(graph, edge_weights, w, v, used_edges, max_allowed_edge_weight);
            if (path_vertices.empty()) continue;

            // Calculate total cycle weight
            double cycle_weight = vw_weight;
            for (const auto& [from, to] : path_vertices) cycle_weight += edge_weights.at({from, to});

            // Skip if cycle weight >= 1 (we know that there can't be another cycle using {v, w} with lower weight, since the path {w -> v} was the
            // shortest one we could find, using unused edges)
            if (cycle_weight >= 1.0 - HPLUS_EPSILON) continue;

            // Build the cycle from edge labels
            std::vector<T> cycle;
            cycle.reserve(path_vertices.size() + 1);  // Reserve space for efficiency

            // Add the path edges (w -> ... -> v)
            for (const auto& [from, to] : path_vertices) cycle.push_back(edge_labels.at({from, to}));

            // Add the closing edge (v -> w)
            cycle.push_back(edge_labels.at({v, w}));

            // Mark all edges in the cycle as used
            used_edges[v][w_idx] = true;
            unsigned int current = w;
            for (unsigned int edge_idx : path_edges_indices) {
                used_edges[current][edge_idx] = true;
                current = graph[current][edge_idx];
            }

            cycles.emplace_back(std::move(cycle));
        }
    }

    return cycles;
}

// ##################################################################### //
// ############################## MAX FLOW ############################# //
// ##################################################################### //

typedef struct {
    unsigned int to;   // Destination node
    unsigned int rev;  // Index of reverse edge in the destination's adjacency list
    double c;          // Remaining capacity of the edge
    bool is_reverse;
} network_edge;

[[nodiscard]]
static inline bool max_flow_bfs(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int n,
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

[[nodiscard]]
static inline double max_flow_dfs(std::vector<std::vector<network_edge>>& graph, unsigned int v, unsigned int t, double f, std::vector<int>& level,
                                  std::vector<int>& iter) {
    if (v == t) return f;  // Reached sink, return the flow

    // iter[v] remembers which edge to start from (optimization to avoid revisiting dead ends)
    for (int& i = iter[v]; i < static_cast<int>(graph[v].size()); i++) {
        auto& e = graph[v][i];

        // Only use edges with capacity that go to next level
        if (e.c > HPLUS_EPSILON && level[v] < level[e.to]) {
            double d{max_flow_dfs(graph, e.to, t, std::min(f, e.c), level, iter)};
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

[[nodiscard]]
static inline double compute_max_flow(std::vector<std::vector<network_edge>>& graph, unsigned int source, unsigned int sink) {
    double flow{0};
    const unsigned int n{static_cast<unsigned int>(graph.size())};
    std::vector<int> level(n), iter(n);

    // Repeat while there's an augmenting path from source to sink
    while (max_flow_bfs(graph, source, sink, n, level)) {
        iter.assign(n, 0);
        double f;

        // Keep finding blocking flows until no more paths exist in this level graph
        while ((f = max_flow_dfs(graph, source, sink, std::numeric_limits<double>::infinity(), level, iter)) > HPLUS_EPSILON) flow += f;
    }
    return flow;
}

[[nodiscard]] static inline binary_set get_min_cut_lpartition(const std::vector<std::vector<network_edge>>& graph, unsigned int source) {
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
// ############### INCREMENTAL MAX FLOW ON CHANGING GRAPH ############## //
// ##################################################################### //
// TODO: Optimize
static double dfs_remove_flow(std::vector<std::vector<network_edge>>& graph, unsigned int u, const unsigned int target, double flow_to_remove,
                              binary_set& visited) {
    if (u == target) return flow_to_remove;

    visited.add(u);

    for (auto& edge : graph[u]) {
        if (visited[edge.to] || edge.is_reverse) continue;

        // Check available flow on this edge (stored in reverse edge capacity)
        double available_flow = graph[edge.to][edge.rev].c;

        if (available_flow > HPLUS_EPSILON) {  // Has flow
            // Recursively try to push through to target
            double pushed = dfs_remove_flow(graph, edge.to, target, std::min(flow_to_remove, available_flow), visited);

            if (pushed > 0) {
                // Successfully found path - update edges
                graph[edge.to][edge.rev].c -= pushed;  // Reduce flow
                edge.c += pushed;                      // Increase residual capacity
                return pushed;
            }
        }
    }

    return 0;  // No path found
}

// TODO: Optimize
static inline void flow_removal(std::vector<std::vector<network_edge>>& graph, unsigned int from, unsigned int to, const double flow_to_remove) {
    double remaining = flow_to_remove;

    // Keep finding paths and removing flow until we've removed enough
    while (remaining > HPLUS_EPSILON) {
        binary_set visited(graph.size());
        double removed = dfs_remove_flow(graph, from, to, remaining, visited);

        if (removed < HPLUS_EPSILON) {
            // No more paths with flow available
            // This shouldn't happen in a valid flow network
            break;
        }

        remaining -= removed;
    }
}

// TODO: Implement incremental computation with these methods
// Refer to "Incremental Maximum Flow Computation on Evolving Networks" by Greco, Molinaro, Pulice and Quintana
// /**
//  * Augmenting Shortest Path (ASP) for Incremental Max Flow
//  *
//  * Performs BFS to find the shortest path from x to y with positive residual capacity.
//  * Returns the maximum flow that can be pushed along this path.
//  *
//  * Parameters:
//  * - graph: adjacency list with network_edge structures
//  * - x: start vertex
//  * - y: end vertex
//  * - prev_vertex: output array storing predecessor vertex
//  * - prev_edge_idx: output array storing which edge index was used to reach each vertex
//  * - flow_capacity: output array storing max flow that can reach each vertex
//  *
//  * Returns: maximum flow that can be pushed from x to y (0 if no path exists)
//  */
// [[nodiscard]]
// static inline double incremental_asp(const std::vector<std::vector<network_edge>>& graph, unsigned int x, unsigned int y,
//                                      std::vector<int>& prev_vertex, std::vector<int>& prev_edge_idx, std::vector<double>& flow_capacity) {
//     const unsigned int n = graph.size();

//     // Initialize arrays
//     prev_vertex.assign(n, -1);
//     prev_edge_idx.assign(n, -1);
//     flow_capacity.assign(n, -1.0);

//     flow_capacity[x] = std::numeric_limits<double>::infinity();

//     if (x == y) {
//         return std::numeric_limits<double>::infinity();
//     }

//     std::queue<unsigned int> q;
//     q.push(x);

//     while (!q.empty()) {
//         unsigned int u = q.front();
//         q.pop();

//         for (unsigned int edge_idx = 0; edge_idx < graph[u].size(); edge_idx++) {
//             const auto& [v, rev, res_flow] = graph[u][edge_idx];

//             // Skip if already visited or no residual capacity
//             if (flow_capacity[v] >= 0 || res_flow <= HPLUS_EPSILON) continue;

//             // Update flow capacity and predecessor
//             flow_capacity[v] = std::min(flow_capacity[u], res_flow);
//             prev_vertex[v] = u;
//             prev_edge_idx[v] = edge_idx;

//             // If we reached destination, return immediately
//             if (v == y) {
//                 return flow_capacity[y];
//             }

//             q.push(v);
//         }
//     }

//     return 0.0;  // No path found
// }

// /**
//  * Update Flow along a path
//  *
//  * Pushes deltaF units of flow from x to y following the predecessor chain.
//  * Updates the residual capacities in the graph.
//  *
//  * Parameters:
//  * - graph: adjacency list (modified in place)
//  * - deltaF: amount of flow to push
//  * - x: start vertex
//  * - y: end vertex
//  * - prev_vertex: predecessor vertex array
//  * - prev_edge_idx: predecessor edge index array
//  */
// static inline void incremental_update_flow(std::vector<std::vector<network_edge>>& graph, double deltaF, unsigned int x, unsigned int y,
//                                            const std::vector<int>& prev_vertex, const std::vector<int>& prev_edge_idx) {
//     unsigned int b = y;

//     // Traverse path backwards from y to x
//     while (b != x) {
//         int a = prev_vertex[b];
//         int edge_idx = prev_edge_idx[b];

//         // Reduce forward edge capacity
//         graph[a][edge_idx].c -= deltaF;

//         // Increase reverse edge capacity
//         unsigned int rev_idx = graph[a][edge_idx].rev;
//         graph[b][rev_idx].c += deltaF;

//         b = a;
//     }
// }

// /**
//  * Reset Exceeding Flow (REF)
//  *
//  * Sends excess flow back from vertex x toward the sink t.
//  * This is used during edge deletion to handle excess flow.
//  *
//  * Parameters:
//  * - graph: adjacency list (modified in place)
//  * - x: vertex with excess
//  * - t: sink vertex
//  * - excess: amount of excess flow to send back
//  */
// static inline void incremental_reset_exceeding_flow(std::vector<std::vector<network_edge>>& graph, unsigned int x, unsigned int t, double excess) {
//     if (x == t) return;

//     const unsigned int n = graph.size();

//     while (excess > HPLUS_EPSILON) {
//         std::vector<int> prev_vertex(n, -1);
//         std::vector<int> prev_edge_idx(n, -1);
//         std::vector<double> flow_capacity(n, -1.0);

//         flow_capacity[x] = excess;

//         std::queue<unsigned int> q;
//         q.push(x);
//         bool found_path = false;

//         // BFS to find path from x to t along edges with positive reverse flow
//         while (!q.empty() && !found_path) {
//             unsigned int u = q.front();
//             q.pop();

//             for (int edge_idx = 0; edge_idx < static_cast<int>(graph[u].size()); edge_idx++) {
//                 const auto& [v, rev, res_flow] = graph[u][edge_idx];

//                 if (flow_capacity[v] >= 0) continue;

//                 // Look at the reverse edge v see if there's flow v reduce
//                 unsigned int rev_idx = rev;
//                 double reverse_flow = graph[v][rev_idx].c;

//                 if (reverse_flow <= HPLUS_EPSILON) continue;

//                 flow_capacity[v] = std::min(flow_capacity[u], reverse_flow);
//                 prev_vertex[v] = u;
//                 prev_edge_idx[v] = edge_idx;

//                 if (v == t) {
//                     found_path = true;
//                     break;
//                 }

//                 q.push(v);
//             }
//         }

//         if (!found_path) break;

//         // Reduce flow along the path
//         unsigned int b = t;
//         double flow_to_reduce = flow_capacity[t];

//         while (b != x) {
//             unsigned int a = prev_vertex[b];
//             int edge_idx = prev_edge_idx[b];

//             // Increase forward capacity (reducing flow)
//             graph[a][edge_idx].c += flow_to_reduce;

//             // Decrease reverse capacity
//             unsigned int rev_idx = graph[a][edge_idx].rev;
//             graph[b][rev_idx].c -= flow_to_reduce;

//             b = a;
//         }

//         excess -= flow_to_reduce;
//     }
// }

// /**
//  * Edge Insertion Maintenance (EIM)
//  *
//  * Incrementally updates maximum flow after inserting edge (a,b) with capacity w
//  * or increasing capacity of existing edge.
//  *
//  * This is Algorithm 1 from the paper.
//  *
//  * Parameters:
//  * - graph: adjacency list (modified in place)
//  * - s: source vertex
//  * - t: sink vertex
//  * - a, b: edge endpoints
//  * - w: capacity to add (positive value)
//  */
// static inline void incremental_edge_insertion(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int a,
//                                               unsigned int b, double w) {
//     if (w <= HPLUS_EPSILON) return;

//     // Find edge (a,b) if it exists
//     int edge_ab_idx = -1;
//     for (int i = 0; i < static_cast<int>(graph[a].size()); i++) {
//         if (graph[a][i].to == b) {
//             edge_ab_idx = i;
//             break;
//         }
//     }

//     // Check if edge is saturated or doesn't exist
//     double residual = 0.0;
//     if (edge_ab_idx >= 0) {
//         residual = graph[a][edge_ab_idx].c;
//     }

//     // Only proceed if edge is new or saturated
//     if (residual > HPLUS_EPSILON) return;

//     // Add or update edge
//     if (edge_ab_idx < 0) {
//         // Add new edge (a,b) and reverse edge (b,a)
//         unsigned int rev_ba_idx = graph[b].size();
//         unsigned int rev_ab_idx = graph[a].size();

//         graph[a].push_back({b, rev_ba_idx, w});
//         graph[b].push_back({a, rev_ab_idx, 0.0});

//         edge_ab_idx = rev_ab_idx;
//     } else {
//         // Update existing edge capacity
//         graph[a][edge_ab_idx].c += w;
//     }

//     const unsigned int n = graph.size();
//     std::vector<int> prev_sa_v(n), prev_sa_e(n);
//     std::vector<int> prev_bt_v(n), prev_bt_e(n);
//     std::vector<double> flow_sa(n), flow_bt(n);

//     // Find path from source to a
//     double deltaF_sa = incremental_asp(graph, s, a, prev_sa_v, prev_sa_e, flow_sa);

//     if (deltaF_sa <= HPLUS_EPSILON) return;

//     // Find path from b to sink
//     double deltaF_bt = incremental_asp(graph, b, t, prev_bt_v, prev_bt_e, flow_bt);

//     // Push flow while augmenting paths exist
//     while (graph[a][edge_ab_idx].c > HPLUS_EPSILON && deltaF_sa > HPLUS_EPSILON && deltaF_bt > HPLUS_EPSILON) {
//         double deltaF = std::min({graph[a][edge_ab_idx].c, deltaF_sa, deltaF_bt});

//         unsigned int rev_ba_idx = graph[a][edge_ab_idx].rev;
//         // Update flow on edge (a,b)
//         graph[a][edge_ab_idx].c -= deltaF;
//         graph[b][rev_ba_idx].c += deltaF;

//         // Update flow on paths
//         incremental_update_flow(graph, deltaF, s, a, prev_sa_v, prev_sa_e);
//         incremental_update_flow(graph, deltaF, b, t, prev_bt_v, prev_bt_e);

//         deltaF_sa -= deltaF;
//         deltaF_bt -= deltaF;

//         // Find new paths if needed
//         if (graph[a][edge_ab_idx].c > HPLUS_EPSILON) {
//             if (deltaF_sa > HPLUS_EPSILON) {
//                 deltaF_bt = incremental_asp(graph, b, t, prev_bt_v, prev_bt_e, flow_bt);
//             } else if (deltaF_bt > HPLUS_EPSILON) {
//                 deltaF_sa = incremental_asp(graph, s, a, prev_sa_v, prev_sa_e, flow_sa);
//             } else {
//                 deltaF_sa = incremental_asp(graph, s, a, prev_sa_v, prev_sa_e, flow_sa);
//                 if (deltaF_sa > HPLUS_EPSILON) {
//                     deltaF_bt = incremental_asp(graph, b, t, prev_bt_v, prev_bt_e, flow_bt);
//                 }
//             }
//         }
//     }
// }

// /**
//  * Edge Deletion Maintenance (EDM)
//  *
//  * Incrementally updates maximum flow after deleting edge (a,b) or decreasing
//  * its capacity by |w| units.
//  *
//  * This is Algorithm 2 from the paper.
//  *
//  * Parameters:
//  * - graph: adjacency list (modified in place)
//  * - s: source vertex
//  * - t: sink vertex
//  * - a, b: edge endpoints
//  * - w: capacity to remove (positive value representing amount to decrease)
//  */
// static inline void incremental_edge_deletion(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int a,
//                                              unsigned int b, double w) {
//     if (w <= HPLUS_EPSILON) return;

//     // Find edge (a,b)
//     int edge_ab_idx = -1;
//     for (int i = 0; i < static_cast<int>(graph[a].size()); i++) {
//         if (graph[a][i].to == b) {
//             edge_ab_idx = i;
//             break;
//         }
//     }

//     if (edge_ab_idx < 0) return;  // Edge doesn't exist

//     // Calculate current flow on edge (using reverse edge capacity)
//     unsigned int rev_ba_idx = graph[a][edge_ab_idx].rev;
//     double current_flow = graph[b][rev_ba_idx].c;

//     // Calculate new capacity
//     double old_capacity = graph[a][edge_ab_idx].c + current_flow;
//     double new_capacity =
//         std::max(0.0, old_capacity - w);  // here is - : on the paper uses + but because w is negative... I use a positive w, so I need to use the
//         -

//     // Calculate excess that needs to be rerouted
//     double excess = current_flow - new_capacity;

//     if (excess <= HPLUS_EPSILON) {
//         // Just update capacity, no rerouting needed
//         graph[a][edge_ab_idx].c = new_capacity - current_flow;
//         return;
//     }

//     // Update capacity
//     graph[a][edge_ab_idx].c = 0;
//     graph[b][rev_ba_idx].c = new_capacity;

//     // Phase 1: Send excess flow back from t to b
//     incremental_reset_exceeding_flow(graph, b, t, excess);

//     const unsigned int n = graph.size();
//     std::vector<int> prev_at_v(n), prev_at_e(n);
//     std::vector<double> flow_at(n);

//     // Phase 2: Try to push excess from a to t through alternative paths
//     double deltaF_at = incremental_asp(graph, a, t, prev_at_v, prev_at_e, flow_at);
//     deltaF_at = std::min(deltaF_at, excess);

//     while (excess > HPLUS_EPSILON && deltaF_at > HPLUS_EPSILON) {
//         excess -= deltaF_at;
//         incremental_update_flow(graph, deltaF_at, a, t, prev_at_v, prev_at_e);

//         if (excess > HPLUS_EPSILON) {
//             deltaF_at = incremental_asp(graph, a, t, prev_at_v, prev_at_e, flow_at);
//             deltaF_at = std::min(deltaF_at, excess);
//         }
//     }

//     // Phase 3: If still excess, push back to source
//     if (excess > HPLUS_EPSILON) {
//         std::vector<int> prev_as_v(n), prev_as_e(n);
//         std::vector<double> flow_as(n);

//         double deltaF_as = incremental_asp(graph, a, s, prev_as_v, prev_as_e, flow_as);
//         deltaF_as = std::min(deltaF_as, excess);

//         while (excess > HPLUS_EPSILON && deltaF_as > HPLUS_EPSILON) {
//             excess -= deltaF_as;
//             incremental_update_flow(graph, deltaF_as, a, s, prev_as_v, prev_as_e);

//             if (excess > HPLUS_EPSILON) {
//                 deltaF_as = incremental_asp(graph, a, s, prev_as_v, prev_as_e, flow_as);
//                 deltaF_as = std::min(deltaF_as, excess);
//             }
//         }
//     }
// }

#endif