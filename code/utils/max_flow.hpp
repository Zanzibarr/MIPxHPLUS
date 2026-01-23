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

/** @brief DFS algorithm to remove flow from a u -> target path (if any) */
[[nodiscard]]
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

// Refer to "Incremental Maximum Flow Computation on Evolving Networks" by Greco, Molinaro, Pulice and Quintana
/** @brief Performs BFS to find the shortest path from x to y with positive residual capacity. Returns the maximum flow that can be pushed along this
 * path.
 */
[[nodiscard]]
static inline double incremental_asp(const std::vector<std::vector<network_edge>>& graph, unsigned int x, unsigned int y,
                                     std::vector<int>& prev_vertex, std::vector<int>& prev_edge_idx, std::vector<double>& flow_capacity,
                                     bool reverse_path = false) {
    const unsigned int n = graph.size();

    // Initialize arrays
    prev_vertex.assign(n, -1);
    prev_edge_idx.assign(n, -1);
    flow_capacity.assign(n, -1.0);

    flow_capacity[x] = std::numeric_limits<double>::infinity();

    if (x == y) {
        return std::numeric_limits<double>::infinity();
    }

    std::queue<unsigned int> q;
    q.push(x);

    while (!q.empty()) {
        unsigned int u = q.front();
        q.pop();

        for (unsigned int edge_idx = 0; edge_idx < graph[u].size(); edge_idx++) {
            const auto& [v, rev, res_flow, is_rev] = graph[u][edge_idx];

            // If we are looking for a reverse path, we are only interested in reverse edges... the residual capacity of the reverse
            // edges is the flow used in that path, so we are succesfully removing existing flow, not creating additional flow that
            // will create loops
            if (reverse_path && !is_rev) continue;

            // Skip if already visited or no residual capacity
            if (flow_capacity[v] >= 0 || res_flow <= HPLUS_EPSILON) continue;

            // Update flow capacity and predecessor
            flow_capacity[v] = std::min(flow_capacity[u], res_flow);
            prev_vertex[v] = u;
            prev_edge_idx[v] = edge_idx;

            // If we reached destination, return immediately
            if (v == y) {
                return flow_capacity[y];
            }

            q.push(v);
        }
    }

    return 0.0;  // No path found
}

// Refer to "Incremental Maximum Flow Computation on Evolving Networks" by Greco, Molinaro, Pulice and Quintana
/** @brief Update Flow along a path according to the predecessor chain, computed in the incremental asp */
static inline void incremental_update_flow(std::vector<std::vector<network_edge>>& graph, double deltaF, unsigned int x, unsigned int y,
                                           const std::vector<int>& prev_vertex, const std::vector<int>& prev_edge_idx) {
    unsigned int b = y;

    // Traverse path backwards from y to x
    while (b != x) {
        int a = prev_vertex[b];
        int edge_idx = prev_edge_idx[b];

        // Reduce forward edge capacity
        graph[a][edge_idx].c -= deltaF;

        // Increase reverse edge capacity
        unsigned int rev_idx = graph[a][edge_idx].rev;
        graph[b][rev_idx].c += deltaF;

        b = a;
    }
}

// Refer to "Incremental Maximum Flow Computation on Evolving Networks" by Greco, Molinaro, Pulice and Quintana
/** @brief Sends excess flow back from vertex x toward the sink t. This is used during edge deletion to handle excess flow. */
static inline void incremental_reset_exceeding_flow(std::vector<std::vector<network_edge>>& graph, unsigned int x, unsigned int t, double excess) {
    if (x == t) return;

    const unsigned int n = graph.size();

    while (excess > HPLUS_EPSILON) {
        std::vector<int> prev_vertex(n, -1);
        std::vector<int> prev_edge_idx(n, -1);
        std::vector<double> flow_capacity(n, -1.0);

        flow_capacity[x] = excess;

        std::queue<unsigned int> q;
        q.push(x);
        bool found_path = false;

        // BFS to find path from x to t along edges with positive reverse flow
        while (!q.empty() && !found_path) {
            unsigned int u = q.front();
            q.pop();

            for (int edge_idx = 0; edge_idx < static_cast<int>(graph[u].size()); edge_idx++) {
                const auto& [v, rev, res_flow, is_rev] = graph[u][edge_idx];

                if (flow_capacity[v] >= 0) continue;

                // Look at the reverse edge v see if there's flow v reduce
                unsigned int rev_idx = rev;
                double reverse_flow = graph[v][rev_idx].c;

                if (reverse_flow <= HPLUS_EPSILON) continue;

                flow_capacity[v] = std::min(flow_capacity[u], reverse_flow);
                prev_vertex[v] = u;
                prev_edge_idx[v] = edge_idx;

                if (v == t) {
                    found_path = true;
                    break;
                }

                q.push(v);
            }
        }

        if (!found_path) break;

        // Reduce flow along the path
        unsigned int b = t;
        double flow_to_reduce = flow_capacity[t];

        while (b != x) {
            unsigned int a = prev_vertex[b];
            int edge_idx = prev_edge_idx[b];

            // Increase forward capacity (reducing flow)
            graph[a][edge_idx].c += flow_to_reduce;

            // Decrease reverse capacity
            unsigned int rev_idx = graph[a][edge_idx].rev;
            graph[b][rev_idx].c -= flow_to_reduce;

            b = a;
        }

        excess -= flow_to_reduce;
    }
}

// Refer to "Incremental Maximum Flow Computation on Evolving Networks" by Greco, Molinaro, Pulice and Quintana
/**
 * @brief Edge Insertion Maintenance (EIM)
 *
 * Incrementally updates maximum flow after inserting edge (a,b) with capacity w or increasing capacity of existing edge.
 *
 * This is Algorithm 1 from the paper.
 *
 * @param graph adjacency list (modified in place)
 * @param s source vertex
 * @param t sink vertex
 * @param a, b edge endpoints
 * @param w capacity to add (positive value)
 */
static inline void incremental_edge_insertion(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int a,
                                              unsigned int b, double w) {
    if (w <= HPLUS_EPSILON) return;

    // Find edge (a,b) if it exists
    int edge_ab_idx = -1;
    for (int i = 0; i < static_cast<int>(graph[a].size()); i++) {
        if (graph[a][i].to == b) {
            edge_ab_idx = i;
            break;
        }
    }

    // Check if edge is saturated or doesn't exist
    double residual = 0.0;
    if (edge_ab_idx >= 0) {
        residual = graph[a][edge_ab_idx].c;
    }

    // Only proceed if edge is new or saturated
    if (residual > HPLUS_EPSILON) return;

    // Add or update edge
    if (edge_ab_idx < 0) {
        // Add new edge (a,b) and reverse edge (b,a)
        unsigned int rev_ba_idx = graph[b].size();
        unsigned int rev_ab_idx = graph[a].size();

        graph[a].push_back({b, rev_ba_idx, w, false});
        graph[b].push_back({a, rev_ab_idx, 0.0, true});

        edge_ab_idx = rev_ab_idx;
    } else {
        // Update existing edge capacity
        graph[a][edge_ab_idx].c += w;
    }

    const unsigned int n = graph.size();
    std::vector<int> prev_sa_v(n), prev_sa_e(n);
    std::vector<int> prev_bt_v(n), prev_bt_e(n);
    std::vector<double> flow_sa(n), flow_bt(n);

    // Find path from source to a
    double deltaF_sa = incremental_asp(graph, s, a, prev_sa_v, prev_sa_e, flow_sa);

    if (deltaF_sa <= HPLUS_EPSILON) return;

    // Find path from b to sink
    double deltaF_bt = incremental_asp(graph, b, t, prev_bt_v, prev_bt_e, flow_bt);

    // Push flow while augmenting paths exist
    while (graph[a][edge_ab_idx].c > HPLUS_EPSILON && deltaF_sa > HPLUS_EPSILON && deltaF_bt > HPLUS_EPSILON) {
        double deltaF = std::min({graph[a][edge_ab_idx].c, deltaF_sa, deltaF_bt});

        unsigned int rev_ba_idx = graph[a][edge_ab_idx].rev;
        // Update flow on edge (a,b)
        graph[a][edge_ab_idx].c -= deltaF;
        graph[b][rev_ba_idx].c += deltaF;

        // Update flow on paths
        incremental_update_flow(graph, deltaF, s, a, prev_sa_v, prev_sa_e);
        incremental_update_flow(graph, deltaF, b, t, prev_bt_v, prev_bt_e);

        deltaF_sa -= deltaF;
        deltaF_bt -= deltaF;

        // Find new paths if needed
        if (graph[a][edge_ab_idx].c > HPLUS_EPSILON) {
            if (deltaF_sa > HPLUS_EPSILON) {
                deltaF_bt = incremental_asp(graph, b, t, prev_bt_v, prev_bt_e, flow_bt);
            } else if (deltaF_bt > HPLUS_EPSILON) {
                deltaF_sa = incremental_asp(graph, s, a, prev_sa_v, prev_sa_e, flow_sa);
            } else {
                deltaF_sa = incremental_asp(graph, s, a, prev_sa_v, prev_sa_e, flow_sa);
                if (deltaF_sa > HPLUS_EPSILON) {
                    deltaF_bt = incremental_asp(graph, b, t, prev_bt_v, prev_bt_e, flow_bt);
                }
            }
        }
    }
}

// Refer to "Incremental Maximum Flow Computation on Evolving Networks" by Greco, Molinaro, Pulice and Quintana
/**
 * @brief Edge Deletion Maintenance (EDM)
 *
 * Incrementally updates maximum flow after deleting edge (a,b) or decreasing its capacity by w units.
 *
 * This is Algorithm 2 from the paper.
 *
 * @param graph adjacency list (modified in place)
 * @param s source vertex
 * @param t sink vertex
 * @param a, b edge endpoints
 * @param w capacity to remove (positive value representing amount to decrease)
 */
static inline void incremental_edge_deletion(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int a,
                                             unsigned int b, double w) {
    if (w <= HPLUS_EPSILON) return;

    // Find edge (a,b)
    int edge_ab_idx = -1;
    for (int i = 0; i < static_cast<int>(graph[a].size()); i++) {
        if (graph[a][i].to == b) {
            edge_ab_idx = i;
            break;
        }
    }

    if (edge_ab_idx < 0) return;  // Edge doesn't exist

    // Calculate current flow on edge (using reverse edge capacity)
    unsigned int rev_ba_idx = graph[a][edge_ab_idx].rev;
    double current_flow = graph[b][rev_ba_idx].c;

    // Calculate new capacity
    double old_capacity = graph[a][edge_ab_idx].c + current_flow;
    double new_capacity =
        std::max(0.0, old_capacity - w);  // here is - : on the paper uses + but because w is negative... I use a positive w, so I need to use the -

    // Calculate excess that needs to be rerouted
    double excess = current_flow - new_capacity;

    if (excess <= HPLUS_EPSILON) {
        // Just update capacity, no rerouting needed
        graph[a][edge_ab_idx].c = new_capacity - current_flow;
        return;
    }

    // Update capacity
    graph[a][edge_ab_idx].c = 0;
    graph[b][rev_ba_idx].c = new_capacity;

    // Phase 1: Send excess flow back from t to b
    incremental_reset_exceeding_flow(graph, b, t, excess);

    const unsigned int n = graph.size();
    std::vector<int> prev_at_v(n), prev_at_e(n);
    std::vector<double> flow_at(n);

    // Phase 2: Try to push excess from a to t through alternative paths
    double deltaF_at = incremental_asp(graph, a, t, prev_at_v, prev_at_e, flow_at);
    deltaF_at = std::min(deltaF_at, excess);

    while (excess > HPLUS_EPSILON && deltaF_at > HPLUS_EPSILON) {
        excess -= deltaF_at;
        incremental_update_flow(graph, deltaF_at, a, t, prev_at_v, prev_at_e);

        if (excess > HPLUS_EPSILON) {
            deltaF_at = incremental_asp(graph, a, t, prev_at_v, prev_at_e, flow_at);
            deltaF_at = std::min(deltaF_at, excess);
        }
    }

    // Phase 3: If still excess, push back to source
    if (excess > HPLUS_EPSILON) {
        std::vector<int> prev_as_v(n), prev_as_e(n);
        std::vector<double> flow_as(n);
        double deltaF_as = incremental_asp(graph, a, s, prev_as_v, prev_as_e, flow_as, true);
        deltaF_as = std::min(deltaF_as, excess);
        while (excess > HPLUS_EPSILON && deltaF_as > HPLUS_EPSILON) {
            excess -= deltaF_as;
            incremental_update_flow(graph, deltaF_as, a, s, prev_as_v, prev_as_e);

            if (excess > HPLUS_EPSILON) {
                deltaF_as = incremental_asp(graph, a, s, prev_as_v, prev_as_e, flow_as, true);
                deltaF_as = std::min(deltaF_as, excess);
            }
        }
    }
}

#endif