/**
 * Utility functions used in various parts of this project
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_UTILS_ALGORITHMS_HPP
#define HPLUS_UTILS_ALGORITHMS_HPP

#include <stdlib.h>  // size_t

#include <algorithm>  // std::lower_bound
#include <numeric>    //std::iota
#include <queue>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

struct TarjanState {
    std::vector<int> index;
    std::vector<int> lowlink;
    std::vector<bool> on_stack;
    std::stack<unsigned int> stack;
    std::vector<std::vector<unsigned int>> sccs;
    int current_index = 0;
};

static inline void tarjan_dfs(unsigned int start, const std::vector<std::vector<unsigned int>>& graph, TarjanState& state) {
    struct Frame {
        unsigned int v;
        size_t edge_index;
    };

    std::stack<Frame> call_stack;
    call_stack.push({start, 0});

    while (!call_stack.empty()) {
        Frame& frame = call_stack.top();
        unsigned int v = frame.v;

        // First time we see this node
        if (state.index[v] == -1) {
            state.index[v] = state.current_index;
            state.lowlink[v] = state.current_index;
            state.current_index++;
            state.stack.push(v);
            state.on_stack[v] = true;
        }

        // Process neighbors one by one
        auto& neighbors = graph[v];
        if (frame.edge_index < neighbors.size()) {
            unsigned int w = neighbors[frame.edge_index];
            frame.edge_index++;

            if (state.index[w] == -1) {
                // Recurse on w
                call_stack.push({w, 0});
            } else if (state.on_stack[w]) {
                state.lowlink[v] = std::min(state.lowlink[v], state.index[w]);
            }
        } else {
            // All neighbors are processed, now backtrack
            call_stack.pop();

            if (!call_stack.empty()) {
                unsigned int parent = call_stack.top().v;
                state.lowlink[parent] = std::min(state.lowlink[parent], state.lowlink[v]);
            }

            // If v is root of an SCC
            if (state.lowlink[v] == state.index[v]) {
                std::vector<unsigned int> scc;
                unsigned int w;
                do {
                    w = state.stack.top();
                    state.stack.pop();
                    state.on_stack[w] = false;
                    scc.push_back(w);
                } while (w != v);
                state.sccs.push_back(std::move(scc));
            }
        }
    }
}

[[nodiscard]]
static inline std::vector<std::vector<unsigned int>> find_strongly_connected_components(const std::vector<std::vector<unsigned int>>& graph) {
    TarjanState state;
    state.index.assign(graph.size(), -1);
    state.lowlink.assign(graph.size(), -1);
    state.on_stack.assign(graph.size(), false);

    for (unsigned int v = 0; v < graph.size(); ++v) {
        if (state.index[v] == -1) {
            tarjan_dfs(v, graph, state);
        }
    }

    return state.sccs;
}

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
 * @brief Find a cycle within a single SCC, skipping removed edges and nodes
 */
template <typename T>
static inline void cycle_dfs_in_scc(const std::vector<std::vector<unsigned int>>& graph,
                                    const std::unordered_map<std::pair<unsigned int, unsigned int>, T, pair_hash>& edge_labels,
                                    const std::unordered_set<unsigned int>& scc_set, unsigned int start_vertex,
                                    std::vector<std::vector<bool>>& removed_edges, std::vector<bool>& removed_nodes,
                                    std::vector<std::vector<T>>& cycles) {
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

            // Only consider edges within the SCC
            if (scc_set.find(neighbor) == scc_set.end()) continue;

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
 * NOTE! IF A CYCLE IS PRESENT IN THE GRAPH, THIS ALGORITHM WILL RETURN AT LEAST ONE OF THEM (it never happens that it returns no cycle even if there
 * is at least one)
 *
 * @tparam T The type of the edge labels
 * @param graph An adjacency list for the graph -> an edge is a pair (p, q) so that q is in graph[p]
 * @param edge_labels Labels to be associated to each edge -> edge_labels.at({p,q}) = label({p,q})
 * @return std::vector<std::vector<T>> The list of cycles expressed by the labels of the edges that would compose that cycle
 */
template <typename T>
[[nodiscard]]
static inline std::vector<std::vector<T>> find_cycles_unweighted(
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

        bool has_unused_edges = false;
        for (unsigned int i = 0; i < graph[v].size(); ++i) {
            if (!used_edges[v][i]) {
                has_unused_edges = true;
                break;
            }
        }
        if (!has_unused_edges) continue;

        cycle_dfs(graph, edge_labels, v, used_edges, globally_visited, cycles);
    }

    return cycles;
}

/**
 * @brief Find (some) cycles in a directed unweighted graph -> an edge can't be in more than one cycle
 * This is a variant of the algorithm above, where we first split the graph in Strongly Connected Components (SCCs)
 * NOTE! IF A CYCLE IS PRESENT IN THE GRAPH, THIS ALGORITHM WILL RETURN AT LEAST ONE OF THEM (it never happens that it returns no cycle even if there
 * is at least one)
 *
 * @tparam T The type of the edge labels
 * @param graph An adjacency list for the graph -> an edge is a pair (p, q) so that q is in graph[p]
 * @param edge_labels Labels to be associated to each edge -> edge_labels.at({p,q}) = label({p,q})
 * @return std::vector<std::vector<T>> The list of cycles expressed by the labels of the edges that would compose that cycle
 */
template <typename T>
[[nodiscard]]
static inline std::vector<std::vector<T>> find_cycles_unweighted_SCC(
    const std::vector<std::vector<unsigned int>>& graph, const std::unordered_map<std::pair<unsigned int, unsigned int>, T, pair_hash>& edge_labels) {
    if (graph.empty()) return {};

    // Step 1: Find all strongly connected components using Tarjan's algorithm - O(V+E)
    auto sccs = find_strongly_connected_components(graph);

    std::vector<std::vector<T>> cycles;

    // Track edge usage per (from, to_index)
    std::vector<std::vector<bool>> used_edges(graph.size());
    for (unsigned int from = 0; from < graph.size(); ++from) used_edges[from].resize(graph[from].size(), false);

    // Data structure to prune completely visited nodes -> if a node has been completely visited (and no (new) cycle has been found -> that node won't
    // appear in any cycle)
    std::vector<bool> globally_visited(graph.size(), false);

    // Step 2: Process each non-trivial SCC
    for (const auto& scc : sccs) {
        // Skip trivial SCCs (single nodes with no self-loops)
        if (scc.size() <= 1) continue;

        // Create set for O(1) membership checking
        std::unordered_set<unsigned int> scc_set(scc.begin(), scc.end());

        // Search for cycles within this SCC
        for (unsigned int v : scc) {
            if (globally_visited[v]) continue;

            // Check if this vertex has any unused edges within the SCC
            bool has_unused_edges = false;
            for (unsigned int i = 0; i < graph[v].size(); ++i) {
                unsigned int neighbor = graph[v][i];
                if (scc_set.find(neighbor) != scc_set.end() && !used_edges[v][i]) {
                    has_unused_edges = true;
                    break;
                }
            }
            if (!has_unused_edges) continue;

            // Run DFS to find one cycle starting from this vertex
            cycle_dfs_in_scc(graph, edge_labels, scc_set, v, used_edges, globally_visited, cycles);
        }
    }

    return cycles;
}

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
    for (unsigned int from = 0; from < graph.size(); ++from) {
        used_edges[from].resize(graph[from].size(), false);
    }

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
            const auto& [path_vertices, path_edge_indices] = dijkstra_with_path_info(graph, edge_weights, w, v, used_edges, max_allowed_edge_weight);
            if (path_vertices.empty()) continue;

            // Calculate total cycle weight
            double cycle_weight = vw_weight;
            for (const auto& [from, to] : path_vertices) cycle_weight += edge_weights.at({from, to});

            // Skip if cycle weight >= 1 (we know that there can't be another cycle using {v, w} with lower weight, since the path {w -> v} was the
            // shortest one we could find, using unused edges)
            if (cycle_weight >= 1.0) continue;

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
            for (unsigned int edge_idx : path_edge_indices) {
                used_edges[current][edge_idx] = true;
                current = graph[current][edge_idx];
            }

            cycles.emplace_back(std::move(cycle));
        }
    }

    return cycles;
}

/**
 * @brief Convert to string the content of a vector
 *
 * @tparam T The type of the elements in the vector (note: the elements of the vector will be added to the string using the std::to_string function)
 * @param v The vector
 * @param size = 20 The number of elements to be shown in the string (first size/2 and last size/2 if v.size() > size)
 * @return st::string The string representation of the vector (using std::to_string for each T element of the vector)
 */
template <typename T>
[[nodiscard]]
static inline std::string vtos(std::vector<T> v, unsigned int size = 20) {
    std::string s;
    if (v.size() <= size)
        for (const auto& x : v) s.append(std::to_string(x)).append(";");
    else {
        for (unsigned int i = 0; i < size / 2; i++) s.append(std::to_string(v[i])).append(";");
        s.append("...[").append(std::to_string(v.size() - size)).append("];");
        for (unsigned int i = size / 2; i > 0; i--) s.append(std::to_string(v[v.size() - i])).append(";");
    }
    return s;
}

#endif