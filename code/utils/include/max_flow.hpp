#pragma once

#include <limits>
#include <queue>
#include <unordered_set>
#include <vector>

#include "constants.hpp"

struct NetworkEdge {
    unsigned int to;   // Destination node
    unsigned int rev;  // Index of reverse edge in the destination's adjacency list
    double c;          // Remaining capacity of the edge
    bool is_reverse;
};

using FlowNetwork = std::vector<std::vector<NetworkEdge>>;

// Dinic: build the level graph, returns whether the sink is reachable
[[nodiscard]]
static inline auto max_flow_augmenting_path(const FlowNetwork& graph, unsigned int s, unsigned int t, std::vector<int>& level) -> bool {
    level.assign(graph.size(), -1);
    std::queue<unsigned int> q;
    level[s] = 0;
    q.push(s);

    while (!q.empty()) {
        const auto v = q.front();
        q.pop();
        for (const auto& e : graph[v]) {
            // If this edge still has capacity and its destination hasn't been visited
            if (e.c > constants::epsilon && level[e.to] < 0) {
                level[e.to] = level[v] + 1;
                q.push(e.to);
            }
        }
    }

    return level[t] >= 0;
}

// Dinic: push flow through the level graph
[[nodiscard]]
static inline auto max_flow_push_flow(FlowNetwork& graph, unsigned int v, unsigned int t, double f, const std::vector<int>& level,
                                      std::vector<unsigned int>& iter) -> double {
    if (v == t) {
        return f;
    }

    // iter[v] remembers which edge to start from (avoids revisiting dead ends)
    for (unsigned int& i = iter[v]; i < graph[v].size(); i++) {
        auto& e = graph[v][i];

        // Only use edges with capacity that go to the next level
        if (e.c > constants::epsilon && level[v] < level[e.to]) {
            const double d{max_flow_push_flow(graph, e.to, t, std::min(f, e.c), level, iter)};
            if (d > constants::epsilon) {
                e.c -= d;
                graph[e.to][e.rev].c += d;
                return d;
            }
        }
    }

    return 0;
}

// Compute the max flow on graph (residual capacities are modified in place)
[[nodiscard]]
static inline auto compute_max_flow(FlowNetwork& graph, unsigned int source, unsigned int sink) -> double {
    double flow{0};
    std::vector<int> level(graph.size());
    std::vector<unsigned int> iter(graph.size());

    while (max_flow_augmenting_path(graph, source, sink, level)) {
        iter.assign(graph.size(), 0);
        double f{0};
        while ((f = max_flow_push_flow(graph, source, sink, std::numeric_limits<double>::infinity(), level, iter)) > constants::epsilon) {
            flow += f;
        }
    }

    return flow;
}

// Nodes reachable from source in the residual graph (left side of the min-cut)
[[nodiscard]]
static inline auto get_min_cut_lpartition(const FlowNetwork& graph, unsigned int source) -> std::unordered_set<unsigned int> {
    std::unordered_set<unsigned int> reachable;
    std::queue<unsigned int> q;
    q.push(source);
    reachable.insert(source);

    while (!q.empty()) {
        const auto v = q.front();
        q.pop();
        for (const auto& e : graph[v]) {
            if (e.c > constants::epsilon && !reachable.contains(e.to)) {
                reachable.insert(e.to);
                q.push(e.to);
            }
        }
    }

    return reachable;
}
