#include <list>

#include "relax_callback.hpp"

static inline void compute_r1_r2_incremental(const hplus::instance& inst, const std::vector<double>& relax_point, std::vector<double>& r1_values,
                                             std::vector<double>& r1_act_values, std::vector<double>& r2_values, std::vector<double>& r2_act_values,
                                             binary_set& state, std::queue<unsigned int>& actions_queue, binary_set& acts_in_queue,
                                             std::stack<std::pair<unsigned int, double>>& trail) {
    // Trail keys:
    // - [0, inst.n): r1_values
    // - [inst.n, inst.n + inst.m): r1_act_values
    // - [inst.n + inst.m, 2*inst.n + inst.m): r2_values
    // - [2*inst.n + inst.m, 2*inst.n + 2*inst.m): r2_act_values
    binary_set trail_flags(2 * inst.n + 2 * inst.m);

    while (!actions_queue.empty()) {
        const auto choice{actions_queue.front()};
        actions_queue.pop();
        acts_in_queue.remove(choice);

        // Store the R1 and R2 values for this action... if we see that there's no update on these values we can skip
        double prev_r1_act_value{r1_act_values[choice]}, prev_r2_act_value{r2_act_values[choice]};

        // Values for actions are the minimum value of its precondition, with a cap at it's relaxed value
        r1_act_values[choice] = relax_point[choice];
        r2_act_values[choice] = relax_point[choice];
        for (const auto& p : inst.actions[choice].pre_sparse) {
            r1_act_values[choice] = std::min(r1_act_values[choice], r1_values[p]);
            r2_act_values[choice] = std::min(r2_act_values[choice], r2_values[p]);
        }

        // If neither the R1 or R2 value for this action has changed, we can skip
        if (r1_act_values[choice] - prev_r1_act_value <= HPLUS_EPSILON && r2_act_values[choice] - prev_r2_act_value <= HPLUS_EPSILON) continue;

        // Write to the trail the previous value
        if (r1_act_values[choice] - prev_r1_act_value > HPLUS_EPSILON) {
            unsigned int trail_key = inst.n + choice;
            if (!trail_flags[trail_key]) {
                trail.emplace(trail_key, prev_r1_act_value);
                trail_flags.add(trail_key);
            }
        }
        if (r2_act_values[choice] - prev_r2_act_value > HPLUS_EPSILON) {
            unsigned int trail_key = 2 * inst.n + inst.m + choice;
            if (!trail_flags[trail_key]) {
                trail.emplace(trail_key, prev_r2_act_value);
                trail_flags.add(trail_key);
            }
        }

        state |= inst.actions[choice].eff;

        // Values for facts are:
        // R1: the maximum among the values of the actions that achieve it
        // R2: the sum of the values of the actions that achieve it
        for (const auto& p : inst.actions[choice].eff_sparse) {
            // If the R1 value of p is higher than this action's value we won't need to update R1 values
            // If the R2 value of p is already 1 we won't need to update R2 values
            if (r1_values[p] >= r1_act_values[choice] && r2_values[p] >= 1 - HPLUS_EPSILON) continue;

            // Now we know that either R1 or R2 values will be updated...

            // Update R1 values
            if (r1_values[p] < r1_act_values[choice]) {
                unsigned int trail_key = p;
                if (!trail_flags[trail_key]) {
                    trail.emplace(trail_key, r1_values[p]);
                    trail_flags.add(trail_key);
                }
                r1_values[p] = std::max(r1_values[p], r1_act_values[choice]);
            }
            // Update R2 values
            if (r2_values[p] < 1 - HPLUS_EPSILON) {
                unsigned int trail_key = inst.n + inst.m + p;
                if (!trail_flags[trail_key]) {
                    trail.emplace(trail_key, r2_values[p]);
                    trail_flags.add(trail_key);
                }
                r2_values[p] += (r2_act_values[choice] - prev_r2_act_value);
                r2_values[p] = std::min(r2_values[p], 1.0);
            }

            // Since either R1 or R2 value for p has been updated, we need to add to the queue all actions that have it as precondition
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (!state.contains(inst.actions[act_i].pre)) continue;  // Skip actions that can't be applied yet
                if (acts_in_queue[act_i]) continue;                      // Skip actions that are already in the queue
                if (relax_point[act_i] <= HPLUS_EPSILON) continue;       // Skip actions whose weight is 0
                actions_queue.push(act_i);
                acts_in_queue.add(act_i);
            }
        }
    }
}

[[nodiscard]]
static inline std::vector<unsigned int> compute_pcf(const hplus::instance& inst, const std::vector<double>& r1_values,
                                                    const std::vector<double>& r2_values) {
    // Choose pcf as the precondition with the lowest R1 value
    std::vector<unsigned int> pcf(inst.m + 1, 0);  // Last spot is reserved for the dummy goal action's pcf
    static const unsigned int source = inst.n, sink_action = inst.m;

    auto check_and_update_pcf = [&](unsigned int act_i, unsigned int p, double& min_r1, double& min_r2) {
        if (r1_values[p] < min_r1 - HPLUS_EPSILON) {
            min_r1 = r1_values[p];
            min_r2 = r2_values[p];
            pcf[act_i] = p;
        } else if (std::abs(r1_values[p] - min_r1) < HPLUS_EPSILON && r2_values[p] < min_r2 - HPLUS_EPSILON) {  // Use R2 as tie-breaking
            min_r2 = r2_values[p];
            pcf[act_i] = p;
        }
    };

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            pcf[act_i] = source;
            continue;
        }
        double min_r1{2}, min_r2{2};  // 1 is the max value each R1 or R2 value can have... using 2 as initial min guarantees a lower R1 value
        for (const auto& p : inst.actions[act_i].pre_sparse) check_and_update_pcf(act_i, p, min_r1, min_r2);
    }

    // Compute the pcf for the dummy goal action
    double min_r1{2}, min_r2{2};
    for (const auto& p : inst.goal) check_and_update_pcf(sink_action, p, min_r1, min_r2);

    return pcf;
}

static inline std::pair<std::vector<std::vector<network_edge>>, std::vector<unsigned int>> max_flow_graph_construction(const hplus::instance& inst) {
    std::vector<std::vector<network_edge>> graph(inst.n + 2);
    // nodes [0 -> n - 1] => facts
    // node [n] => source
    static const unsigned int source{inst.n};
    // node [n + 1] => sink
    static const unsigned int sink{inst.n + 1};
    // nodes [n + 2 -> ...] => dummy nodes for actions effects (at most m)
    // estimated number of nodes: O(n + m)

    std::vector<unsigned int> actions_effect(inst.m + 1, 0);
    // 0 -> m - 1 => actions
    // m => dummy goal action
    static const unsigned int sink_action{inst.m};

    auto add_edge = [&](unsigned int a, unsigned int b, double capacity) {
        const unsigned int edge_ba_idx = graph[b].size(), edge_ab_idx = graph[a].size();
        graph[a].emplace_back(b, edge_ba_idx, capacity);  // Forward (residual) edge
        graph[b].emplace_back(a, edge_ab_idx, 0);         // Reverse (used flow) edge
    };

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        // Find this action's effect
        ASSERT(!inst.actions[act_i].eff_sparse.empty());
        // If this action has only one effect, that's it... if it has more effects use a dummy node
        unsigned int to = inst.actions[act_i].eff_sparse.size() == 1 ? inst.actions[act_i].eff_sparse[0] : graph.size();
        if (inst.actions[act_i].eff_sparse.size() > 1) graph.push_back(std::vector<network_edge>());

        actions_effect[act_i] = to;

        // Get all preconditions (linking the source node if needed)
        std::vector<unsigned int> preconditions;
        if (inst.actions[act_i].pre_sparse.empty())
            preconditions.push_back(source);
        else
            preconditions = inst.actions[act_i].pre_sparse;

        for (const auto& from : preconditions) {
            // Whether "to" is a dummy node or the only effect, create an edge from each precondition to "to"
            // This must be adjusted according to the selected pcf and the action's weight in the relax point:
            // w(from, act_eff[act_i]) = (from == pcf[act_i]) ? relax_point[act_i] : 0
            add_edge(from, to, 0);

            switch (inst.actions[act_i].eff_sparse.size()) {
                case 0:
                    LOG_ERROR << "Found action with no effects...";
                    break;
                case 1:  // If only one effect, then "to" is it: no more edges to add
                    break;
                default:  // If there are multiple effects we need to add the "to" -> eff edges
                    for (const auto& eff : inst.actions[act_i].eff_sparse)
                        add_edge(to, eff,
                                 1);  // dummy edges have capacity of 1 since they are sourced only by the from -> to edge, which will have
                                      // the action's weight... in this way we guarantee the min-cut to only be associated to real actions
                                      // and not dummy ones (if capacity is 1, then this won't ever be saturated, unless the minimum cut is >= 1)
            }
        }
    }

    // Adding the edges linking the sink with the rest of the graph
    for (const auto& g : inst.goal) add_edge(g, sink, 0);  // This must be adjusted according the selected pcf
    actions_effect[sink_action] = sink;

    return {graph, actions_effect};
}

// static inline bool is_flow_conservative(const hplus::instance& inst, const std::vector<std::vector<network_edge>>& graph, const unsigned int
// source,
//                                         const unsigned int sink) {
//     const unsigned int n = graph.size();
//     std::vector<double> net_flow(n, 0.0);
//     binary_set visited(n);

//     std::queue<unsigned int> queue;
//     queue.push(source);

//     while (!queue.empty()) {
//         const auto u = queue.front();
//         queue.pop();
//         visited.add(u);

//         for (const auto& [to, rev, c] : graph[u]) {
//             double flow = c - graph[to][rev].c;
//             net_flow[u] += flow;
//             if (!visited[to]) queue.push(to);
//         }
//     }

//     // // Calculate net flow for each node
//     // // Strategy: only process each edge pair once (when u < edge.to)
//     // for (unsigned int u = 0; u < n; ++u) {
//     //     for (const auto& edge : graph[u]) {
//     //         // Only process each edge pair once to avoid double counting
//     //         if (u < edge.to) {
//     //             // Flow through edge u -> edge.to is the reverse edge capacity
//     //             double flow = graph[edge.to][edge.rev].c;

//     //             // This flow goes OUT of u and INTO edge.to
//     //             net_flow[u] -= flow;
//     //             net_flow[edge.to] += flow;
//     //         }
//     //     }
//     // }

//     // Check conservation for all nodes except source and sink
//     for (unsigned int i = 0; i < n; ++i) {
//         if (i == source || i == sink) {
//             if (std::abs(net_flow[source] + net_flow[sink]) > HPLUS_EPSILON) {
//                 LOG_WARNING << "Flow out of source doesn't match flow into sink";
//                 return false;
//             }
//         }

//         // For conservation, net_flow should be approximately 0
//         if (std::abs(net_flow[i]) > HPLUS_EPSILON) {
//             LOG_DEBUG << "A net flow is not zero";
//             return false;
//         }
//     }

//     return true;
// }

static inline double update_graph(const hplus::instance& inst, std::vector<std::vector<network_edge>>& graph, const std::vector<double>& relax_point,
                                  const std::vector<unsigned int> pcf, const std::vector<unsigned int>& actions_eff) {
    static const unsigned int source = inst.n, sink = inst.n + 1, sink_action = inst.m + 1;

    double removed_flow = 0;

    // TODO: Remove, just for debugging
    double previous_flow = 0, current_flow = 0;
    for (const auto& [to, rev, c] : graph[source]) previous_flow += graph[to][rev].c;

    // ASSERT(is_flow_conservative(graph, source, sink));

    LOG_DEBUG << "Previous flow: " << previous_flow;

    LOG_DEBUG << "Source: " << source << " - Sink: " << sink << " - Sink action: " << sink_action;

    // Remove old pcf edges
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        std::vector<unsigned int> pre;
        if (inst.actions[act_i].pre_sparse.empty())
            pre.push_back(source);
        else
            pre = inst.actions[act_i].pre_sparse;

        for (const auto& p : pre) {
            LOG_DEBUG << "| Precondition: " << p;
            if (p == pcf[act_i]) continue;

            const unsigned int to = actions_eff[act_i], rev_index = graph[p][to].rev;

            LOG_DEBUG << "Action's effect: " << to;

            // Reset the residual capacity... no additional steps required
            graph[p][to].c = 0;
            // If there's flow, remove it and fix the graph accordingly
            if (graph[to][rev_index].c > HPLUS_EPSILON) {
                const double flow_to_remove = graph[to][rev_index].c;
                graph[to][rev_index].c = 0;
                flow_removal(graph, to, sink, flow_to_remove);
                LOG_DEBUG << "Finished the first one";
                flow_removal(graph, source, p, flow_to_remove);
                LOG_DEBUG << "Finished the second one";
                exit(1);
            }
        }
    }

    LOG_DEBUG << "HERE";
    exit(1);

    // Fix the goal's pcf
    for (const auto& p : inst.goal) {
        unsigned int edge_idx = 0;
        for (unsigned int i = 0; i < graph[p].size(); i++) {
            if (graph[p][i].to == sink) {
                edge_idx = i;
                break;
            }
        }
        const unsigned int rev_idx = graph[p][edge_idx].rev;
        const double capacity = graph[p][edge_idx].c + graph[sink][rev_idx].c;

        // If this goal fact is the pcf...
        if (p == pcf[sink_action]) {
            // If the capacity of this edge is already 1, then the pcf didn't change from the previous iteration... we can directly skip this update
            if (capacity >= 1 - HPLUS_EPSILON) break;

            // ... otherwise, we need to set it's capacity to 1 (just increase the residual capacity)

            graph[p][edge_idx].c += 1 - capacity;
        }

        // If this goal fact isn't the pcf (anymore)...
        else {
            // Reset the residual capacity... no additional steps required
            graph[p][sink].c = 0;
            // If there's flow, remove it and fix the graph accordingly
            if (graph[sink][rev_idx].c > HPLUS_EPSILON) {
                const double flow_to_remove = graph[sink][rev_idx].c;
                graph[sink][rev_idx].c = 0;
                flow_removal(graph, source, p, flow_to_remove);
            }
        }
    }

    // Update graph according to action's new weights
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        const unsigned int from = pcf[act_i], to = actions_eff[act_i];
        unsigned int to_idx = 0;
        for (unsigned int i = 0; i < graph[from].size(); i++) {
            const auto& [t, _1, _2] = graph[from][i];
            if (to == t) {
                to_idx = i;
                break;
            }
        }

        const unsigned int rev_idx = graph[from][to_idx].rev;
        const double edge_flow = graph[to][rev_idx].c, edge_residual = graph[from][to_idx].c, edge_capacity = edge_flow + edge_residual,
                     diff = relax_point[act_i] - edge_capacity;

        // No change, go to next action
        if (std::abs(diff) <= HPLUS_EPSILON) {
            continue;
        }
        // Increase the capacity: simply increase the residual capacity
        else if (diff > HPLUS_EPSILON) {
            graph[from][to_idx].c += diff;
        }
        // Reduce residual capacity and (if needed) the flow
        else {
            const double new_residual_capacity = graph[from][to_idx].c + diff;

            // Reducing residual capacity is enough... no flow has changed, so the flow conservation is preserved
            if (new_residual_capacity >= -HPLUS_EPSILON) {
                graph[from][to_idx].c = (new_residual_capacity > HPLUS_EPSILON) ? new_residual_capacity : 0;
                continue;
            }
            // The residual flow is not enough.. we have to reduce the flow aswell
            else {
                graph[from][to_idx].c = 0;                             // Remove all residual capacity
                const double flow_to_remove = -new_residual_capacity;  // Compute the flow needed to be removed
                graph[to][rev_idx].c -= flow_to_remove;                // Remove it from this edge
                flow_removal(graph, to, sink, flow_to_remove);         // Reduce the flow reaching the sink
                flow_removal(graph, source, from, flow_to_remove);     // Reduce the flow leaving the source

                removed_flow += flow_to_remove;
            }
        }

        // Invariant: at the end of each iteration, the graph is still in a situation of flow conservation and the
        // current flow (computable as the amount of flow leaving the source) is previous_max_flow - removed_flow
        current_flow = 0;
        for (const auto& [to, rev, c] : graph[source]) current_flow += graph[to][rev].c;
        ASSERT(std::abs(previous_flow - removed_flow - current_flow) < HPLUS_EPSILON);
        // ASSERT(is_flow_conservative(graph, source, sink));
    }

    return removed_flow;
}

[[nodiscard]]
static inline std::vector<std::vector<network_edge>> build_max_flow_graph(const hplus::instance& inst, const std::vector<double>& actions_weights,
                                                                          const std::vector<unsigned int>& pcf, std::vector<int>& actions_effect) {
    std::vector<std::vector<network_edge>> graph(inst.n + 2);
    // nodes [0 -> n - 1] => facts
    // node [n] => source
    // static const unsigned int source{inst.n};
    // node [n + 1] => sink
    static const unsigned int sink{inst.n + 1}, sink_action = inst.m;
    // nodes [n + 2 -> ...] => dummy nodes for actions effects (at most m)
    // estimated number of nodes: O(n + m)

    actions_effect = std::vector<int>(inst.m, -1);

    auto add_edge = [&](unsigned int from, unsigned int to, double capacity) {
        graph[from].emplace_back(to, graph[to].size(), capacity);
        graph[to].emplace_back(from, graph[from].size() - 1, 0);  // Reverse edge
    };

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        const unsigned int from = pcf[act_i];
        switch (inst.actions[act_i].eff_sparse.size()) {
            case 0:  // No effect -> No edge
                LOG_ERROR << "Found action with no effects...";
                break;
            case 1:  // One effect -> Simple edge
                add_edge(from, inst.actions[act_i].eff_sparse[0], actions_weights[act_i]);
                actions_effect[act_i] = inst.actions[act_i].eff_sparse[0];
                break;
            default:  // Multiple effects -> Create one edge a -> dummy and multiple edges dummy -> effect
                const unsigned int dummy{static_cast<unsigned int>(graph.size())};
                actions_effect[act_i] = dummy;
                graph.push_back(std::vector<network_edge>());  // Create new dummy node
                add_edge(from, dummy, actions_weights[act_i]);
                for (const auto& to : inst.actions[act_i].eff_sparse) add_edge(dummy, to, 1);
                break;
        }
    }

    // Add edge for goal's pcf
    add_edge(pcf[sink_action], sink, 1);

    return graph;
}

[[nodiscard]]
static inline std::vector<unsigned int> get_r3_violated_landmark(const hplus::instance& inst, const std::vector<std::vector<network_edge>>& graph) {
    binary_set graph_reach{get_min_cut_lpartition(graph, inst.n)}, facts_reach(inst.n);
    // This needs to be done since binary_set check for the capacity of the sets... I need to make sure this has the same capacity of the
    // preconditions and effects of actions
    for (unsigned int i = 0; i < inst.n; i++) {
        if (graph_reach[i]) facts_reach.add(i);
    }

    ASSERT(!facts_reach.contains(inst.goal));

    std::vector<unsigned int> landmark;
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (facts_reach.contains(inst.actions[act_i].pre) && !facts_reach.contains(inst.actions[act_i].eff)) landmark.push_back(act_i);
    }

    return landmark;
}

[[nodiscard]]
std::pair<bool, std::vector<unsigned int>> relax_cuts::get_violated_landmark(const hplus::execution& exec, const hplus::instance& inst,
                                                                             const std::vector<double>& relax_point) {
    std::vector<unsigned int> landmark(1);

    // ====================================================== //
    // ============ Minimization data structures ============ //
    // ====================================================== //
    std::vector<double> relax_point_copy(relax_point.begin(), relax_point.end());
    unsigned int rounded_act_lmidx{0}, rounded_act{0};
    double prev_act_val{relax_point_copy[rounded_act_lmidx]};
    bool found{false};
    const unsigned int source = inst.n, sink = inst.n + 1;

    // ====================================================== //
    // ============ R1 R2 incremental computation =========== //
    // ====================================================== //
    std::vector<double> r1_values(inst.n, 0), r1_act_values(inst.m, 0), r2_values(inst.n, 0), r2_act_values(inst.m, 0);
    std::queue<unsigned int> r1r2_actions_queue;
    binary_set r1r2_state(inst.n), r1r2_reversing_state(inst.n), r1r2_acts_in_queue(inst.m);
    // Trail keys:
    // - [0, inst.n): r1_values
    // - [inst.n, inst.n + inst.m): r1_act_values
    // - [inst.n + inst.m, 2*inst.n + inst.m): r2_values
    // - [2*inst.n + inst.m, 2*inst.n + 2*inst.m): r2_act_values
    std::stack<std::pair<unsigned int, double>> r1r2_trail;
    auto revert_r1r2_changes = [&]() {
        while (!r1r2_trail.empty()) {
            const auto& [key, value] = r1r2_trail.top();
            r1r2_trail.pop();
            if (key < inst.n)  // R1 values
                r1_values[key] = value;
            else if (key < inst.n + inst.m)  // R1 act values
                r1_act_values[key - inst.n] = value;
            else if (key < 2 * inst.n + inst.m)  // R2 values
                r2_values[key - inst.n - inst.m] = value;
            else  // R2 act values
                r2_act_values[key - 2 * inst.n - inst.m] = value;
        }
        r1r2_state = r1r2_reversing_state;
    };
    // Initialization for incremental R1 R2 computation
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (relax_point_copy[act_i] < HPLUS_EPSILON) continue;
        if (inst.actions[act_i].pre_sparse.empty()) {
            r1r2_actions_queue.push(act_i);
            r1r2_acts_in_queue.add(act_i);
        }
    }

    // ====================================================== //
    // ============= R3 incremental computation ============= //
    // ====================================================== //
    std::vector<std::vector<network_edge>> old_max_flow_graph;
    double r3 = 0;
    auto [new_max_flow_graph, actions_effect] = max_flow_graph_construction(inst);
    auto revert_r3_changes = [&]() { new_max_flow_graph = old_max_flow_graph; };

    // ====================================================== //
    // =========== Minimization procedure helpers =========== //
    // ====================================================== //
    auto round_action = [&](int act_idx) {
        rounded_act = landmark[act_idx];
        prev_act_val = relax_point_copy[rounded_act];
        relax_point_copy[rounded_act] = 1;
        r1r2_actions_queue.push(rounded_act);
        r1r2_acts_in_queue.add(rounded_act);
    };

    auto round_next_action = [&]() {
        relax_point_copy[rounded_act] = prev_act_val;
        rounded_act_lmidx++;
        if (rounded_act_lmidx >= landmark.size()) return false;
        round_action(rounded_act_lmidx);
        return true;
    };

    auto round_new_action = [&]() {
        // TODO: Maybe an heuristic to tell which actions to use first? (If done, update it also in the revert sections)
        found = true;
        rounded_act_lmidx = 0;
        round_action(rounded_act_lmidx);
    };

    // TODO: Remove... just for debugging
    size_t initial_lm_size = 0, final_lm_size = 0;
    double initial_violation = 0, final_violation = 0, start_time = GET_TIME(), normal_time = 0, minimization_time = 0;
    unsigned int repetitions{0};

    // ====================================================== //
    // =============== Minimization procedure =============== //
    // ====================================================== //
    while (rounded_act_lmidx < landmark.size()) {
        repetitions++;

        // ~~~~~~~~~~ R1/R2 COMPUTATION ~~~~~~~~~~ //
        // const auto& [r1, r1_values, r2_values] = compute_r1_r2(inst, relax_point_copy);
        r1r2_trail = std::stack<std::pair<unsigned int, double>>();  // Reset the trail
        r1r2_reversing_state = r1r2_state;                           // Store current state
        compute_r1_r2_incremental(inst, relax_point_copy, r1_values, r1_act_values, r2_values, r2_act_values, r1r2_state, r1r2_actions_queue,
                                  r1r2_acts_in_queue, r1r2_trail);
        double r1{1};
        for (const auto& p : inst.goal) r1 = std::min(r1, r1_values[p]);

        // If R1 >= 1 there's no violated landmark
        if (r1 >= 1 - HPLUS_EPSILON) {
            if (found) {
                revert_r1r2_changes();
                bool has_next = round_next_action();
                if (!has_next) break;
                // LOG_DEBUG << "R1 == 1, next action";
                continue;
            } else
                return {false, {}};
        }

        // Otherwise, we rely on R3 to understand wether there's a violated landmark...

        // ~~~~~~~~~~~~ R3 COMPUTATION ~~~~~~~~~~~ //
        LOG_DEBUG << "We get here";
        double removed_flow = update_graph(inst, new_max_flow_graph, relax_point_copy, compute_pcf(inst, r1_values, r2_values), actions_effect);
        exit(1);
        r3 = r3 - removed_flow + compute_max_flow(new_max_flow_graph, source, sink);

        // LOG_DEBUG << r3;
        // exit(1);

        // if (!found) {  // First iteration
        // new_max_flow_graph = build_max_flow_graph(inst, relax_point_copy, r1_values, r2_values, new_pcf, actions_eff);
        // r3 = compute_max_flow(new_max_flow_graph, source, sink);
        // } else {  // This is a minimization iteration
        //     // TODO: Remove, only for debugging...
        // std::vector<int> tmp;
        // new_max_flow_graph = build_max_flow_graph(inst, relax_point_copy, compute_pcf(inst, r1_values, r2_values), tmp);
        // r3 = compute_max_flow(new_max_flow_graph, source, sink);

        // ASSERT(is_flow_conservative(inst, new_max_flow_graph, source, sink));

        //     // ~~~~~~ R3 INCREMENTAL COMPUTATION ~~~~~ //
        //     unsigned int previous_pcf = old_pcf[rounded_act] < 0 ? source : static_cast<unsigned int>(old_pcf[rounded_act]);
        //     unsigned int current_pcf = new_pcf[rounded_act] < 0 ? source : static_cast<unsigned int>(new_pcf[rounded_act]);
        //     unsigned int action_eff = actions_eff[rounded_act];

        //     // Update the rounded action's edge
        //     if (previous_pcf != current_pcf) {
        //         incremental_edge_deletion(new_max_flow_graph, source, sink, previous_pcf, action_eff, prev_act_val);
        //         incremental_edge_insertion(new_max_flow_graph, source, sink, current_pcf, action_eff, 1);
        //     } else {
        //         incremental_edge_insertion(new_max_flow_graph, source, sink, current_pcf, action_eff, 1 - prev_act_val);
        //     }

        //     // Update the action's whose pcf changed
        //     for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        //         previous_pcf = old_pcf[act_i] < 0 ? source : static_cast<unsigned int>(old_pcf[act_i]);
        //         current_pcf = new_pcf[act_i] < 0 ? source : static_cast<unsigned int>(new_pcf[act_i]);
        //         if (previous_pcf == current_pcf || act_i == rounded_act) continue;

        //         const double weight = relax_point_copy[act_i];
        //         action_eff = actions_eff[act_i];

        //         incremental_edge_deletion(new_max_flow_graph, source, sink, previous_pcf, action_eff, weight);
        //         incremental_edge_insertion(new_max_flow_graph, source, sink, current_pcf, action_eff, weight);
        //     }

        //     // Compute the new R3
        //     r3 = 0;
        //     for (unsigned int i = 0; i < new_max_flow_graph[source].size(); i++) {
        //         const auto& [to, rev, c] = new_max_flow_graph[source][i];
        //         r3 += new_max_flow_graph[to][rev].c;
        //     }

        // if (std::abs(manual_r3 - r3) >= HPLUS_EPSILON) LOG_DEBUG << manual_r3 << " / " << r3;

        // ASSERT(std::abs(manual_r3 - r3) < HPLUS_EPSILON);
        // }

        // Note: if R3 < 1, then there's a violated landmark, BUT if R3 == 1 we can't assume that no landmark is violated... in this case we simply
        // ignore the possible landmark Statistically speaking, R3 == 1 but there exists a violated landmark happens in 3% of cases
        if (r3 >= 1 - HPLUS_EPSILON) {
            if (found) {
                revert_r1r2_changes();
                revert_r3_changes();
                bool has_next = round_next_action();
                if (!has_next) break;
                // LOG_DEBUG << "R3 == 1, next action";
                continue;
            } else
                return {false, {}};
        }

        // Now we know that there's a new landmark to extract...
        old_max_flow_graph = new_max_flow_graph;

        // ~~~~~~~ GET LANDMARK AS MIN-CUT ~~~~~~~ //
        landmark = get_r3_violated_landmark(inst, new_max_flow_graph);

        // TODO: Remove... just for debugging
        double post_cutval{0};
        for (const auto& x : landmark) post_cutval += relax_point[x];
        if (!found) {
            initial_lm_size = landmark.size();
            initial_violation = 1 - post_cutval;
            normal_time = (GET_TIME() - start_time) * 1000;
        } else {
            final_lm_size = landmark.size();
            final_violation = 1 - post_cutval;
        }

        // ~~~~~ MINIMIZATION PROCEDURE SETUP ~~~~ //
        round_new_action();

        // LOG_DEBUG << "Found landmark, rounding new action";

        if (!exec.min_fract_lm) break;
    }

    // TODO: Remove... just for debugging
    minimization_time = (GET_TIME() - start_time) * 1000;
    // if (exec.min_fract_lm)
    LOG_DEBUG << "Minimization results: LM size: " << std::setw(5) << initial_lm_size << " -> " << std::setw(5) << final_lm_size
              << " -- Violation: " << std::fixed << std::setprecision(4) << initial_violation << " -> " << std::fixed << std::setprecision(4)
              << final_violation << " -- Repetitions: " << std::setw(4) << repetitions << " -- Time: normal: " << std::fixed << std::setw(6)
              << std::setprecision(2) << normal_time << "ms total: " << std::fixed << std::setw(6) << std::setprecision(2) << minimization_time
              << "ms";

    return {true, landmark};
}

[[nodiscard]]
unsigned int relax_cuts::add_lm_cut(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst,
                                    const std::vector<double>& relax_point) {
    const auto& [found, landmark]{get_violated_landmark(exec, inst, relax_point)};
    if (!found) return 0;
    std::vector<int> ind(landmark.size());
    int nnz{0};
    for (const auto& act_i : landmark) ind[nnz++] = act_i;
    std::vector<double> val(landmark.size(), 1.0);
    constexpr double rhs{1};
    constexpr char sense{'G'};
    constexpr int begin{0}, purgeable{CPX_USECUT_FORCE}, local{0};
    CPX_HANDLE_CALL(CPXcallbackaddusercuts(context, 1, nnz, &rhs, &sense, &begin, ind.data(), val.data(), &purgeable, &local));
    return 1;
}
