#include <iostream>
#include <list>
#include <set>
#include <stack>

#include "../utils/max_flow.hpp"
#include "relax_callback.hpp"

[[nodiscard]]
static inline double compute_r1_r2_incremental(const hplus::instance& inst, const std::vector<double>& relax_point, std::vector<double>& r1_values,
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

    double r1{1};
    for (const auto& p : inst.goal) r1 = std::min(r1, r1_values[p]);

    return r1;
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
        graph[a].emplace_back(b, edge_ba_idx, capacity, false);  // Forward (residual) edge
        graph[b].emplace_back(a, edge_ab_idx, 0, true);          // Reverse (used flow) edge
    };

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        // Find this action's effect
        // To prevent duplicates in the adjacency lists always create a dummy node so that we have: pcf -> dummy -> effect(s)
        unsigned int to = graph.size();
        graph.push_back(std::vector<network_edge>());

        actions_effect[act_i] = to;

        // Get all preconditions (linking the source node if needed)
        std::vector<unsigned int> preconditions;
        if (inst.actions[act_i].pre_sparse.empty())
            preconditions.push_back(source);
        else
            preconditions = inst.actions[act_i].pre_sparse;

        for (const auto& from : preconditions) {
            // This must be adjusted according to the selected pcf and the action's weight in the relax point:
            // w(from, act_eff[act_i]) = (from == pcf[act_i]) ? relax_point[act_i] : 0
            add_edge(from, to, 0);

            for (const auto& eff : inst.actions[act_i].eff_sparse)
                add_edge(to, eff,
                         1);  // dummy edges have capacity of 1 since they are sourced only by the from -> to edge, which will have
                              // the action's weight... in this way we guarantee the min-cut to only be associated to real actions
                              // and not dummy ones (if capacity is 1, then this won't ever be saturated, unless the minimum cut is >= 1)
        }
    }

    // Adding the edges linking the sink with the rest of the graph
    for (const auto& g : inst.goal) add_edge(g, sink, 0);  // This must be adjusted according the selected pcf
    actions_effect[sink_action] = sink;

    return {graph, actions_effect};
}

[[nodiscard]]
static inline double compute_r3_incremental(const hplus::instance& inst, std::vector<std::vector<network_edge>>& graph,
                                            const std::vector<double>& relax_point, const std::vector<unsigned int> pcf,
                                            const std::vector<unsigned int>& actions_eff, double prev_r3) {
    static const unsigned int source = inst.n, sink = inst.n + 1, sink_action = inst.m;

    double removed_flow = 0;

    // Fix goal's pcf
    for (const auto& p : inst.goal) {
        unsigned int to_idx = 0;
        for (unsigned int i = 0; i < graph[p].size(); i++) {
            if (graph[p][i].to == sink) {
                to_idx = i;
                break;
            }
        }
        const unsigned int rev_idx = graph[p][to_idx].rev;
        const double flow = graph[sink][rev_idx].c, res_cap = graph[p][to_idx].c, capacity = flow + res_cap;

        if (p == pcf[sink_action]) {
            double delta = 1 - capacity;
            if (delta <= HPLUS_EPSILON) continue;

            graph[p][to_idx].c = 1;
        } else {
            if (capacity < HPLUS_EPSILON) continue;

            graph[p][to_idx].c = 0;
            if (flow > HPLUS_EPSILON) {
                const double flow_to_remove = graph[sink][rev_idx].c;
                graph[sink][rev_idx].c = 0;
                double removed = flow_removal(graph, source, p, flow_to_remove);

                ASSERT(abs(flow_to_remove - removed) <= HPLUS_EPSILON);

                removed_flow += removed;
            }
        }
    }

    // Update flow in pcf changes and in fractional point increases
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        std::vector<unsigned int> pre;
        if (inst.actions[act_i].pre_sparse.empty())
            pre.push_back(source);
        else
            pre = inst.actions[act_i].pre_sparse;

        const unsigned int to = actions_eff[act_i];

        for (const auto& p : pre) {
            // Get info on the current edge
            unsigned int to_idx = 0;
            for (unsigned int i = 0; i < graph[p].size(); i++) {
                if (graph[p][i].to == to) {
                    to_idx = i;
                    break;
                }
            }
            const unsigned int rev_index = graph[p][to_idx].rev;
            double flow = graph[to][rev_index].c, res_cap = graph[p][to_idx].c, capacity = flow + res_cap;

            if (p == pcf[act_i]) {
                double delta = relax_point[act_i] - capacity;

                if (delta > HPLUS_EPSILON)
                    graph[p][to_idx].c += delta;
                else if (delta < -HPLUS_EPSILON) {
                    delta = -delta;
                    const double new_residual_capacity = graph[p][to_idx].c - delta;

                    graph[p][to_idx].c = (new_residual_capacity > HPLUS_EPSILON) ? new_residual_capacity : 0;

                    // The residual flow is not enough.. we have to reduce the flow aswell
                    if (new_residual_capacity < -HPLUS_EPSILON) {
                        const double flow_to_remove = -new_residual_capacity;                    // Compute the flow needed to be removed
                        graph[to][rev_index].c -= flow_to_remove;                                // Remove it from this edge
                        double removed_ahead = flow_removal(graph, to, sink, flow_to_remove);    // Reduce the flow reaching the sink
                        double removed_before = flow_removal(graph, source, p, flow_to_remove);  // Reduce the flow leaving the source

                        ASSERT(std::abs(removed_ahead - removed_before) < HPLUS_EPSILON);

                        removed_flow += removed_ahead;
                    }
                }

            } else {
                if (capacity < HPLUS_EPSILON) continue;

                graph[p][to_idx].c = 0;
                if (flow > HPLUS_EPSILON) {
                    const double flow_to_remove = flow;
                    graph[to][rev_index].c = 0;
                    double removed_ahead = flow_removal(graph, to, sink, flow_to_remove);
                    double removed_before = flow_removal(graph, source, p, flow_to_remove);

                    ASSERT(abs(removed_ahead - removed_before) <= HPLUS_EPSILON);

                    removed_flow += removed_ahead;
                }
            }
        }
    }

    // Remove disconnected parts of the graph
    binary_set reachable(graph.size());
    std::queue<unsigned int> to_visit;
    to_visit.push(source);
    reachable.add(source);

    // Parts connected to the graph
    while (!to_visit.empty()) {
        unsigned int node = to_visit.front();
        to_visit.pop();

        for (const auto& [to, rev, c, is_rev] : graph[node]) {
            // A node is connected with the rest if there's an edge connnected to it, which has flow
            if (!is_rev && graph[to][rev].c > HPLUS_EPSILON && !reachable[to]) {
                reachable.add(to);
                to_visit.push(to);
            }
        }
    }

    // Remove parts that have no flow reaching them from the source
    for (const auto& node : !reachable) {
        for (auto& edge : graph[node]) {
            // If this is a reverse edge (contains flow)
            if (edge.is_reverse && edge.c > HPLUS_EPSILON) {
                const double flow_to_remove = edge.c;
                edge.c = 0;
                graph[edge.to][edge.rev].c += flow_to_remove;
            }
        }
    }

    double r3 = prev_r3 - removed_flow + compute_max_flow(graph, source, sink);

    return r3;
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
                                                                             std::vector<double> relax_point, unsigned int& act_in_lm,
                                                                             unsigned int& n_lm) {
    std::vector<unsigned int> landmark;

    // R1 and R2 data
    std::vector<double> r1_values(inst.n, 0), r1_act_values(inst.m, 0), r2_values(inst.n, 0), r2_act_values(inst.m, 0);
    std::queue<unsigned int> r1r2_actions_queue;
    binary_set r1r2_state(inst.n), r1r2_acts_in_queue(inst.m);
    std::stack<std::pair<unsigned int, double>> r1r2_trail;

    // R3 data
    auto [max_flow_graph, actions_effect] = max_flow_graph_construction(inst);
    std::vector<unsigned int> pcf;
    double r3{0};

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (relax_point[act_i] < HPLUS_EPSILON) continue;
        if (inst.actions[act_i].pre_sparse.empty()) {
            r1r2_actions_queue.push(act_i);
            r1r2_acts_in_queue.add(act_i);
        }
    }

    double r1 = compute_r1_r2_incremental(inst, relax_point, r1_values, r1_act_values, r2_values, r2_act_values, r1r2_state, r1r2_actions_queue,
                                          r1r2_acts_in_queue, r1r2_trail);
    if (r1 >= 1 - HPLUS_EPSILON) return {false, {}};

    pcf = compute_pcf(inst, r1_values, r2_values);
    r3 = compute_r3_incremental(inst, max_flow_graph, relax_point, pcf, actions_effect, r3);

    if (r3 >= 1 - HPLUS_EPSILON) return {false, {}};

    landmark = get_r3_violated_landmark(inst, max_flow_graph);

    if (exec.min_fract_lm) {
        act_in_lm += landmark.size();
        n_lm++;

        unsigned int rounded_act_lmidx{0}, rounded_act{0}, minimization_repetitions{0};
        double prev_act_val{0};
        std::multiset<unsigned int> act_rejected_count;

        // ====================================================== //
        // ======== R1 R2 incremental computation helpers ======= //
        // ====================================================== //
        binary_set r1r2_prev_state(r1r2_state);
        // Trail keys:
        // - [0, inst.n): r1_values
        // - [inst.n, inst.n + inst.m): r1_act_values
        // - [inst.n + inst.m, 2*inst.n + inst.m): r2_values
        // - [2*inst.n + inst.m, 2*inst.n + 2*inst.m): r2_act_values
        auto revert_r1r2_changes = [&]() {
            while (!r1r2_trail.empty()) {
                const auto [key, value] = r1r2_trail.top();
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
            r1r2_state = r1r2_prev_state;
        };

        // ====================================================== //
        // ========= R3 incremental computation helpers ========= //
        // ====================================================== //
        std::vector<std::vector<network_edge>> prev_max_flow_graph = max_flow_graph;
        double prev_r3{r3}, initial_r3{r3}, violation{1 - r3};
        unsigned int max_flow_computations{0};
        auto revert_r3_changes = [&]() {
            max_flow_graph = prev_max_flow_graph;
            r3 = prev_r3;
        };

        // ====================================================== //
        // =========== Minimization procedure helpers =========== //
        // ====================================================== //
        auto round_action = [&](int act_idx) {
            rounded_act = landmark[act_idx];
            prev_act_val = relax_point[rounded_act];
            relax_point[rounded_act] = 1;
            // R1 R2 preparations
            r1r2_actions_queue.push(rounded_act);
            r1r2_acts_in_queue.add(rounded_act);
        };

        auto round_next_action = [&]() {
            relax_point[rounded_act] = prev_act_val;
            act_rejected_count.insert(rounded_act);
            rounded_act_lmidx++;
            if (rounded_act_lmidx >= landmark.size()) return false;
            round_action(rounded_act_lmidx);
            return true;
        };

        auto sort_landmark = [&]() {
            std::sort(landmark.begin(), landmark.end(), [&](unsigned int a, unsigned int b) {
                if (std::abs(relax_point[a] - relax_point[b]) <= HPLUS_EPSILON) {
                    // Prefer actions that were rejected less (an action is "rejected" when rounding it causes r3 to be >= 1)
                    return act_rejected_count.count(a) < act_rejected_count.count(b);
                }
                return relax_point[a] > relax_point[b];  // Prefer actions with smaller violation (violation = 1 - fract_value)
            });
        };

        auto round_new_action = [&]() {
            r1r2_prev_state = r1r2_state;
            r1r2_trail = std::stack<std::pair<unsigned int, double>>();
            prev_max_flow_graph = max_flow_graph;
            prev_r3 = r3;

            rounded_act_lmidx = 0;
            if (exec.lm_min_sort) sort_landmark();
            round_action(rounded_act_lmidx);
        };

        auto terminate_condition = [&]() {
            if (CHECK_STOP()) return true;

            // If we reached the end of this landmark we have no more actions to round... return the "best" landmark we found
            if (rounded_act_lmidx >= std::min(exec.lm_min_lookahead, static_cast<unsigned int>(landmark.size()))) return true;

            // If we reached our iteration limit, stop
            if (max_flow_computations >= exec.lm_min_it) return true;

            minimization_repetitions++;

            return false;
        };

        // ====================================================== //
        // ============ Start minimization procedure ============ //
        // ====================================================== //
        round_new_action();

        while (!terminate_condition()) {
            r1 = compute_r1_r2_incremental(inst, relax_point, r1_values, r1_act_values, r2_values, r2_act_values, r1r2_state, r1r2_actions_queue,
                                           r1r2_acts_in_queue, r1r2_trail);
            if (r1 >= 1 - HPLUS_EPSILON) {
                revert_r1r2_changes();
                bool has_next = round_next_action();
                if (!has_next) break;
                continue;
            }

            max_flow_computations++;
            pcf = compute_pcf(inst, r1_values, r2_values);
            r3 = compute_r3_incremental(inst, max_flow_graph, relax_point, pcf, actions_effect, r3);

            if (r3 >= 1 - exec.lm_min_viol * (1 - initial_r3) - HPLUS_EPSILON) {
                revert_r1r2_changes();
                revert_r3_changes();
                bool has_next = round_next_action();
                if (!has_next) break;
                continue;
            }

            const std::vector<unsigned int>& proposed_landmark = get_r3_violated_landmark(inst, max_flow_graph);
            double proposed_violation{1};
            for (const auto& a : proposed_landmark)
                proposed_violation -= relax_point[a];  // We can use this vector (with rounded actions) and don't need the original one, because
                                                       // rounded actions will never be in a landmark (since their weight is 1)

            if (exec.lm_min_improv) {
                if (proposed_landmark.size() > landmark.size() || (proposed_landmark.size() == landmark.size() && proposed_violation <= violation)) {
                    revert_r1r2_changes();
                    revert_r3_changes();
                    bool has_next = round_next_action();
                    if (!has_next) break;
                    continue;
                }
            }

            landmark = proposed_landmark;
            violation = proposed_violation;

            round_new_action();
        }
    }

    return {true, landmark};
}

[[nodiscard]]
unsigned int relax_cuts::add_lm_cut(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst,
                                    const std::vector<double>& relax_point, unsigned int& n_act_in_lm, unsigned int& n_lm) {
    const auto& [found, landmark]{get_violated_landmark(exec, inst, relax_point, n_act_in_lm, n_lm)};
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
