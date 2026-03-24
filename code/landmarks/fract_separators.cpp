#include "fract_separators.hpp"

#include <set>
#include <stack>
#include <unordered_set>

#include "bs_utils.hpp"
#include "landmark_utils.hpp"
#include "limits.hxx"
#include "lmcut.hpp"
#include "max_flow.hpp"
#include "stats_registry.hxx"
#include "timer.hxx"

namespace {
// TODO: Consider using watch preconditions here aswell
[[nodiscard]]
inline auto compute_r1_r2_incremental(const hplus::instance& inst, const std::vector<double>& relax_point, std::vector<double>& r1_values,
                                      std::vector<double>& r1_act_values, std::vector<double>& r2_values, std::vector<double>& r2_act_values,
                                      BinarySet& state, std::queue<unsigned int>& actions_queue, std::unordered_set<unsigned int>& acts_in_queue,
                                      std::stack<std::pair<unsigned int, double>>& trail) -> double {
    // Trail keys:
    // - [0, inst.n): r1_values
    // - [inst.n, inst.n + inst.m): r1_act_values
    // - [inst.n + inst.m, 2*inst.n + inst.m): r2_values
    // - [2*inst.n + inst.m, 2*inst.n + 2*inst.m): r2_act_values
    std::unordered_set<unsigned int> trail_flags;

    while (!actions_queue.empty()) {
        const auto choice{actions_queue.front()};
        actions_queue.pop();
        acts_in_queue.erase(choice);

        // Store the R1 and R2 values for this action... if we see that there's no update on these values we can skip
        double prev_r1_act_value{r1_act_values[choice]};
        double prev_r2_act_value{r2_act_values[choice]};

        // Values for actions are the minimum value of its precondition, with a cap at it's relaxed value
        r1_act_values[choice] = relax_point[choice];
        r2_act_values[choice] = relax_point[choice];
        for (const auto& pre : inst.actions[choice].pre_sparse) {
            r1_act_values[choice] = std::min(r1_act_values[choice], r1_values[pre]);
            r2_act_values[choice] = std::min(r2_act_values[choice], r2_values[pre]);
        }

        // If neither the R1 or R2 value for this action has changed, we can skip
        if (r1_act_values[choice] - prev_r1_act_value <= HPLUS_EPSILON && r2_act_values[choice] - prev_r2_act_value <= HPLUS_EPSILON) {
            continue;
        }

        // Write to the trail the previous value
        if (r1_act_values[choice] - prev_r1_act_value > HPLUS_EPSILON) {
            unsigned int trail_key = inst.n + choice;
            if (!trail_flags.contains(trail_key)) {
                trail.emplace(trail_key, prev_r1_act_value);
                trail_flags.insert(trail_key);
            }
        }
        if (r2_act_values[choice] - prev_r2_act_value > HPLUS_EPSILON) {
            unsigned int trail_key = (2 * inst.n) + inst.m + choice;
            if (!trail_flags.contains(trail_key)) {
                trail.emplace(trail_key, prev_r2_act_value);
                trail_flags.insert(trail_key);
            }
        }

        state |= inst.actions[choice].eff_sparse;

        // Values for facts are:
        // R1: the maximum among the values of the actions that achieve it
        // R2: the sum of the values of the actions that achieve it
        for (const auto& eff : inst.actions[choice].eff_sparse) {
            // If the R1 value of eff is higher than this action's value we won't need to update R1 values
            // If the R2 value of eff is already 1 we won't need to update R2 values
            if (r1_values[eff] >= r1_act_values[choice] && r2_values[eff] >= 1 - HPLUS_EPSILON) {
                continue;
            }

            // Now we know that either R1 or R2 values will be updated...

            // Update R1 values
            if (r1_values[eff] < r1_act_values[choice]) {
                unsigned int trail_key = eff;
                if (!trail_flags.contains(trail_key)) {
                    trail.emplace(trail_key, r1_values[eff]);
                    trail_flags.insert(trail_key);
                }
                r1_values[eff] = std::max(r1_values[eff], r1_act_values[choice]);
            }
            // Update R2 values
            if (r2_values[eff] < 1 - HPLUS_EPSILON) {
                unsigned int trail_key = inst.n + inst.m + eff;
                if (!trail_flags.contains(trail_key)) {
                    trail.emplace(trail_key, r2_values[eff]);
                    trail_flags.insert(trail_key);
                }
                r2_values[eff] += (r2_act_values[choice] - prev_r2_act_value);
                r2_values[eff] = std::min(r2_values[eff], 1.0);
            }

            // Since either R1 or R2 value for eff has been updated, we need to add to the queue all actions that have it as precondition
            for (const auto& act_i : inst.act_with_pre[eff]) {
                if (!bs_contains(state, inst.actions[act_i].pre_sparse)) {
                    continue;  // Skip actions that can't be applied yet
                }
                if (acts_in_queue.contains(act_i)) {
                    continue;  // Skip actions that are already in the queue
                }
                if (relax_point[act_i] <= HPLUS_EPSILON) {
                    continue;  // Skip actions whose weight is 0
                }
                actions_queue.push(act_i);
                acts_in_queue.insert(act_i);
            }
        }
    }

    double min_r1{1};
    for (const auto gfact : inst.goal) {
        min_r1 = std::min(min_r1, r1_values[gfact]);
    }

    return min_r1;
}

[[nodiscard]]
inline auto compute_pcf(const hplus::instance& inst, const std::vector<double>& r1_values, const std::vector<double>& r2_values)
    -> std::vector<unsigned int> {
    // Choose pcf as the precondition with the lowest R1 value
    std::vector<unsigned int> pcf(inst.m + 1, 0);  // Last spot is reserved for the dummy goal action's pcf
    static const unsigned int source = inst.n;
    static const unsigned int sink_action = inst.m;

    auto check_and_update_pcf = [&](unsigned int act_i, unsigned int fact, double& min_r1, double& min_r2) {
        if (r1_values[fact] < min_r1 - HPLUS_EPSILON) {
            min_r1 = r1_values[fact];
            min_r2 = r2_values[fact];
            pcf[act_i] = fact;
        } else if (std::abs(r1_values[fact] - min_r1) < HPLUS_EPSILON && r2_values[fact] < min_r2 - HPLUS_EPSILON) {  // Use R2 as tie-breaking
            min_r2 = r2_values[fact];
            pcf[act_i] = fact;
        }
    };

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (inst.actions[act_i].pre_sparse.empty()) {
            pcf[act_i] = source;
            continue;
        }
        double min_r1{2};
        double min_r2{2};  // 1 is the max value each R1 or R2 value can have... using 2 as initial min guarantees a lower R1 value
        for (const auto& pre : inst.actions[act_i].pre_sparse) {
            check_and_update_pcf(act_i, pre, min_r1, min_r2);
        }
    }

    // Compute the pcf for the dummy goal action
    double min_r1{2};
    double min_r2{2};
    for (const auto gfact : inst.goal) {
        check_and_update_pcf(sink_action, gfact, min_r1, min_r2);
    }

    return pcf;
}

inline auto max_flow_graph_construction(const hplus::instance& inst) -> std::pair<std::vector<std::vector<network_edge>>, std::vector<unsigned int>> {
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

    auto add_edge = [&](unsigned int node_a, unsigned int node_b, double capacity) {
        const auto edge_ba_idx = static_cast<unsigned int>(graph[node_b].size());
        const auto edge_ab_idx = static_cast<unsigned int>(graph[node_a].size());
        graph[node_a].emplace_back(node_b, edge_ba_idx, capacity, false);  // Forward (residual) edge
        graph[node_b].emplace_back(node_a, edge_ab_idx, 0, true);          // Reverse (used flow) edge
    };

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        // Find this action's effect
        // To prevent duplicates in the adjacency lists always create a dummy node so that we have: pcf -> dummy -> effect(s)
        auto eff_node = static_cast<unsigned int>(graph.size());
        graph.emplace_back();

        actions_effect[act_i] = eff_node;

        // Get all preconditions (linking the source node if needed)
        std::vector<unsigned int> preconditions;
        if (inst.actions[act_i].pre_sparse.empty()) {
            preconditions.push_back(source);
        } else {
            preconditions = inst.actions[act_i].pre_sparse;
        }

        for (const auto& from : preconditions) {
            // This must be adjusted according eff_node the selected pcf and the action's weight in the relax point:
            // w(from, act_eff[act_i]) = (from == pcf[act_i]) ? relax_point[act_i] : 0
            add_edge(from, eff_node, 0);

            for (const auto& eff : inst.actions[act_i].eff_sparse) {
                add_edge(eff_node, eff,
                         1);  // dummy edges have capacity of 1 since they are sourced only by the from -> eff_node edge, which will have
                              // the action's weight... in this way we guarantee the min-cut to only be associated to real actions
                              // and not dummy ones (if capacity is 1, then this won't ever be saturated, unless the minimum cut is >= 1)
            }
        }
    }

    // Adding the edges linking the sink with the rest of the graph
    for (const auto gfact : inst.goal) {
        add_edge(gfact, sink, 0);  // This must be adjusted according the selected pcf
    }
    actions_effect[sink_action] = sink;

    return {graph, actions_effect};
}

[[nodiscard]]
inline auto compute_r3_incremental(const hplus::instance& inst, std::vector<std::vector<network_edge>>& graph, const std::vector<double>& relax_point,
                                   const std::vector<unsigned int>& pcf, const std::vector<unsigned int>& actions_eff, double prev_r3) -> double {
    static const unsigned int source = inst.n;
    static const unsigned int sink = inst.n + 1;
    static const unsigned int sink_action = inst.m;

    double removed_flow = 0;

    // Fix goal's pcf
    for (const auto gfact : inst.goal) {
        unsigned int to_idx = 0;
        for (unsigned int i = 0; i < graph[gfact].size(); i++) {
            if (graph[gfact][i].to == sink) {
                to_idx = i;
                break;
            }
        }
        const unsigned int rev_idx = graph[gfact][to_idx].rev;
        const double flow = graph[sink][rev_idx].c;
        const double res_cap = graph[gfact][to_idx].c;
        const double capacity = flow + res_cap;

        if (gfact == pcf[sink_action]) {
            double delta = 1 - capacity;
            if (delta <= HPLUS_EPSILON) {
                continue;
            }

            graph[gfact][to_idx].c = 1;
        } else {
            if (capacity < HPLUS_EPSILON) {
                continue;
            }

            graph[gfact][to_idx].c = 0;
            if (flow > HPLUS_EPSILON) {
                const double flow_to_remove = graph[sink][rev_idx].c;
                graph[sink][rev_idx].c = 0;
                double removed = flow_removal(graph, source, gfact, flow_to_remove);

                ASSERT(std::abs(flow_to_remove - removed) <= HPLUS_EPSILON);

                removed_flow += removed;
            }
        }
    }

    // Update flow in pcf changes and in fractional point increases
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        std::vector<unsigned int> pre;
        if (inst.actions[act_i].pre_sparse.empty()) {
            pre.push_back(source);
        } else {
            pre = inst.actions[act_i].pre_sparse;
        }

        const unsigned int eff_node = actions_eff[act_i];

        for (const auto& pre_node : pre) {
            // Get info on the current edge
            unsigned int to_idx = 0;
            for (unsigned int i = 0; i < graph[pre_node].size(); i++) {
                if (graph[pre_node][i].to == eff_node) {
                    to_idx = i;
                    break;
                }
            }
            const unsigned int rev_index = graph[pre_node][to_idx].rev;
            double flow = graph[eff_node][rev_index].c;
            double res_cap = graph[pre_node][to_idx].c;
            double capacity = flow + res_cap;

            if (pre_node == pcf[act_i]) {
                double delta = relax_point[act_i] - capacity;

                if (delta > HPLUS_EPSILON) {
                    graph[pre_node][to_idx].c += delta;
                } else if (delta < -HPLUS_EPSILON) {
                    delta = -delta;
                    const double new_residual_capacity = graph[pre_node][to_idx].c - delta;

                    graph[pre_node][to_idx].c = (new_residual_capacity > HPLUS_EPSILON) ? new_residual_capacity : 0;

                    // The residual flow is not enough.. we have to reduce the flow aswell
                    if (new_residual_capacity < -HPLUS_EPSILON) {
                        const double flow_to_remove = -new_residual_capacity;                           // Compute the flow needed to be removed
                        graph[eff_node][rev_index].c -= flow_to_remove;                                 // Remove it from this edge
                        double removed_ahead = flow_removal(graph, eff_node, sink, flow_to_remove);     // Reduce the flow reaching the sink
                        double removed_before = flow_removal(graph, source, pre_node, flow_to_remove);  // Reduce the flow leaving the source

                        ASSERT(std::abs(removed_ahead - removed_before) < HPLUS_EPSILON);

                        removed_flow += removed_ahead;
                    }
                }

            } else {
                if (capacity < HPLUS_EPSILON) {
                    continue;
                }

                graph[pre_node][to_idx].c = 0;
                if (flow > HPLUS_EPSILON) {
                    const double flow_to_remove = flow;
                    graph[eff_node][rev_index].c = 0;
                    double removed_ahead = flow_removal(graph, eff_node, sink, flow_to_remove);
                    double removed_before = flow_removal(graph, source, pre_node, flow_to_remove);

                    ASSERT(std::abs(removed_ahead - removed_before) <= HPLUS_EPSILON);

                    removed_flow += removed_ahead;
                }
            }
        }
    }

    // Remove disconnected parts of the graph
    std::unordered_set<unsigned int> reachable;
    std::queue<unsigned int> to_visit;
    to_visit.push(source);
    reachable.insert(source);

    // Parts connected to the graph
    while (!to_visit.empty()) {
        unsigned int node = to_visit.front();
        to_visit.pop();

        for (const auto& [to, rev, c, is_rev] : graph[node]) {
            // A node is connected with the rest if there's an edge connnected to it, which has flow
            if (!is_rev && graph[to][rev].c > HPLUS_EPSILON && !reachable.contains(to)) {
                reachable.insert(to);
                to_visit.push(to);
            }
        }
    }

    std::vector<unsigned int> not_reachable;
    for (unsigned int i = 0; i < graph.size(); ++i) {
        if (!reachable.contains(i)) {
            not_reachable.push_back(i);
        }
    }
    // Remove parts that have no flow reaching them from the source
    for (const auto& node : not_reachable) {
        for (auto& edge : graph[node]) {
            // If this is a reverse edge (contains flow)
            if (edge.is_reverse && edge.c > HPLUS_EPSILON) {
                const double flow_to_remove = edge.c;
                edge.c = 0;
                graph[edge.to][edge.rev].c += flow_to_remove;
            }
        }
    }

    return prev_r3 - removed_flow + compute_max_flow(graph, source, sink);
}

[[nodiscard]]
inline auto extract_landmark(const hplus::instance& inst, const std::vector<std::vector<network_edge>>& graph)
    -> std::pair<std::vector<unsigned int>, BinarySet> {
    std::unordered_set<unsigned int> graph_reach{get_min_cut_lpartition(graph, inst.n)};
    BinarySet facts_reach(inst.n);
    // This needs to be done since BinarySet check for the capacity of the sets... I need to make sure this has the same capacity of the
    // preconditions and effects of actions
    for (unsigned int i = 0; i < inst.n; i++) {
        if (graph_reach.contains(i)) {
            facts_reach.add(i);
        }
    }

    ASSERT(!facts_reach.superset_of(inst.goal));

    std::vector<unsigned int> landmark;
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (bs_contains(facts_reach, inst.actions[act_i].pre_sparse) && !bs_contains(facts_reach, inst.actions[act_i].eff_sparse)) {
            landmark.push_back(act_i);
        }
    }

    return {landmark, facts_reach};
}
}  // namespace

[[nodiscard]]
auto fract_lm_sep::get_r3_violated_landmark(const hplus::execution& exec, const hplus::instance& inst, std::vector<double> relax_point)
    -> std::pair<bool, std::vector<unsigned int>> {
    auto _fract_lm = make_scoped_timer<"fract_lm_separator">(STATS);
    std::vector<unsigned int> landmark;

    // R1 and R2 data
    std::vector<double> r1_values(inst.n, 0);
    std::vector<double> r1_act_values(inst.m, 0);
    std::vector<double> r2_values(inst.n, 0);
    std::vector<double> r2_act_values(inst.m, 0);
    std::queue<unsigned int> r1r2_actions_queue;
    BinarySet r1r2_state(inst.n);
    std::unordered_set<unsigned int> r1r2_acts_in_queue;
    std::stack<std::pair<unsigned int, double>> r1r2_trail;

    // R3 data
    auto [max_flow_graph, actions_effect] = max_flow_graph_construction(inst);
    std::vector<unsigned int> pcf;
    double r3_val{0};

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (relax_point[act_i] < HPLUS_EPSILON) {
            continue;
        }
        if (inst.actions[act_i].pre_sparse.empty()) {
            r1r2_actions_queue.push(act_i);
            r1r2_acts_in_queue.insert(act_i);
        }
    }

    double r1_val = compute_r1_r2_incremental(inst, relax_point, r1_values, r1_act_values, r2_values, r2_act_values, r1r2_state, r1r2_actions_queue,
                                              r1r2_acts_in_queue, r1r2_trail);
    if (r1_val >= 1 - HPLUS_EPSILON) {
        return {false, {}};
    }

    pcf = compute_pcf(inst, r1_values, r2_values);
    r3_val = compute_r3_incremental(inst, max_flow_graph, relax_point, pcf, actions_effect, r3_val);

    if (r3_val >= 1 - HPLUS_EPSILON) {
        return {false, {}};
    }

    auto [lm_found, reachable_state] = extract_landmark(inst, max_flow_graph);
    landmark = std::move(lm_found);

    if (exec.fractlm_min.find('i') != std::string::npos) {
        unsigned int rounded_act_lmidx{0};
        unsigned int rounded_act{0};
        unsigned int minimization_repetitions{0};
        double prev_act_val{0};
        std::multiset<unsigned int> act_rejected_count;

        // ====================================================== //
        // ======== R1 R2 incremental computation helpers ======= //
        // ====================================================== //
        BinarySet r1r2_prev_state(r1r2_state);
        // Trail keys:
        // - [0, inst.n): r1_values
        // - [inst.n, inst.n + inst.m): r1_act_values
        // - [inst.n + inst.m, 2*inst.n + inst.m): r2_values
        // - [2*inst.n + inst.m, 2*inst.n + 2*inst.m): r2_act_values
        auto revert_r1r2_changes = [&]() {
            while (!r1r2_trail.empty()) {
                const auto [key, value] = r1r2_trail.top();
                r1r2_trail.pop();
                if (key < inst.n) {  // R1 values
                    r1_values[key] = value;
                } else if (key < inst.n + inst.m) {  // R1 act values
                    r1_act_values[key - inst.n] = value;
                } else if (key < 2 * inst.n + inst.m) {  // R2 values
                    r2_values[key - inst.n - inst.m] = value;
                } else {  // R2 act values
                    r2_act_values[key - (2 * inst.n) - inst.m] = value;
                }
            }
            r1r2_state = r1r2_prev_state;
        };

        // ====================================================== //
        // ========= R3 incremental computation helpers ========= //
        // ====================================================== //
        std::vector<std::vector<network_edge>> prev_max_flow_graph = max_flow_graph;
        double prev_r3{r3_val};
        double initial_r3{r3_val};
        double violation{1 - r3_val};
        unsigned int max_flow_computations{0};
        auto revert_r3_changes = [&]() {
            max_flow_graph = prev_max_flow_graph;
            r3_val = prev_r3;
        };

        // ====================================================== //
        // =========== Minimization procedure helpers =========== //
        // ====================================================== //
        auto round_action = [&](int act_idx) {
            rounded_act = landmark[static_cast<size_t>(act_idx)];
            prev_act_val = relax_point[rounded_act];
            relax_point[rounded_act] = 1;
            // R1 R2 preparations
            r1r2_actions_queue.push(rounded_act);
            r1r2_acts_in_queue.insert(rounded_act);
        };

        auto round_next_action = [&]() {
            relax_point[rounded_act] = prev_act_val;
            act_rejected_count.insert(rounded_act);
            rounded_act_lmidx++;
            if (rounded_act_lmidx >= landmark.size()) {
                return false;
            }
            round_action(static_cast<int>(rounded_act_lmidx));
            return true;
        };

        auto sort_landmark = [&]() {
            std::ranges::sort(landmark, [&](unsigned int act_a, unsigned int act_b) {
                if (std::abs(relax_point[act_a] - relax_point[act_b]) <= HPLUS_EPSILON) {
                    // Prefer actions that were rejected less (an action is "rejected" when rounding it causes r3_val to be >= 1)
                    return act_rejected_count.count(act_a) < act_rejected_count.count(act_b);
                }
                return relax_point[act_a] > relax_point[act_b];  // Prefer actions with smaller violation (violation = 1 - fract_value)
            });
        };

        auto round_new_action = [&]() {
            r1r2_prev_state = r1r2_state;
            r1r2_trail = std::stack<std::pair<unsigned int, double>>();
            prev_max_flow_graph = max_flow_graph;
            prev_r3 = r3_val;

            rounded_act_lmidx = 0;
            if (exec.lm_min_sort) {
                sort_landmark();
            }
            round_action(static_cast<int>(rounded_act_lmidx));
        };

        auto terminate_condition = [&]() {
            if (CHECK_STOP()) {  // This has a meaning only inside the cutloop, but it's not a problem because inside of CPLEX it's his job to stop
                return true;
            }

            // If we reached the end of this landmark we have no more actions to round... return the "best" landmark we found
            if (rounded_act_lmidx >= std::min(exec.lm_min_lookahead, static_cast<unsigned int>(landmark.size()))) {
                return true;
            }

            // If we reached our iteration limit, stop
            if (max_flow_computations >= exec.lm_min_it) {
                return true;
            }

            minimization_repetitions++;

            return false;
        };

        // ====================================================== //
        // ============ Start minimization procedure ============ //
        // ====================================================== //
        round_new_action();

        while (!terminate_condition()) {
            r1_val = compute_r1_r2_incremental(inst, relax_point, r1_values, r1_act_values, r2_values, r2_act_values, r1r2_state, r1r2_actions_queue,
                                               r1r2_acts_in_queue, r1r2_trail);
            if (r1_val >= 1 - HPLUS_EPSILON) {
                revert_r1r2_changes();
                bool has_next = round_next_action();
                if (!has_next) {
                    break;
                }
                continue;
            }

            max_flow_computations++;
            pcf = compute_pcf(inst, r1_values, r2_values);
            r3_val = compute_r3_incremental(inst, max_flow_graph, relax_point, pcf, actions_effect, r3_val);

            if (r3_val >= 1 - exec.lm_min_viol * (1 - initial_r3) - HPLUS_EPSILON) {
                revert_r1r2_changes();
                revert_r3_changes();
                bool has_next = round_next_action();
                if (!has_next) {
                    break;
                }
                continue;
            }

            auto [proposed_landmark, reach] = extract_landmark(inst, max_flow_graph);
            double proposed_violation{1};
            for (const auto& act : proposed_landmark) {
                proposed_violation -= relax_point[act];  // We can use this vector (with rounded actions) and don't need the original one, because
                // rounded actions will never be in a landmark (since their weight is 1)
            }

            if (exec.lm_min_improv) {
                if (proposed_landmark.size() > landmark.size() || (proposed_landmark.size() == landmark.size() && proposed_violation <= violation)) {
                    revert_r1r2_changes();
                    revert_r3_changes();
                    bool has_next = round_next_action();
                    if (!has_next) {
                        break;
                    }
                    continue;
                }
            }

            landmark = std::move(proposed_landmark);
            reachable_state = std::move(reach);
            violation = proposed_violation;

            round_new_action();
        }
    }
    if (exec.fractlm_min.find('c') != std::string::npos) {
        std::vector<unsigned int> unapplicable_actions;
        for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
            if (!bs_contains(reachable_state, inst.actions[act_i].pre_sparse)) {
                unapplicable_actions.push_back(act_i);
            }
        }
        lmutils::landmark_minimalization(inst, landmark, unapplicable_actions, reachable_state);
    }
    // Greedy minimalization is missing because it's unnecessary by construction of the base landmark extracted

    STATS.gauge_record<"fract_lm_size">(static_cast<double>(landmark.size()));

    return {true, landmark};
}

auto fract_lm_sep::get_lmcut_violated_landmarks(const hplus::execution& exec, const hplus::instance& inst, const std::vector<double>& relax_point)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>> {
    auto _fract_lm = make_scoped_timer<"fract_lm_separator">(STATS);
    LMcut lmcut(inst, exec.fractlm_min.find('g') != std::string::npos, exec.fractlm_min.find('c') != std::string::npos);
    std::vector<double> actions_weights(relax_point.begin(), relax_point.begin() + inst.m);
    const auto& [found, landmarks] = lmcut.fract_separation(actions_weights, hmax::hmax_arbitrary);

    for (const auto& landmark : landmarks) {
        STATS.gauge_record<"fract_lm_size">(static_cast<double>(landmark.size()));
    }

    return {found, landmarks};
}
