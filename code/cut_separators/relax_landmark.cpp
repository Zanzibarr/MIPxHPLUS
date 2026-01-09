#include <list>

#include "relax_callback.hpp"

[[nodiscard]]
static inline std::tuple<double, std::vector<double>, std::vector<double>> compute_r1_r2(const hplus::instance& inst,
                                                                                         const std::vector<double>& relax_point) {
    std::vector<double> r1_values(inst.n, 0), r2_values(inst.n, 0), act_r1_values(inst.m, 0), act_r2_values(inst.m, 0);
    std::list<unsigned int> actions_queue;
    binary_set state(inst.n), act_in_queue(inst.m);

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        // Skip actions whose weight is 0
        if (relax_point[act_i] <= HPLUS_EPSILON) continue;

        // Actions with no preconditions are the first to be added to the queue... we are performing a forward reachability analysis
        if (inst.actions[act_i].pre_sparse.empty()) {
            actions_queue.push_back(act_i);
            act_in_queue.add(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const auto choice{actions_queue.front()};
        actions_queue.pop_front();
        act_in_queue.remove(choice);

        // Store the R1 and R2 values for this action... if we see that there's no update on these values we can skip
        double prev_r1_act_value{act_r1_values[choice]}, prev_r2_act_value{act_r2_values[choice]};

        // Values for actions are the minimum value of its precondition, with a cap at it's relaxed value
        act_r1_values[choice] = relax_point[choice];
        act_r2_values[choice] = relax_point[choice];
        for (const auto& p : inst.actions[choice].pre_sparse) {
            act_r1_values[choice] = std::min(act_r1_values[choice], r1_values[p]);
            act_r2_values[choice] = std::min(act_r2_values[choice], r2_values[p]);
        }

        // If neither the R1 or R2 value for this action has changed, we can skip
        if (act_r1_values[choice] - prev_r1_act_value <= HPLUS_EPSILON && act_r2_values[choice] - prev_r2_act_value <= HPLUS_EPSILON) continue;

        state |= inst.actions[choice].eff;

        // Values for facts are:
        // R1: the maximum among the values of the actions that achieve it
        // R2: the sum of the values of the actions that achieve it
        for (const auto& p : inst.actions[choice].eff_sparse) {
            // If the R1 value of p is higher than this action's value we won't need to update R1 values
            // If the R2 value of p is already 1 we won't need to update R2 values
            if (r1_values[p] >= act_r1_values[choice] && r2_values[p] >= 1 - HPLUS_EPSILON) continue;

            // Now we know that either R1 or R2 values will be updated...

            // Update R1 values
            if (r1_values[p] < act_r1_values[choice]) r1_values[p] = std::max(r1_values[p], act_r1_values[choice]);
            // Update R2 values
            if (r2_values[p] < 1 - HPLUS_EPSILON) {
                r2_values[p] += (act_r2_values[choice] - prev_r2_act_value);
                r2_values[p] = std::min(r2_values[p], 1.0);
            }

            // Since either R1 or R2 value for p has been updated, we need to add to the queue all actions that have it as precondition
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (!state.contains(inst.actions[act_i].pre)) continue;  // Skip actions that can't be applied yet
                if (act_in_queue[act_i]) continue;                       // Skip actions that are already in the queue
                if (relax_point[act_i] <= HPLUS_EPSILON) continue;       // Skip actions whose weight is 0
                actions_queue.push_back(act_i);
                act_in_queue.add(act_i);
            }
        }
    }

    double r1{1};
    for (const auto& p : inst.goal) r1 = std::min(r1, r1_values[p]);

    return {r1, r1_values, r2_values};
}

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
static inline std::vector<std::vector<network_edge>> build_max_flow_graph(const hplus::instance& inst, const std::vector<double>& actions_weights,
                                                                          const std::vector<double>& r1_values,
                                                                          const std::vector<double>& r2_values) {
    // Choose pcf as the precondition with the lowest R1 value
    std::vector<int> pcf(inst.m, -1);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        double min_r1{2}, min_r2{2};  // 1 is the max value each R1 or R2 value can have... using 2 as initial min guarantees a lower R1 value
        for (const auto& p : inst.actions[act_i].pre_sparse) {
            if (r1_values[p] < min_r1 - HPLUS_EPSILON) {
                min_r1 = r1_values[p];
                min_r2 = r2_values[p];
                pcf[act_i] = p;
            } else if (std::abs(r1_values[p] - min_r1) < HPLUS_EPSILON && r2_values[p] < min_r2 - HPLUS_EPSILON) {  // Use R2 as tie-breaking
                min_r2 = r2_values[p];
                pcf[act_i] = p;
            }
        }
    }

    std::vector<std::vector<network_edge>> graph(inst.n + 2);
    // nodes [0 -> n - 1] => facts
    // node [n] => source
    static const unsigned int source{inst.n};
    // node [n + 1] => sink
    static const unsigned int sink{inst.n + 1};
    // nodes [n + 2 -> ...] => dummy nodes for actions effects (at most m)
    // estimated number of nodes: O(n + m)

    auto add_edge = [&](unsigned int from, unsigned int to, double capacity) {
        graph[from].emplace_back(to, graph[to].size(), capacity);
        graph[to].emplace_back(from, graph[from].size() - 1, 0);  // Reverse edge
    };

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        const unsigned int from = pcf[act_i] < 0 ? source : static_cast<unsigned int>(pcf[act_i]);
        switch (inst.actions[act_i].eff_sparse.size()) {
            case 0:  // No effect -> No edge
                break;
            case 1:  // One effect -> Simple edge
                add_edge(from, inst.actions[act_i].eff_sparse[0], actions_weights[act_i]);
                break;
            default:  // Multiple effects -> Create one edge a -> dummy and multiple edges dummy -> effect
                const unsigned int dummy{static_cast<unsigned int>(graph.size())};
                graph.push_back(std::vector<network_edge>());  // Create new dummy node
                add_edge(from, dummy, actions_weights[act_i]);
                for (const auto& to : inst.actions[act_i].eff_sparse) add_edge(dummy, to, 1);
                break;
        }
    }

    // Compute the pcf for the dummy goal action
    double min_r1{2}, min_r2{2};
    unsigned int g_pcf{0};
    for (const auto& g : inst.goal) {
        if (r1_values[g] < min_r1 - HPLUS_EPSILON) {
            min_r1 = r1_values[g];
            min_r2 = r2_values[g];
            g_pcf = g;
        } else if (std::abs(r1_values[g] - min_r1) < HPLUS_EPSILON && r2_values[g] < min_r2 - HPLUS_EPSILON) {  // Use R2 as tie-breaking
            min_r2 = r2_values[g];
            g_pcf = g;
        }
    }
    add_edge(g_pcf, sink, 1);

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
    unsigned int rounded_act_lmidx{0};
    double prev_act_val{relax_point_copy[rounded_act_lmidx]};
    bool found{false};

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
    std::vector<std::vector<network_edge>> max_flow_graph;
    double r3;
    auto revert_r3_changes = []() {};

    // TODO: Remove... just for debugging
    size_t initial_lm_size, final_lm_size;
    double initial_violation, final_violation, start_time = GET_TIME(), normal_time, minimization_time;
    unsigned int repetitions{0};

    // ====================================================== //
    // =============== Minimization procedure =============== //
    // ====================================================== //
    while (rounded_act_lmidx < landmark.size()) {
        repetitions++;

        // ~~~~~~~~~~ R1/R2 COMPUTATION ~~~~~~~~~~ //
        // TODO: Remove computation from scratch... just for debugging
        const auto& [manual_r1, manual_r1_values, manual_r2_values]{compute_r1_r2(inst, relax_point_copy)};
        r1r2_trail = std::stack<std::pair<unsigned int, double>>();
        r1r2_reversing_state = r1r2_state;
        compute_r1_r2_incremental(inst, relax_point_copy, r1_values, r1_act_values, r2_values, r2_act_values, r1r2_state, r1r2_actions_queue,
                                  r1r2_acts_in_queue, r1r2_trail);
        double r1{1};
        for (const auto& p : inst.goal) r1 = std::min(r1, r1_values[p]);

        for (unsigned int i = 0; i < inst.n; i++) {
            ASSERT(std::abs(manual_r1_values[i] - r1_values[i]) < HPLUS_EPSILON);
            ASSERT(std::abs(manual_r2_values[i] - r2_values[i]) < HPLUS_EPSILON);
        }

        // If R1 >= 1 there's no violated landmark
        if (r1 >= 1 - HPLUS_EPSILON) {
            if (found) {
                revert_r1r2_changes();
                relax_point_copy[landmark[rounded_act_lmidx]] = prev_act_val;
                rounded_act_lmidx++;
                if (rounded_act_lmidx >= landmark.size()) break;
                prev_act_val = relax_point_copy[landmark[rounded_act_lmidx]];
                relax_point_copy[landmark[rounded_act_lmidx]] = 1;
                r1r2_actions_queue.push(landmark[rounded_act_lmidx]);
                r1r2_acts_in_queue.add(landmark[rounded_act_lmidx]);
                continue;
            } else
                return {false, {}};
        }

        // Otherwise, we rely on R3 to understand wether there's a violated landmark...

        // ~~~~~~~~~~~~ R3 COMPUTATION ~~~~~~~~~~~ //
        // if (!found) {
        std::vector<std::vector<network_edge>> manual_max_flow_graph =
            build_max_flow_graph(inst, relax_point_copy, manual_r1_values, manual_r2_values);
        double manual_r3 = compute_max_flow(manual_max_flow_graph, inst.n, inst.n + 1);

        max_flow_graph = build_max_flow_graph(inst, relax_point_copy, r1_values, r2_values);
        r3 = compute_max_flow(max_flow_graph, inst.n, inst.n + 1);

        ASSERT(std::abs(manual_r3 - r3) < HPLUS_EPSILON);
        // } else {
        //     TODO: Find a way to track the single (dummy or not) effect of an action -> that's the eff[act] I'm referring to here
        //     TODO: Add a trail so that I can revert changes easily
        //     TODO: Increment chosen action's weight
        //     incremental_edge_insertion(max_flow_graph, inst.n, inst.n + 1, pcf[act_to_round], eff[act_to_round], 1 - prev_act_val);
        //     TODO: Change pcf
        //     for (act s.t. pcf[act] changed) {
        //         incremental_edge_deletion(max_flow_graph, inst.n, inst.n + 1, old_pcf[act], eff[act], relax_point_copy[act])
        //         incremental_edge_insertion(max_flow_graph, inst.n, inst.n + 1, new_pcf[act], eff[act], relax_point_copy[act])
        //     }
        // }

        // Note: if R3 < 1, then there's a violated landmark, BUT if R3 == 1 we can't assume that no landmark is violated... in this case we simply
        // ignore the possible landmark Statistically speaking, R3 == 1 but there exists a violated landmark happens in 3% of cases
        if (r3 >= 1 - HPLUS_EPSILON) {
            if (found) {
                revert_r1r2_changes();
                revert_r3_changes();
                relax_point_copy[landmark[rounded_act_lmidx]] = prev_act_val;
                rounded_act_lmidx++;
                if (rounded_act_lmidx >= landmark.size()) break;
                prev_act_val = relax_point_copy[landmark[rounded_act_lmidx]];
                relax_point_copy[landmark[rounded_act_lmidx]] = 1;
                r1r2_actions_queue.push(landmark[rounded_act_lmidx]);
                r1r2_acts_in_queue.add(landmark[rounded_act_lmidx]);
                continue;
            } else
                return {false, {}};
        }

        // ~~~~~~~ GET LANDMARK AS MIN-CUT ~~~~~~~ //
        std::vector<unsigned int> manual_landmark = get_r3_violated_landmark(inst, manual_max_flow_graph);
        landmark = get_r3_violated_landmark(inst, max_flow_graph);

        ASSERT(manual_landmark.size() == landmark.size());
        for (unsigned int i = 0; i < manual_landmark.size(); i++) {
            ASSERT(manual_landmark[i] == landmark[i]);
        }

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
        // TODO: Maybe an heuristic to tell which actios to use first?? (If done, update it also in the revert sections)
        found = true;
        rounded_act_lmidx = 0;
        prev_act_val = relax_point_copy[landmark[rounded_act_lmidx]];
        relax_point_copy[landmark[rounded_act_lmidx]] = 1;
        r1r2_actions_queue.push(landmark[rounded_act_lmidx]);
        r1r2_acts_in_queue.add(landmark[rounded_act_lmidx]);

        if (!exec.min_fract_lm) break;
    }

    // TODO: Remove... just for debugging
    minimization_time = (GET_TIME() - start_time) * 1000;
    if (exec.min_fract_lm)
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
