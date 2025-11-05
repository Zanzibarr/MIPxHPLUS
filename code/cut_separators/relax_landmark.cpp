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
            if (r2_values[p] < 1 - HPLUS_EPSILON) r2_values[p] += (act_r2_values[choice] - prev_r2_act_value);

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

    // TODO: R2 computation here is just to make sure this is never wrong... remove after testing
    double r1{1}, r2{1};
    for (const auto& p : inst.goal) {
        r1 = std::min(r1, r1_values[p]);
        r2 = std::min(r2, r2_values[p]);
    }

    // TODO: Remove after testing
    ASSERT(r1 <= r2 + HPLUS_EPSILON);                      // Making sure that R1 <= R2
    if (r1 <= HPLUS_EPSILON) ASSERT(r2 <= HPLUS_EPSILON);  // Making sure that if R1 == 0, than also R2 == 0

    return {r1, r1_values, r2_values};
}

[[nodiscard]] static inline std::pair<std::vector<std::vector<network_edge>>, std::vector<std::tuple<unsigned int, unsigned int, unsigned int>>>
build_max_flow_graph(const hplus::instance& inst, const std::vector<double>& actions_weights, const std::vector<double>& r1_values,
                     const std::vector<double>& r2_values) {
    // Choose pcf as the precondition with the lowest R1 value
    std::vector<int> pcf(inst.m, -1);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        double min_r1{2}, min_r2{2};  // 1 is the max value each R1 or R2 value can have... using 2 as initial min guarantees a lower R1 value
        for (const auto& p : inst.actions[act_i].pre_sparse) {
            if (r1_values[p] < min_r1) {
                min_r1 = r1_values[p];
                min_r2 = r2_values[p];
                pcf[act_i] = p;
            } else if (r1_values[p] == min_r1 && r2_values[p] < min_r2) {  // Use R2 as tie-breaking
                min_r2 = r2_values[p];
                pcf[act_i] = p;
            }
        }
    }

    std::vector<std::vector<network_edge>> graph(inst.n + 2);
    std::vector<std::tuple<unsigned int, unsigned int, unsigned int>> edges;  // Here I store the edges
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
                edges.emplace_back(from, inst.actions[act_i].eff_sparse[0], act_i);
                break;
            default:  // Multiple effects -> Create one edge a -> dummy and multiple edges dummy -> effect
                const unsigned int dummy{static_cast<unsigned int>(graph.size())};
                graph.push_back(std::vector<network_edge>());  // Create new dummy node
                add_edge(from, dummy, actions_weights[act_i]);
                for (const auto& to : inst.actions[act_i].eff_sparse) {
                    add_edge(dummy, to, actions_weights[act_i]);
                    edges.emplace_back(from, to, act_i);
                }
                break;
        }
    }

    // Compute the pcf for the dummy goal action
    double min_r1{2}, min_r2{2};
    unsigned int g_pcf{0};
    for (const auto& g : inst.goal) {
        if (r1_values[g] < min_r1) {
            min_r1 = r1_values[g];
            min_r2 = r2_values[g];
            g_pcf = g;
        } else if (r1_values[g] == min_r1 && r2_values[g] < min_r2) {  // Use R2 as tie-breaking
            min_r2 = r2_values[g];
            g_pcf = g;
        }
    }
    add_edge(g_pcf, sink, 1);

    return {graph, edges};
}

[[nodiscard]]
static inline std::tuple<double, std::vector<std::vector<network_edge>>, std::vector<std::tuple<unsigned int, unsigned int, unsigned int>>>
compute_r3(const hplus::instance& inst, const std::vector<double>& relax_point, const std::vector<double>& r1_values,
           const std::vector<double>& r2_values) {
    auto [graph, edges] = build_max_flow_graph(inst, relax_point, r1_values, r2_values);
    return {compute_max_flow(graph, inst.n, inst.n + 1), graph, edges};
}

[[nodiscard]]
static inline std::vector<unsigned int> get_r3_violated_landmark(const hplus::instance& inst, const std::vector<std::vector<network_edge>>& graph,
                                                                 const std::vector<std::tuple<unsigned int, unsigned int, unsigned int>>& edges) {
    binary_set reachable{get_min_cut(graph, inst.n)};
    std::unordered_set<unsigned int> landmark;
    for (const auto& [from, to, label] : edges) {
        if (reachable[from] && !reachable[to]) landmark.insert(label);
    }
    return std::vector<unsigned int>(landmark.begin(), landmark.end());
}

[[nodiscard]]
std::pair<bool, std::vector<unsigned int>> relax_cuts::get_violated_landmark(const hplus::instance& inst, const std::vector<double>& relax_point) {
    const auto& [r1, r1_values, r2_values]{compute_r1_r2(inst, relax_point)};

    // If R1 >= 1 there's no violated landmark
    if (r1 >= 1 - HPLUS_EPSILON) return {false, {}};

    // Otherwise, we rely on R3 to understand wether there's a violated landmark...

    const auto& [r3, r3_graph, r3_edges]{compute_r3(inst, relax_point, r1_values, r2_values)};

    // TODO: Remove after testing
    if (r1 <= HPLUS_EPSILON) ASSERT(r3 <= HPLUS_EPSILON);  // Making sure that if R1 == 0, than also R3 == 0

    // Note: if R3 < 1, then there's a violated landmark, BUT if R3 == 1 we can't assume that no landmark is violated... in this case we simply ignore
    // the possible landmark Statistically speaking, R3 == 1 but there exists a violated landmark happens in 3% of cases
    if (r3 >= 1 - HPLUS_EPSILON) return {false, {}};

    // TODO: Test to ensure we are not creating wrong cutting planes
    return {true, get_r3_violated_landmark(inst, r3_graph, r3_edges)};
}

[[nodiscard]]
unsigned int relax_cuts::add_lm_cut(CPXCALLBACKCONTEXTptr context, const hplus::instance& inst, const std::vector<double>& relax_point) {
    const auto& [found, landmark]{get_violated_landmark(inst, relax_point)};
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
