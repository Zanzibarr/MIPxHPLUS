#include <list>

#include "relax_callback.hpp"

[[nodiscard]]
static inline std::pair<double, std::vector<double>> compute_r1(const hplus::instance& inst, const std::vector<double>& relax_point) {
    std::vector<double> var_values(inst.n, 0), act_values(inst.m, 0);
    std::list<unsigned int> actions_queue;
    binary_set state(inst.n), act_in_queue(inst.m);

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (relax_point[act_i] <= HPLUS_EPSILON) continue;
        if (inst.actions[act_i].pre_sparse.empty()) {
            actions_queue.push_back(act_i);
            act_in_queue.add(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const auto choice{actions_queue.front()};
        actions_queue.pop_front();
        act_in_queue.remove(choice);

        act_values[choice] = relax_point[choice];
        for (const auto& p : inst.actions[choice].pre_sparse) act_values[choice] = std::min(act_values[choice], var_values[p]);

        if (act_values[choice] <= HPLUS_EPSILON) continue;

        state |= inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff_sparse) {
            if (var_values[p] >= act_values[choice]) continue;
            var_values[p] = std::max(var_values[p], act_values[choice]);
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (!state.contains(inst.actions[act_i].pre)) continue;
                if (act_in_queue[act_i]) continue;
                if (relax_point[act_i] <= HPLUS_EPSILON) continue;
                actions_queue.push_back(act_i);
                act_in_queue.add(act_i);
            }
        }
    }

    double r1{1};
    for (const auto& p : inst.goal) r1 = std::min(r1, var_values[p]);

    return {r1, var_values};
}

[[nodiscard]]
static inline std::pair<double, std::vector<double>> compute_r2(const hplus::instance& inst, const std::vector<double>& relax_point) {
    std::vector<double> var_values(inst.n, 0), act_values(inst.m, 0);
    std::list<unsigned int> actions_queue;
    binary_set state(inst.n), act_in_queue(inst.m);

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (relax_point[act_i] <= HPLUS_EPSILON) continue;
        if (inst.actions[act_i].pre_sparse.empty()) {
            actions_queue.push_back(act_i);
            act_in_queue.add(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const auto choice{actions_queue.front()};
        actions_queue.pop_front();
        act_in_queue.remove(choice);

        double pre_update_act_value{act_values[choice]};

        act_values[choice] = relax_point[choice];
        for (const auto& p : inst.actions[choice].pre_sparse) act_values[choice] = std::min(act_values[choice], var_values[p]);

        if (act_values[choice] - pre_update_act_value <= HPLUS_EPSILON) continue;

        state |= inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff_sparse) {
            if (var_values[p] >= 1 - HPLUS_EPSILON) continue;
            var_values[p] += (act_values[choice] - pre_update_act_value);
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (!state.contains(inst.actions[act_i].pre)) continue;
                if (act_in_queue[act_i]) continue;
                if (relax_point[act_i] <= HPLUS_EPSILON) continue;
                actions_queue.push_back(act_i);
                act_in_queue.add(act_i);
            }
        }
    }

    double r2{1};
    for (const auto& p : inst.goal) r2 = std::min(r2, var_values[p]);

    return {r2, var_values};
}

typedef struct {
    unsigned int to, rev;
    double c;
} network_edge;

static inline std::vector<std::vector<network_edge>> build_max_flow_graph(const hplus::instance& inst, const std::vector<double>& actions_weights,
                                                                          const std::vector<double>& r1_values/*,
                                                                          const std::vector<double>& r2_values*/) {
    // Choose pcf as the precondition with the lowest r1 value
    std::vector<int> pcf(inst.m, -1);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        double min_r1{2} /*, min_r2{2}*/;  // 1 is the max value each r1 value can have... using 2 as initial min guarantees a lower r1 value
        for (const auto& p : inst.actions[act_i].pre_sparse) {
            if (r1_values[p] < min_r1) {
                min_r1 = r1_values[p];
                // min_r2 = r2_values[p];
                pcf[act_i] = p;
                // } else if (r1_values[p] == min_r1 && r2_values[p] < min_r2) {
                //     min_r2 = r2_values[p];
                //     pcf[act_i] = p;
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
            case 0:
                break;
            case 1:
                add_edge(from, inst.actions[act_i].eff_sparse[0], actions_weights[act_i]);
                break;
            default:
                const unsigned int dummy{static_cast<unsigned int>(graph.size())};
                graph.push_back(std::vector<network_edge>());  // Create new dummy node
                add_edge(from, dummy, actions_weights[act_i]);
                for (const auto& to : inst.actions[act_i].eff_sparse) add_edge(dummy, to, actions_weights[act_i]);
                break;
        }
    }

    double min_r1{2} /*, min_r2{2}*/;
    unsigned int g_pcf{0};
    for (const auto& g : inst.goal) {
        if (r1_values[g] < min_r1) {
            min_r1 = r1_values[g];
            // min_r2 = r2_values[g];
            g_pcf = g;
            // } else if (r1_values[g] == min_r1 && r2_values[g] < min_r2) {
            //     min_r2 = r2_values[g];
            //     g_pcf = g;
        }
    }
    add_edge(g_pcf, sink, 1);

    return graph;
}

static inline bool max_flow_bfs(std::vector<std::vector<network_edge>>& graph, unsigned int s, unsigned int t, unsigned int n,
                                std::vector<int>& level) {
    level.assign(n, -1);
    std::queue<int> q;
    level[s] = 0;
    q.push(s);

    while (!q.empty()) {
        const auto v{q.front()};
        q.pop();
        for (const auto& e : graph[v]) {
            if (e.c > HPLUS_EPSILON && level[e.to] < -HPLUS_EPSILON) {
                level[e.to] = level[v] + 1;
                q.push(e.to);
            }
        }
    }

    return level[t] >= 0;
}

static inline double max_flow_dfs(std::vector<std::vector<network_edge>>& graph, unsigned int v, unsigned int t, double f, std::vector<int>& level,
                                  std::vector<int>& iter) {
    if (v == t) return f;

    for (int& i = iter[v]; i < static_cast<int>(graph[v].size()); i++) {
        auto& e = graph[v][i];
        if (e.c > HPLUS_EPSILON && level[v] < level[e.to]) {
            double d{max_flow_dfs(graph, e.to, t, std::min(f, e.c), level, iter)};
            if (d > HPLUS_EPSILON) {
                e.c -= d;
                graph[e.to][e.rev].c += d;
                return d;
            }
        }
    }
    return 0;
}

static inline double compute_max_flow(std::vector<std::vector<network_edge>>& graph, unsigned int source, unsigned int sink) {
    double flow{0};
    const unsigned int n{static_cast<unsigned int>(graph.size())};
    std::vector<int> level(n), iter(n);
    while (max_flow_bfs(graph, source, sink, n, level)) {
        iter.assign(n, 0);
        double f;
        while ((f = max_flow_dfs(graph, source, sink, std::numeric_limits<double>::infinity(), level, iter)) > HPLUS_EPSILON) flow += f;
    }
    return flow;
}

[[nodiscard]]
static inline double compute_r3(const hplus::instance& inst, const std::vector<double>& relax_point, const std::vector<double>& r1_values/*,
                                const std::vector<double>& r2_values*/) {
    auto graph = build_max_flow_graph(inst, relax_point, r1_values /*, r2_values*/);

    return compute_max_flow(graph, inst.n, inst.n + 1);
}

[[nodiscard]]
static inline std::tuple<double, double, double> compute_r1_r2_r3(const hplus::instance& inst, const std::vector<double>& relax_point) {
    const auto& [r1, r1_values]{compute_r1(inst, relax_point)};
    if (r1 >= 1) return {1, 1, 1};
    const auto& [r2, r2_values]{compute_r2(inst, relax_point)};
    double r3{compute_r3(inst, relax_point, r1_values /*, r2_values*/)};
    return {r1, r2, r3};
}

[[nodiscard]]
std::pair<bool, std::vector<unsigned int>> relax_cuts::get_violated_landmark(CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec,
                                                                             const hplus::instance& inst, const std::vector<double>& relax_point) {
    // Set the time limit for the flmdet model
    if (exec.timelimit > 0 && static_cast<double>(exec.timelimit) > GET_TIME()) {
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_TimeLimit, static_cast<double>(exec.timelimit) - GET_TIME()));
    } else
        throw timelimit_exception("Reached time limit.");

    // Update the objective function -> adapt the model to the current relaxed solution
    std::vector<int> ind(inst.m);
    std::iota(ind.begin(), ind.end(), 0);
    CPX_HANDLE_CALL(CPXchgobj(env, lp, inst.m, ind.data(), relax_point.data()));

    // Solve the flmdet model
    CPX_HANDLE_CALL(CPXmipopt(env, lp));

    // Get status and solution
    int status = CPXgetstat(env, lp);
    if (status != CPXMIP_OPTIMAL && status != CPXMIP_OPTIMAL_TOL) return {false, {}};  // For time-limit breaching -> no solution found
    double cutval{CPX_INFBOUND};
    CPX_HANDLE_CALL(CPXgetobjval(env, lp, &cutval));

    // TODO: Remove, this is only for testing

    // Compute R1 and R2
    const auto [r1, r2, r3] = compute_r1_r2_r3(inst, relax_point);

    // Get number of nodes used to solve the flmdet model
    int nodes{CPXgetnodecnt(env, lp)};

    // Print statistics
    LOG_DEBUG << "(nodes: " << std::setw(4) << nodes << ") R1 / R2 / R3 / FLMDET : " << std::fixed << std::setprecision(5) << r1 << " / "
              << std::fixed << std::setprecision(5) << r2 << " / " << std::fixed << std::setprecision(5) << r3 << " / " << std::fixed
              << std::setprecision(5) << cutval << (r1 <= HPLUS_EPSILON ? " *" : "");

    // If the value of the cut is greater than 1, than no landmark has been violated
    if (cutval >= 1 - HPLUS_EPSILON) return {false, {}};

    // Now we know that a landmark is violated

    std::vector<double> facts_partition(inst.n);
    CPX_HANDLE_CALL(CPXgetx(env, lp, facts_partition.data(), inst.m, inst.m + inst.n - 1));
    binary_set reach(inst.n);
    // Obtain the reach side of the cut (variables set to 1)
    for (unsigned int p = 0; p < inst.n; ++p) {
        if (facts_partition[p] >= HPLUS_CPX_INT_ROUNDING) reach.add(p);
    }
    // Actions crossing the cut are those that have all preconditions in the left side, and at least one effect in the right side
    std::vector<unsigned int> landmark;
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (reach.contains(inst.actions[act_i].pre) && !reach.contains(inst.actions[act_i].eff)) landmark.push_back(act_i);
    }

    return {true, landmark};
}

[[nodiscard]]
unsigned int relax_cuts::lm(CPXCALLBACKCONTEXTptr context, CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec, const hplus::instance& inst,
                            const std::vector<double>& relax_point) {
    const auto& [found, landmark]{get_violated_landmark(env, lp, exec, inst, relax_point)};
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
