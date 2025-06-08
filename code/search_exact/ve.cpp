#include <set>
#include <tuple>

#include "../external/pq.hpp"
#include "../utils/algorithms.hpp"
#include "exact.hpp"

void ve::add_acyclicity_constraints(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Adding acyclicity constraints for VE model";

    const auto stopcheck = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    // ====================================================== //
    // ================= VERTEX ELIMINATION ================= //
    // ====================================================== //

    // Initialize data structures
    std::vector<std::set<unsigned int>> graph(inst.n);
    inst.veg_cumulative_graph.resize(inst.n);
    std::vector<std::tuple<unsigned int, unsigned int, unsigned int>> triangles_list;
    priority_queue<unsigned int> nodes_queue(2 * inst.n);
    std::vector<unsigned int> degree_counter(inst.n, 0);

    // Build initial graph G_0
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            for (const auto& var_j : inst.actions[act_i].eff_sparse) {
                if (var_i == var_j) [[unlikely]] {
                    continue;
                }

                // Insert edge if it doesn't exist already
                if (const auto [iter, inserted] = graph[var_i].insert(var_j); inserted) {
                    ++degree_counter[var_i];
                    ++degree_counter[var_j];
                }
            }

            // Update cumulative graph with all effects
            for (const auto& eff_var : inst.actions[act_i].eff_sparse) {
                insert_sorted(inst.veg_cumulative_graph[var_i], eff_var);
            }
        }
    }

    stopcheck();

    // Initialize priority queue with nodes that have edges
    for (unsigned int node_i = 0; node_i < inst.n; ++node_i) {
        if (degree_counter[node_i] > 0) [[likely]] {
            nodes_queue.push(node_i, degree_counter[node_i]);
        }
    }

    // Apply minimum degree heuristics for G_i
    for ([[maybe_unused]] unsigned int _ = 0; _ < inst.n; ++_) {
        if (nodes_queue.empty()) [[unlikely]] {
            break;
        }

        const unsigned int idx = nodes_queue.top();
        nodes_queue.pop();

        // Graph structure:
        // | \ > |
        // p -> idx -> q
        // | / > |

        std::set<unsigned int> new_nodes;

        // Process all predecessors of idx
        for (unsigned int p = 0; p < inst.n; ++p) {
            if (graph[p].find(idx) == graph[p].end()) {
                continue;
            }

            // Connect p to all successors of idx
            for (const auto& q : graph[idx]) {
                if (p == q) [[unlikely]] {
                    continue;
                }

                // Add edge p -> q if it doesn't exist
                if (const auto [iter, inserted] = graph[p].insert(q); inserted) {
                    ++degree_counter[p];
                    ++degree_counter[q];
                }

                // Update cumulative graph
                insert_sorted(inst.veg_cumulative_graph[p], q);
            }

            // Remove edge p -> idx
            graph[p].erase(idx);
            --degree_counter[p];
            new_nodes.insert(p);

            // Add triangles involving p, idx, and successors of idx
            for (const auto& q : graph[idx]) {
                if (p != q) [[likely]] {
                    triangles_list.emplace_back(p, idx, q);
                }
            }
        }

        // Remove all edges from idx and update degrees
        for (const auto& q : graph[idx]) {
            --degree_counter[q];
            new_nodes.insert(q);
        }

        graph[idx].clear();
        degree_counter[idx] = 0;

        // Update priority queue for affected nodes
        std::for_each(new_nodes.begin(), new_nodes.end(), [&](const auto& x) {
            if (degree_counter[x] > 0 && nodes_queue.has(x)) {
                nodes_queue.change(x, degree_counter[x]);
            }
        });

        stopcheck();
    }

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    const unsigned int veg_start{static_cast<unsigned int>(CPXgetnumcols(env, lp))};

    std::vector<double> objs(inst.n, 0.0);
    std::vector<double> lbs(inst.n, 0.0);
    std::vector<double> ubs(inst.n, 1.0);
    std::vector<char> types(inst.n, 'B');

    inst.veg_starts.resize(inst.n);

    unsigned int count{0};
    for (unsigned int var_i = 0; var_i < inst.n; var_i++) {
        inst.veg_starts[var_i] = count;
        CPX_HANDLE_CALL(CPXnewcols(env, lp, inst.veg_cumulative_graph[var_i].size(), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
        count += inst.veg_cumulative_graph[var_i].size();
        stopcheck();
    }

    stats.var_acyc = count;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const unsigned int fa_start{inst.m};  // Look at base model
    const auto get_fa_idx = [&inst, &fa_start](unsigned int act_idx, unsigned int var_count) {
        return static_cast<int>(fa_start + inst.fadd_cpx_start[act_idx] + var_count);
    };
    const auto get_veg_idx = [&inst, &veg_start](unsigned int var_i, unsigned int var_j) {
        return static_cast<int>(veg_start + inst.veg_starts[var_i] + static_cast<unsigned int>(sorted_find(inst.veg_cumulative_graph[var_i], var_j)));
    };

    std::vector<int> ind(3);
    std::vector<double> val(3);
    constexpr double rhs_0{0.0}, rhs_1{1.0};
    constexpr char sense_l = 'L';
    constexpr int begin{0};

    // Constraint C6
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            int var_count{-1};
            for (const auto& var_j : inst.actions[act_i].eff_sparse) {
                var_count++;
                ind[0] = get_veg_idx(var_i, var_j);
                val[0] = -1;
                ind[1] = get_fa_idx(act_i, var_count);
                val[1] = 1;
                // if the VEG variable was eliminated, we can directly eliminate the first adder too
                if (!std::binary_search(inst.veg_cumulative_graph[var_i].begin(), inst.veg_cumulative_graph[var_i].end(), var_j)) {
                    const char fix = 'B';
                    const double zero = 0;
                    CPX_HANDLE_CALL(CPXchgbds(env, lp, 1, &(ind[1]), &fix, &zero));
                    continue;
                }
                CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
                stats.const_acyc++;
            }
            stopcheck();
        }
    }

    // Constraint C7
    for (unsigned int var_i = 0; var_i < inst.n; ++var_i) {
        for (const auto& var_j : inst.veg_cumulative_graph[var_i]) {
            // if the "inverse" veg variable was eliminated, we can skip the constraint
            if (!std::binary_search(inst.veg_cumulative_graph[var_j].begin(), inst.veg_cumulative_graph[var_j].end(), var_i)) continue;
            ind[0] = get_veg_idx(var_i, var_j);
            val[0] = 1;
            ind[1] = get_veg_idx(var_j, var_i);
            val[1] = 1;
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
            stats.const_acyc++;
            stopcheck();
        }
    }

    const std::vector<char> tmpfix(2, 'B');
    const std::vector<double> tmpzero(2, 0.0);

    // Constraint C8
    for (const auto& [a, b, c] : triangles_list) {
        ind[0] = get_veg_idx(a, b);
        val[0] = 1;
        ind[1] = get_veg_idx(b, c);
        val[1] = 1;
        ind[2] = get_veg_idx(a, c);
        val[2] = -1;
        // if veg(a, c) is eliminated, we can simply eliminate the other two VEG varliables
        if (!std::binary_search(inst.veg_cumulative_graph[a].begin(), inst.veg_cumulative_graph[a].end(), c)) {
            CPX_HANDLE_CALL(CPXchgbds(env, lp, 2, ind.data(), tmpfix.data(), tmpzero.data()));
            continue;
        }
        CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
        stats.const_acyc++;
        stopcheck();
    }
}

void ve::post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Posting warm start to VE model";

    binary_set state{inst.n};
    const auto& warm_start{inst.sol.sequence};

    const unsigned int ncols{static_cast<unsigned int>(CPXgetnumcols(env, lp))};
    std::vector<int> ind(ncols);
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(ncols, 0.0);
    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};

    for (const auto& act_i : warm_start) {
        val[act_i] = 1;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;

            unsigned int fadd_idx = inst.m + inst.fadd_cpx_start[act_i] + var_count;
            val[fadd_idx] = 1;
            unsigned int var_idx = inst.m + inst.nfadd + var_i;
            val[var_idx] = 1;
            for (const auto& var_j : inst.actions[act_i].pre_sparse) {
                unsigned int veg_idx =
                    inst.m + inst.nfadd + inst.n + static_cast<int>(inst.veg_starts[var_j] + sorted_find(inst.veg_cumulative_graph[var_j], var_i));
                val[veg_idx] = 1;
            }
        }
        state |= inst.actions[act_i].eff;
    }

    CPX_HANDLE_CALL(CPXaddmipstarts(env, lp, 1, ncols, &izero, ind.data(), val.data(), &effortlevel, nullptr));
}