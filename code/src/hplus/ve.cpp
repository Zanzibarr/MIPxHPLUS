#include <limits.hxx>
#include <set>

#include "solver.hpp"

// Vertex elimination model VE(Pi), Section 3.3 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// Feyzbakhsh Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
// Rose, Tarjan, Lueker: "Algorithmic Aspects of Vertex Elimination on Graphs"
void Solver::hplus_add_ve_constraints_() {
    const auto stopcheck = []() {
        if (global_limits::time_reached()) {
            throw EarlyExit("building ve model", EarlyExit::TIMELIMIT);
        }
    };

    // ====================================================== //
    // ================= VERTEX ELIMINATION ================= //
    // ====================================================== //

    // Initialize data structures
    // graph: current G_i (starting from the causal relation graph G_Pi), hplus_veg_cumulative_graph: E*_Pi (union of all E_i), triangles_list: Delta
    // This needs to not be an unordered_set because it gets iterated upon to create triangles_list, which is directly used to add constraints to
    // CPLEX (order sensitive)
    std::vector<std::set<unsigned int>> graph(inst_.n);
    global_.hplus_veg_cumulative_graph.resize(inst_.n);
    std::vector<std::tuple<unsigned int, unsigned int, unsigned int>> triangles_list;
    priority_queue<unsigned int> nodes_queue(static_cast<size_t>(2 * inst_.n));
    std::vector<unsigned int> degree_counter(inst_.n, 0);

    // Build initial graph G_0 = G_Pi: (p, q) is an edge iff A_{p->q} is not empty
    for (unsigned int act_i = 0; act_i < inst_.m; ++act_i) {
        for (const auto& var_i : inst_.actions[act_i].pre_sparse) {
            for (const auto& var_j : inst_.actions[act_i].eff_sparse) {
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
            for (const auto& eff_var : inst_.actions[act_i].eff_sparse) {
                insert_sorted(global_.hplus_veg_cumulative_graph[var_i], eff_var);
            }
        }
    }

    stopcheck();

    // Initialize priority queue with nodes that have edges
    for (unsigned int node_i = 0; node_i < inst_.n; ++node_i) {
        if (degree_counter[node_i] > 0) [[likely]] {
            nodes_queue.push(node_i, degree_counter[node_i]);
        }
    }

    // Apply minimum degree heuristics for G_i (computes the elimination ordering O, footnote 2 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning")
    for ([[maybe_unused]] unsigned int _ = 0; _ < inst_.n; ++_) {
        if (nodes_queue.empty()) [[unlikely]] {
            break;
        }

        const auto idx = static_cast<unsigned int>(nodes_queue.top());
        nodes_queue.pop();

        // Graph structure:
        // | \ > |
        // p -> idx -> q
        // | / > |

        // This cannot be an unordered set because it gets iterated upon for updating the nodes_queue (heap re-ordering is order sensitive)
        std::set<unsigned int> new_nodes;

        // Process all predecessors of idx
        for (unsigned int pre = 0; pre < inst_.n; ++pre) {
            if (!graph[pre].contains(idx)) {
                continue;
            }

            // Connect pre to all successors of idx
            for (const auto& eff : graph[idx]) {
                if (pre == eff) [[unlikely]] {
                    continue;
                }

                // Add edge pre -> eff if it doesn't exist
                if (const auto [iter, inserted] = graph[pre].insert(eff); inserted) {
                    ++degree_counter[pre];
                    ++degree_counter[eff];
                }

                // Update cumulative graph
                insert_sorted(global_.hplus_veg_cumulative_graph[pre], eff);
            }

            // Remove edge pre -> idx
            graph[pre].erase(idx);
            --degree_counter[pre];
            new_nodes.insert(pre);

            // Add triangles involving pre, idx, and successors of idx: (pre, idx, eff) in Delta
            for (const auto& eff : graph[idx]) {
                if (pre != eff) [[likely]] {
                    triangles_list.emplace_back(pre, idx, eff);
                }
            }
        }

        // Remove all edges from idx and update degrees
        for (const auto& eff : graph[idx]) {
            --degree_counter[eff];
            new_nodes.insert(eff);
        }

        graph[idx].clear();
        degree_counter[idx] = 0;

        // Update priority queue for affected nodes
        std::ranges::for_each(new_nodes, [&](const auto& node) {
            if (degree_counter[node] > 0 && nodes_queue.has(node)) {
                nodes_queue.change(node, degree_counter[node]);
            }
        });

        stopcheck();
    }

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    // e_{p,q} for each (p, q) in E*_Pi: hplus_veg_starts[p] is the offset of the first one with tail p
    const unsigned int veg_start{static_cast<unsigned int>(CPXgetnumcols(global_.hplus_env, global_.hplus_lp))};

    std::vector<double> objs(inst_.n, 0.0);
    std::vector<double> lbs(inst_.n, 0.0);
    std::vector<double> ubs(inst_.n, 1.0);
    std::vector<char> types(inst_.n, 'B');

    global_.hplus_veg_starts.resize(inst_.n);

    unsigned int count{0};
    for (unsigned int var_i = 0; var_i < inst_.n; var_i++) {
        global_.hplus_veg_starts[var_i] = count;
        call_cplex(CPXnewcols(global_.hplus_env, global_.hplus_lp, static_cast<int>(global_.hplus_veg_cumulative_graph[var_i].size()), objs.data(),
                              lbs.data(), ubs.data(), types.data(), nullptr));
        count += global_.hplus_veg_cumulative_graph[var_i].size();
        stopcheck();
    }

    stats_.counter_set<"n_var_acyc">(count);

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const unsigned int fa_start{inst_.m};  // Look at base model
    const auto get_fa_idx = [this, &fa_start](unsigned int act_idx, unsigned int var_count) {
        return static_cast<int>(fa_start + global_.hplus_fadd_cpx_start[act_idx] + var_count);
    };
    const auto get_veg_idx = [this, &veg_start](unsigned int var_a, unsigned int var_b) {
        return static_cast<int>(veg_start + global_.hplus_veg_starts[var_a] + sorted_find(global_.hplus_veg_cumulative_graph[var_a], var_b));
    };

    std::vector<int> ind(3);
    std::vector<double> val(3);
    constexpr double rhs_0{0.0};
    constexpr double rhs_1{1.0};
    constexpr char sense_l = 'L';
    constexpr int begin{0};

    // Equation 6 - Feyzbakhsh Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    // Equation 9 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
    // z_{a,q} <= e_{p,q}   (p = var_i, q = var_j)
    for (unsigned int act_i = 0; act_i < inst_.m; ++act_i) {
        for (const auto& var_i : inst_.actions[act_i].pre_sparse) {
            int var_count{-1};
            for (const auto& var_j : inst_.actions[act_i].eff_sparse) {
                var_count++;
                ind[0] = get_veg_idx(var_i, var_j);
                val[0] = -1;
                ind[1] = get_fa_idx(act_i, static_cast<unsigned int>(var_count));
                val[1] = 1;
                // if the VEG variable was eliminated, we can directly eliminate the first adder too
                if (!std::binary_search(global_.hplus_veg_cumulative_graph[var_i].begin(), global_.hplus_veg_cumulative_graph[var_i].end(), var_j)) {
                    const char fix = 'B';
                    const double zero = 0;
                    call_cplex(CPXchgbds(global_.hplus_env, global_.hplus_lp, 1, &(ind[1]), &fix, &zero));
                    continue;
                }
                call_cplex(
                    CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
                stats_.counter_inc<"n_const_acyc">();
            }
            stopcheck();
        }
    }

    // Equation 7 - Feyzbakhsh Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    // Equation 10 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
    // e_{p,q} + e_{q,p} <= 1   (p = var_i, q = var_j)
    for (unsigned int var_i = 0; var_i < inst_.n; ++var_i) {
        for (const auto& var_j : global_.hplus_veg_cumulative_graph[var_i]) {
            // if the "inverse" veg variable was eliminated, we can skip the constraint
            if (!std::binary_search(global_.hplus_veg_cumulative_graph[var_j].begin(), global_.hplus_veg_cumulative_graph[var_j].end(), var_i)) {
                continue;
            }
            ind[0] = get_veg_idx(var_i, var_j);
            val[0] = 1;
            ind[1] = get_veg_idx(var_j, var_i);
            val[1] = 1;
            call_cplex(CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
            stats_.counter_inc<"n_const_acyc">();
            stopcheck();
        }
    }

    const std::vector<char> tmpfix(2, 'B');
    const std::vector<double> tmpzero(2, 0.0);

    // Equation 8 - Feyzbakhsh Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    // Equation 11 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
    // e_{p,q} + e_{q,r} - 1 <= e_{p,r}, for each (p, q, r) in Delta
    for (const auto& [fact_p, fact_q, fact_r] : triangles_list) {
        ind[0] = get_veg_idx(fact_p, fact_q);
        val[0] = 1;
        ind[1] = get_veg_idx(fact_q, fact_r);
        val[1] = 1;
        ind[2] = get_veg_idx(fact_p, fact_r);
        val[2] = -1;
        // if e_{p,r} is eliminated, we can simply eliminate the other two VEG varliables
        if (!std::binary_search(global_.hplus_veg_cumulative_graph[fact_p].begin(), global_.hplus_veg_cumulative_graph[fact_p].end(), fact_r)) {
            call_cplex(CPXchgbds(global_.hplus_env, global_.hplus_lp, 2, ind.data(), tmpfix.data(), tmpzero.data()));
            continue;
        }
        call_cplex(CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
        stats_.counter_inc<"n_const_acyc">();
        stopcheck();
    }
}

// MIP start (Section 5.1) for VE(Pi) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// e_{p,q} = 1 iff p is achieved strictly before q in the warm start
void Solver::hplus_post_ve_warm_start_() {
    BinarySet state{inst_.n};
    const auto& warm_start{global_.solution};

    myassert(!warm_start.empty(), "No solution provided as warm start");

    const unsigned int ncols{static_cast<unsigned int>(CPXgetnumcols(global_.hplus_env, global_.hplus_lp))};
    std::vector<int> ind(ncols);
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(ncols, 0.0);
    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};

    std::vector<unsigned int> first_time(inst_.n, UINT_MAX);
    unsigned int step{0};

    for (const auto& act_i : warm_start) {
        val[act_i] = 1;
        int var_count{-1};
        for (const auto& var_i : inst_.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) {
                continue;
            }

            unsigned int fadd_idx = inst_.m + global_.hplus_fadd_cpx_start[act_i] + static_cast<unsigned int>(var_count);
            val[fadd_idx] = 1;
            unsigned int var_idx = inst_.m + inst_.nfadd + var_i;
            val[var_idx] = 1;
            first_time[var_i] = step;
        }
        state |= inst_.actions[act_i].eff_sparse;
        ++step;
    }

    // Set e_{i,j} = 1 iff first_time[i] < first_time[j] (equation numbers refer to Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning").
    //   Equation 9: when z_{act,eff}=1, pre was in state before eff → first_time[pre] < first_time[eff]
    //   Equation 10: antisymmetry holds because < is strict
    //   Equation 11: transitivity holds because < is transitive
    for (unsigned int var_i = 0; var_i < inst_.n; ++var_i) {
        for (unsigned int k = 0; k < global_.hplus_veg_cumulative_graph[var_i].size(); ++k) {
            const unsigned int var_j = global_.hplus_veg_cumulative_graph[var_i][k];
            if (first_time[var_i] < first_time[var_j]) {
                val[inst_.m + inst_.nfadd + inst_.n + global_.hplus_veg_starts[var_i] + k] = 1.0;
            }
        }
    }

    call_cplex(
        CPXaddmipstarts(global_.hplus_env, global_.hplus_lp, 1, static_cast<int>(ncols), &izero, ind.data(), val.data(), &effortlevel, nullptr));
}