#include <algorithm>
#include <binary_set.hxx>
#include <queue>

#include "bs_utils.hpp"
#include "max_flow.hpp"
#include "solver.hpp"
#include "utils.hpp"

// Algorithm 4 (heuristic landmark separator for fractional solutions, Section 6.2.2) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// actions_weights is the fractional point x*; the goal is handled as a dummy action a_t = (G, {t}) with x*_{a_t} = 1, whose node is the sink
auto Solver::maxflow_lm_relax_separation_(const std::vector<double>& actions_weights) -> std::pair<bool, std::vector<unsigned int>> {
    maxflow_lm_init_();

    // actions_weights may carry more entries than actions (e.g. first-adder variables); only the first inst_.m are action variables
    myassert(actions_weights.size() >= inst_.m, "Not enough weights to cover all the actions");

    // Algorithm 4, line 3 (same paper)
    std::vector<double> fmax(inst_.n, 0);
    std::vector<double> fadd(inst_.n, 0);
    maxflow_lm_compute_fmax_fadd_(actions_weights, fmax, fadd);

    // Algorithm 4, line 4 (Lemma 6.2, Proposition 6.1, same paper): f^max_{x*}(t) is a lower bound on the max-flow for any pcf: if it's >= 1 no
    // landmark is violated, and the max-flow can be skipped
    double fmax_goal{1};
    for (const auto gfact : global_.goal_sparse) {
        fmax_goal = std::min(fmax_goal, fmax[gfact]);
    }
    if (is_gr_or_eq_double(fmax_goal, 1)) {
        return {false, {}};
    }

    // Algorithm 4, lines 6-8 (same paper)
    const auto pcf = maxflow_lm_compute_pcf_(fmax, fadd);

    // Justification graph G_D of the pcf D: only the edges leaving the pcf of each action get its weight as capacity, the goal pcf -> sink edge gets 1
    const unsigned int source{inst_.n};
    const unsigned int sink{inst_.n + 1};
    FlowNetwork graph{local_.maxflow_lm_network};
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        const auto& pre_sparse = inst_.actions[act_i].pre_sparse;
        if (pre_sparse.empty()) {
            graph[source][local_.maxflow_lm_pre_edge[act_i][0]].c = actions_weights[act_i];
            continue;
        }
        for (unsigned int k = 0; k < pre_sparse.size(); k++) {
            if (pre_sparse[k] == pcf[act_i]) {
                graph[pre_sparse[k]][local_.maxflow_lm_pre_edge[act_i][k]].c = actions_weights[act_i];
                break;
            }
        }
    }
    for (unsigned int k = 0; k < global_.goal_sparse.size(); k++) {
        if (global_.goal_sparse[k] == pcf[inst_.m]) {
            graph[global_.goal_sparse[k]][local_.maxflow_lm_goal_edge[k]].c = 1;
            break;
        }
    }

    // Algorithm 4, lines 9-10 (Corollary 6.2, same paper): a violated landmark exists in G_D iff f(G_D, x*) < 1
    const double flow = compute_max_flow(graph, source, sink);
    if (is_gr_or_eq_double(flow, 1)) {
        return {false, {}};
    }

    // Algorithm 4, line 11 (same paper): the cut-set is returned as is (no greedy minimalization)
    return {true, maxflow_lm_extract_landmark_(graph, pcf)};
}

// Single effect transformation of Remark 6.1 (Algorithm 4, line 2) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// Each action a gets a dummy node p_a, the edge D(a) -> p_a plays the role of a (capacity x*_a) and the edges p_a -> e, e in add(a), the role of
// the dummy actions a_d (capacity x*_{a_d} = 1)
void Solver::maxflow_lm_init_() {
    // The network depends only on the (preprocessed) instance: build it once per thread
    if (local_.maxflow_lm_act_node.size() == inst_.m + 1) {
        return;
    }

    const unsigned int source{inst_.n};
    const unsigned int sink{inst_.n + 1};

    auto& graph = local_.maxflow_lm_network;
    graph = FlowNetwork(inst_.n + 2);
    local_.maxflow_lm_act_node = std::vector<unsigned int>(inst_.m + 1);
    local_.maxflow_lm_pre_edge = std::vector<std::vector<unsigned int>>(inst_.m);
    local_.maxflow_lm_goal_edge = std::vector<unsigned int>(global_.goal_sparse.size());

    // Adds the forward (residual) edge a -> b and its reverse (used flow) edge, returns the index of the forward edge in graph[a]
    auto add_edge = [&graph](unsigned int node_a, unsigned int node_b, double capacity) -> unsigned int {
        const auto edge_ab_idx = static_cast<unsigned int>(graph[node_a].size());
        const auto edge_ba_idx = static_cast<unsigned int>(graph[node_b].size());
        graph[node_a].push_back({.to = node_b, .rev = edge_ba_idx, .c = capacity, .is_reverse = false});
        graph[node_b].push_back({.to = node_a, .rev = edge_ab_idx, .c = 0, .is_reverse = true});
        return edge_ab_idx;
    };

    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        // Single effect transformation: each action gets a dummy node (pcf -> dummy -> effect(s)), which also prevents duplicates in the adjacency
        // lists
        const auto act_node = static_cast<unsigned int>(graph.size());
        graph.emplace_back();
        local_.maxflow_lm_act_node[act_i] = act_node;

        // Precondition edges get the action's weight as capacity only if they come from its pcf (set at each separation)
        if (inst_.actions[act_i].pre_sparse.empty()) {
            local_.maxflow_lm_pre_edge[act_i].push_back(add_edge(source, act_node, 0));
        } else {
            for (const auto pre : inst_.actions[act_i].pre_sparse) {
                local_.maxflow_lm_pre_edge[act_i].push_back(add_edge(pre, act_node, 0));
            }
        }

        // Dummy edges have capacity 1 since they are sourced only by the pcf -> dummy edge, which has (at most) the action's weight: this way the
        // min-cut is only associated to real actions (unless the min-cut is >= 1, in which case there's no violated landmark anyway)
        for (const auto eff : inst_.actions[act_i].eff_sparse) {
            add_edge(act_node, eff, 1);
        }
    }

    // Edges linking the goal to the sink: capacity 1 only for the pcf of the dummy goal action (set at each separation)
    for (unsigned int k = 0; k < global_.goal_sparse.size(); k++) {
        local_.maxflow_lm_goal_edge[k] = add_edge(global_.goal_sparse[k], sink, 0);
    }
    local_.maxflow_lm_act_node[inst_.m] = sink;
}

// f^max_{x*} (Eq. (22)) and f^add_{x*} (Eq. (23)) of the facts - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// Computed via a label-correcting propagation (initial actions read f(s) = 1)
void Solver::maxflow_lm_compute_fmax_fadd_(const std::vector<double>& actions_weights, std::vector<double>& fmax, std::vector<double>& fadd) {
    std::vector<double> act_fmax(inst_.m, 0);
    std::vector<double> act_fadd(inst_.m, 0);
    BinarySet state(inst_.n);
    std::queue<unsigned int> queue;
    BinarySet in_queue(inst_.m);

    // Actions with 0 weight can't propagate any value
    for (const auto act_i : global_.initial_actions) {
        if (is_lw_or_eq_double(actions_weights[act_i], 0)) {
            continue;
        }
        queue.push(act_i);
        in_queue.add(act_i);
    }

    // TODO: Consider using watch preconditions here aswell
    while (!queue.empty()) {
        const auto choice{queue.front()};
        queue.pop();
        in_queue.remove(choice);

        const double prev_fmax_act_value{act_fmax[choice]};
        const double prev_fadd_act_value{act_fadd[choice]};

        // Values for actions are the minimum value of its preconditions, capped at its weight: f(a) = min(min_{p in pre(a)} f(p), x*_a)
        act_fmax[choice] = actions_weights[choice];
        act_fadd[choice] = actions_weights[choice];
        for (const auto pre : inst_.actions[choice].pre_sparse) {
            act_fmax[choice] = std::min(act_fmax[choice], fmax[pre]);
            act_fadd[choice] = std::min(act_fadd[choice], fadd[pre]);
        }

        // If neither the fmax nor the fadd value of this action changed, there's nothing to propagate: restore the previous values so that
        // the (negligible) increase isn't lost, but accumulated in the delta of the next propagation of this action
        if (!is_gr_strict_double(act_fmax[choice], prev_fmax_act_value) && !is_gr_strict_double(act_fadd[choice], prev_fadd_act_value)) {
            act_fmax[choice] = prev_fmax_act_value;
            act_fadd[choice] = prev_fadd_act_value;
            continue;
        }
        const double fadd_act_delta{act_fadd[choice] - prev_fadd_act_value};

        state |= inst_.actions[choice].eff_sparse;

        // Values for facts are:
        // fmax: the maximum among the values of the actions that achieve it, f^max(q) = max_{a in A_{->q}} f^max(a)
        // fadd: the sum of the values of the actions that achieve it, f^add(q) = sum_{a in A_{->q}} f^add(a) (capped at 1, since only values < 1
        //       matter)
        for (const auto eff : inst_.actions[choice].eff_sparse) {
            const bool fmax_update{fmax[eff] < act_fmax[choice]};
            const bool fadd_update{fadd_act_delta > 0 && is_lw_strict_double(fadd[eff], 1)};
            if (!fmax_update && !fadd_update) {
                continue;
            }

            if (fmax_update) {
                fmax[eff] = act_fmax[choice];
            }
            if (fadd_update) {
                fadd[eff] = std::min(fadd[eff] + fadd_act_delta, 1.0);
            }

            // Since either the fmax or fadd value of eff has been updated, enqueue all the applicable actions that have it as precondition
            for (const auto act_i : global_.act_with_pre[eff]) {
                if (in_queue[act_i] || is_lw_or_eq_double(actions_weights[act_i], 0) || !bs_contains(state, inst_.actions[act_i].pre_sparse)) {
                    continue;
                }
                queue.push(act_i);
                in_queue.add(act_i);
            }
        }
    }
}

// Algorithm 4, lines 6-8 - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// D(a) = argmin_{p in pre(a)} f^max_{x*}(p), ties broken by f^add_{x*}
auto Solver::maxflow_lm_compute_pcf_(const std::vector<double>& fmax, const std::vector<double>& fadd) -> std::vector<unsigned int> {
    // The pcf is the precondition with the lowest fmax value (fadd as tie-breaking); last spot is reserved for the dummy goal action
    std::vector<unsigned int> pcf(inst_.m + 1, 0);

    // 1 is the max fmax/fadd value: using 2 as initial min guarantees an update
    auto choose_pcf = [&fmax, &fadd](const std::vector<unsigned int>& preconditions) -> unsigned int {
        unsigned int choice{0};
        double min_fmax{2};
        double min_fadd{2};
        for (const auto fact : preconditions) {
            if (is_lw_strict_double(fmax[fact], min_fmax) || (is_same_double(fmax[fact], min_fmax) && is_lw_strict_double(fadd[fact], min_fadd))) {
                min_fmax = fmax[fact];
                min_fadd = fadd[fact];
                choice = fact;
            }
        }
        return choice;
    };

    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        pcf[act_i] = inst_.actions[act_i].pre_sparse.empty() ? inst_.n : choose_pcf(inst_.actions[act_i].pre_sparse);
    }
    pcf[inst_.m] = choose_pcf(global_.goal_sparse);

    return pcf;
}

// Cut-set of G_D for the s-side S of the minimum cut (Section 2.3, Algorithm 4, line 11) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
// {a | D(a) in S, add(a) not subset of S}
// Since f(G_D, x*) < 1, no dummy edge (capacity 1) is saturated, so this is a subset of the minimum cut edges and sum_{a in L} x*_a <= f(G_D, x*)
auto Solver::maxflow_lm_extract_landmark_(const FlowNetwork& graph, const std::vector<unsigned int>& pcf) -> std::vector<unsigned int> {
    const auto graph_reach{get_min_cut_lpartition(graph, inst_.n)};
    BinarySet facts_reach(inst_.n);
    for (const auto node : graph_reach) {
        if (node < inst_.n) {
            facts_reach.add(node);
        }
    }

    myassert(!facts_reach.superset_of(inst_.goal), "Max-flow landmark extracted from a cut that contains the whole goal");

    std::vector<unsigned int> landmark;
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        // D(a) = n (the source s) for actions with no preconditions, which is always on the s-side
        const bool pcf_in_s{pcf[act_i] == inst_.n || facts_reach[pcf[act_i]]};
        if (pcf_in_s && !bs_contains(facts_reach, inst_.actions[act_i].eff_sparse)) {
            landmark.push_back(act_i);
        }
    }

    return landmark;
}
