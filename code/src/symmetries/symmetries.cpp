#include <algorithm>
#include <digraph.hh>
#include <fstream>
#include <orbit.hh>
#include <stats.hh>
#include <string>
#include <unordered_map>
#include <vector>

#include "solver.hpp"

// The symmetry graph is a colored directed graph:
//   - one vertex per fact:   color 0 (normal) or 1 (goal)
//   - one vertex per action: one color >= 2 per distinct action cost
//   - edges pre -> act (preconditions) and act -> eff (effects)
// Color-preserving automorphisms of this graph are exactly the instance
// symmetries: they map facts to facts, actions to equal-cost actions, and
// preserve goal membership and the pre/eff structure.
// Vertex order (facts first, then actions) must be kept consistent between
// find_symmetries() and symm_write_graphfile().

namespace {
// One color per distinct action cost, starting from first_free_color
// (colors below it are reserved for facts).
auto symm_action_colors(const Instance& inst, unsigned int first_free_color) -> std::vector<unsigned int> {
    std::vector<unsigned int> colors(inst.m);
    std::unordered_map<unsigned int, unsigned int> cost_color;
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        const auto [entry, _] = cost_color.try_emplace(inst.actions[act_i].cost, first_free_color + cost_color.size());
        colors[act_i] = entry->second;
    }
    return colors;
}

// Cycle notation of an automorphism over n vertices, with vertices mapped back
// to instance entities: p_<i> for facts, a_<i> for actions (fixed points omitted).
auto symm_generator_to_string(unsigned int n_facts, unsigned int n, const unsigned int* aut) -> std::string {
    auto name = [n_facts](unsigned int v) { return v < n_facts ? std::format("p_{}", v) : std::format("a_{}", v - n_facts); };
    std::string cycles;
    std::vector<bool> visited(n, false);
    for (unsigned int v = 0; v < n; v++) {
        if (visited[v] || aut[v] == v) {
            continue;
        }
        cycles += '(' + name(v);
        visited[v] = true;
        for (unsigned int w = aut[v]; w != v; w = aut[w]) {
            visited[w] = true;
            cycles += ',' + name(w);
        }
        cycles += ')';
    }
    return cycles;
}
}  // namespace

// Debug utility: dump the symmetry graph in DIMACS format so the result can be
// cross-checked with the bliss CLI ("bliss -directed <path>").
// Assumes prep_setup_helpers_() has already been called (needs act_with_pre).
void Solver::symm_write_graphfile(const std::string& path) {
    // DIMACS vertices are 1-indexed
    auto fact_to_node = [](unsigned int fact) -> unsigned int { return fact + 1; };
    auto action_to_node = [this](unsigned int action) -> unsigned int { return inst_.n + action + 1; };

    std::ofstream f(path);
    integritycheck(f, "Text file didn't open");

    f << "c Symmetry graph (facts + actions, pre/eff edges)\n";

    // Header: number of nodes and edges
    unsigned int count{0};
    for (const auto& a : inst_.actions) {
        count += a.pre_sparse.size() + a.eff_sparse.size();
    }
    f << std::format("p edge {} {}\n", inst_.n + inst_.m, count);

    // Coloring the facts (color 0 is the DIMACS default, only goals need a line)
    for (const auto p : inst_.goal) {
        f << std::format("n {} 1\n", fact_to_node(p));
    }
    // Coloring the actions
    const auto action_colors = symm_action_colors(inst_, 2);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        f << std::format("n {} {}\n", action_to_node(act_i), action_colors[act_i]);
    }

    // Edges pre -> act
    for (unsigned int p = 0; p < inst_.n; p++) {
        for (const auto act_i : global_.act_with_pre[p]) {
            f << std::format("e {} {}\n", fact_to_node(p), action_to_node(act_i));
        }
    }
    // Edges act -> eff
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto p : inst_.actions[act_i].eff_sparse) {
            f << std::format("e {} {}\n", action_to_node(act_i), fact_to_node(p));
        }
    }
}

void Solver::find_symmetries() {
    // prep_setup_helpers_();

    // Debug: dump the graph for cross-checking with the bliss CLI
    // symm_write_graphfile("test.txt");

    // Build the symmetry graph (bliss vertices are 0-indexed: facts first, then actions)
    auto action_to_vertex = [this](unsigned int action) -> unsigned int { return inst_.n + action; };

    bliss::Digraph graph;
    for (unsigned int p = 0; p < inst_.n; p++) {
        graph.add_vertex(inst_.goal[p] ? 1 : 0);
    }
    const auto action_colors = symm_action_colors(inst_, 2);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        graph.add_vertex(action_colors[act_i]);
    }
    for (unsigned int p = 0; p < inst_.n; p++) {
        for (const auto act_i : global_.act_with_pre[p]) {
            graph.add_edge(p, action_to_vertex(act_i));
        }
    }
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (const auto p : inst_.actions[act_i].eff_sparse) {
            graph.add_edge(action_to_vertex(act_i), p);
        }
    }

    // Search for automorphisms; the orbit partition is the transitive closure
    // of the generators reported by bliss (union-find over their cycles)
    const unsigned int n_vertices = inst_.n + inst_.m;
    bliss::Stats stats;
    bliss::Orbit orbits;
    orbits.init(n_vertices);
    graph.find_automorphisms(stats, [this, &orbits](unsigned int n, const unsigned int* aut) {
        logger_[DEBUG] << std::format("Symmetries: generator {}", symm_generator_to_string(inst_.n, n, aut));
        for (unsigned int v = 0; v < n; v++) {
            if (aut[v] != v) {
                orbits.merge_orbits(v, aut[v]);
            }
        }
    });

    unsigned int largest_orbit = 0;
    for (unsigned int v = 0; v < n_vertices; v++) {
        largest_orbit = std::max(largest_orbit, orbits.orbit_size(v));
    }

    logger_[DEBUG] << std::format("Symmetries: {} generators", stats.get_nof_generators());
    logger_[DEBUG] << std::format("Symmetries: group order {}", stats.get_group_size().to_string());
    logger_[DEBUG] << std::format("Symmetries: largest orbit size {}", largest_orbit);

    logger_[FATAL] << "Forced exit";
}
