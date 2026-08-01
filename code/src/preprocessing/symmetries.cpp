#include <algorithm>
#include <array>
#include <digraph.hh>
#include <fstream>
#include <logger.hxx>
#include <map>
#include <orbit.hh>
#include <timer.hxx>

#include "cli_descriptions.hpp"
#include "solver.hpp"

namespace {}  // namespace

auto Solver::prep_orbital_probing_() -> bool {
    auto _tmr = make_scoped_timer<"orbprob">(stats_);
    logger_[DEBUG] << "PREP: symmetry breaking";

    // ~~~~~~~~~~~~~ Build graph ~~~~~~~~~~~~~ //
    // Facts and actions marked for elimination leave the instance at the end of the pass, so they must not enter the graph: their edges would create
    // or destroy symmetries, and the duplicate actions 'd' has just marked would resurface as the only-action generators the conjecture below rules
    // out. Survivors get a contiguous vertex space of their own, facts first
    std::vector<unsigned int> local_fact;  // vertex -> fact
    std::vector<unsigned int> local_act;   // (vertex - n_local) -> action
    std::vector<unsigned int> fact_vertex(inst_.n, 0);
    for (const auto fact : !global_.eliminated_facts) {
        fact_vertex[fact] = static_cast<unsigned int>(local_fact.size());
        local_fact.push_back(fact);
    }
    for (const auto act_i : !global_.eliminated_actions) {
        local_act.push_back(act_i);
    }
    const auto n_local = static_cast<unsigned int>(local_fact.size());
    const auto m_local = static_cast<unsigned int>(local_act.size());
    if (m_local == 0) {
        logger_[DEBUG] << "SYMM: No surviving action to probe.";
        return false;
    }

    // No surviving action is gated by an eliminated fact: 'b' pulls the preconditions of every relevant action into the relevant set (so phase 1
    // cannot drop them) and phase 2 only drops facts no surviving action consumes. check_landmark_ below leans on this, since it simulates the
    // surviving actions without filtering their preconditions — the eliminated facts it reaches are inert, nothing tests them. The twins marked
    // right below are the one exception, and they are safe on their own account (see the fixing loop)
    myassert(
        [&]() -> bool {
            for (const auto act_i : local_act) {
                for (const auto pre : inst_.actions[act_i].pre_sparse) {
                    if (global_.eliminated_facts[pre]) {
                        return false;
                    }
                }
            }
            return true;
        }(),
        "A surviving action is gated by a fact marked for elimination");

    auto act_to_vertex = [n_local](unsigned int local_a) -> unsigned int { return n_local + local_a; };
    auto vertex_to_act = [&local_act, n_local](unsigned int node) -> unsigned int { return local_act[node - n_local]; };
    auto vertex_to_fact = [&local_fact](unsigned int node) -> unsigned int { return local_fact[node]; };
    auto symm_action_colors = [this, &local_act, m_local](unsigned int first_free_color) -> std::vector<unsigned int> {
        std::vector<unsigned int> colors(m_local);
        std::map<std::pair<unsigned int, bool>, unsigned int> cost_color;
        for (unsigned int local_a = 0; local_a < m_local; local_a++) {
            const std::pair key{inst_.actions[local_act[local_a]].cost, global_.fixed_actions.contains(local_act[local_a])};
            const auto [entry, _] = cost_color.try_emplace(key, first_free_color + cost_color.size());
            colors[local_a] = entry->second;
        }
        return colors;
    };

    bliss::Digraph graph;
    std::vector<unsigned int> fact_colors(n_local);
    const auto action_colors = symm_action_colors(2);  // facts occupy 2 colors (0,1), so the first free color for actions is 2
    {
        auto _build_tmr = make_scoped_timer<"orbprob.build">(stats_);
        for (unsigned int p = 0; p < n_local; p++) {
            // Goal and fixed facts are identical: a fixed fact can be considered a goal
            fact_colors[p] = inst_.goal[local_fact[p]] || global_.fixed_facts[local_fact[p]] ? 1 : 0;
            graph.add_vertex(fact_colors[p]);
        }
        for (unsigned int local_a = 0; local_a < m_local; local_a++) {
            graph.add_vertex(action_colors[local_a]);
        }
        for (unsigned int local_a = 0; local_a < m_local; local_a++) {
            const auto& act = inst_.actions[local_act[local_a]];
            for (const auto p : act.pre_sparse) {
                if (global_.eliminated_facts[p]) {
                    continue;
                }
                graph.add_edge(fact_vertex[p], act_to_vertex(local_a));
            }
            for (const auto q : act.eff_sparse) {
                if (global_.eliminated_facts[q]) {
                    continue;
                }
                graph.add_edge(act_to_vertex(local_a), fact_vertex[q]);
            }
        }
    }

    // static int counter = 0;
    // symm_write_graph_(std::format("symm{}.dot", counter++), fact_colors, action_colors, local_fact, local_act);

    // ~~~~~~~~ Compute automorphisms ~~~~~~~~ //
    const unsigned int n_vertices = n_local + m_local;
    bliss::Stats stats;
    bliss::Orbit orbits;
    std::vector<std::vector<std::vector<unsigned int>>> generators;
    {
        auto _automorph_tmr = make_scoped_timer<"orbprob.automorphisms">(stats_);

        orbits.init(n_vertices);
        graph.find_automorphisms(stats, [&orbits, &generators](unsigned int n, const unsigned int* aut) {
            // Compute orbits
            for (unsigned int v = 0; v < n; v++) {
                if (aut[v] != v) {
                    orbits.merge_orbits(v, aut[v]);
                }
            }

            // Store generators
            generators.emplace_back();  // New generator (permutation)
            auto& generator = generators.back();

            std::vector<bool> visited(n, false);
            for (unsigned int v = 0; v < n; v++) {
                if (visited[v] || aut[v] == v) {
                    continue;
                }
                visited[v] = true;
                generator.emplace_back();  // New cycle
                auto& cycle = generator.back();

                cycle.push_back(v);
                for (unsigned int w = aut[v]; w != v; w = aut[w]) {
                    visited[w] = true;
                    cycle.push_back(w);
                }
            }

            if (global_limits::time_reached()) [[unlikely]] {
                throw EarlyExit("orbital probing (computing automorphisms)", EarlyExit::TIMELIMIT);
            }
        });
    }

    // Analyze generators
    const auto& generator_to_string = [this, n_local, &vertex_to_fact, &vertex_to_act](const std::vector<std::vector<unsigned int>>& generator) {
        auto name = [this, n_local, &vertex_to_fact, &vertex_to_act](unsigned int v) {
            if (v >= n_local) {
                return std::format("A{}", vertex_to_act(v));
            }
            const auto fact = vertex_to_fact(v);
            return inst_.goal[fact] ? std::format("G{}", fact) : std::format("P{}", fact);
        };

        std::string cycles_string;
        for (const auto& cycle : generator) {
            std::string cycle_str = "(";
            for (auto v : cycle) {
                cycle_str += name(v) + (v == cycle.back() ? "" : ";");
            }
            cycle_str += ")";

            cycles_string += cycle_str;
        }
        return cycles_string;
    };

    for (const auto& generator : generators) {
        bool is_onlyfact_generator = true;
        bool is_onlyact_generator = true;

        for (const auto& cycle : generator) {
            bool is_fact_cycle = cycle.front() < n_local;  // Cycles won't ever mix facts and actions

            if (!is_fact_cycle) {
                is_onlyfact_generator = false;
            } else {
                is_onlyact_generator = false;
            }

            if (!is_onlyact_generator && !is_onlyfact_generator) {
                break;
            }
        }

        // If this is an generator of only facts, then this means that each cycle is composed of facts that are preconditions/effects of the same set
        // of actions, meaning that they are virtually identical (each action achieving any, achieves all and each action needing any, needs all) and
        // we can remove all but one of them
        // Note that this might eliminate goal facts, but only if they are virtually identical to a fixed/goal fact... this guarantees that if we
        // delete a goal fact, an identical fixed/goal fact remains in the task (at the end of the preprocessing, all fixed facts are promoted to
        // goals)
        if (is_onlyfact_generator) {
            // logger_[WARNING] << std::format("SYMM: Found only-fact generator (size: {}): {}", generator.size(), generator_to_string(generator));
            // Conjecture: we cannot have a only-fact generator with more than one cycle
            myassert(generator.size() == 1, std::format("Found a only-fact generator with more than one cycle: {}", generator_to_string(generator)));
            const auto& only_cycle = generator.front();
            // Conjecture: only cycles of size 2 are possible
            myassert(only_cycle.size() == 2,
                     std::format("Found a only-fact generator with a cycle with more than 2 facts: {}", generator_to_string(generator)));
            // Keep a goal fact as the cycle's representative whenever the cycle holds one
            unsigned int keep = 0;
            for (unsigned int p = 0; p < only_cycle.size(); p++) {
                if (inst_.goal[vertex_to_fact(only_cycle[p])]) {
                    keep = p;
                    break;
                }
            }

            for (unsigned int p = 0; p < only_cycle.size(); p++) {
                if (p == keep) {
                    continue;
                }
                global_.eliminated_facts.add(vertex_to_fact(only_cycle[p]));
                logger_[DEBUG] << std::format("SYMM: Eliminating fact {} during orbital probing", vertex_to_fact(only_cycle[p]));
            }
        }

        // Conjecture: an only-action generator means that multiple actions have the same preconditions, same effects, same cost and fixed status...
        // if the dominated actions step is enabled, such generator should never appear
        myassert((!is_onlyact_generator || params_.get<cli_desc::preprocess, std::string>().find('d') == std::string::npos),
                 std::format("Found an only-action generator: {}", generator_to_string(generator)));
    }

    // ~~~~~~~~~~~~ Compute orbits ~~~~~~~~~~~ //
    std::vector<BinarySet> orbits_members;
    std::vector<unsigned int> rep_to_idx(n_vertices);
    {
        auto _orbits_tmr = make_scoped_timer<"orbprob.orbits">(stats_);

        for (unsigned int v = 0; v < n_vertices; v++) {
            const auto rep = orbits.get_minimal_representative(v);
            // Ignore singleton orbits since those don't carry symmetries to exploit
            // Ignore orbigs of facts (for now)
            // Orbits of fixed actions are useless
            if (orbits.orbit_size(rep) == 1 || rep < n_local || global_.fixed_actions[vertex_to_act(rep)]) {
                continue;
            }
            if (v == rep) {
                rep_to_idx[rep] = orbits_members.size();
                orbits_members.emplace_back(BinarySet{inst_.m});
            }
            orbits_members[rep_to_idx[rep]].add(vertex_to_act(v));
        }
    }
    if (orbits_members.empty()) {
        logger_[DEBUG] << "SYMM: There are no non-trivial orbits of actions.";
        return false;
    }

    // ~~~~~~~~~~~ Sort the orbits ~~~~~~~~~~~ //
    // TODO: Insertion sort instead
    {
        auto _sorting_tmr = make_scoped_timer<"orbprob.sorting">(stats_);

        std::ranges::sort(orbits_members, [this](const auto& a, const auto& b) {
            // Order by orbit size: largest orbits first
            if (a.size() > b.size()) {
                return true;
            }
            myassert(!a.empty() && !b.empty(), "Empty orbit");
            // If sizes match, prefer the orbit with less preconditions (most likely to be applied immediatelly)
            // If size doesn't match return false (b.size() > a.size())
            return a.size() == b.size() && inst_.actions[*a.begin()].pre_sparse.size() < inst_.actions[*b.begin()].pre_sparse.size();
        });
    }

    //  Fix arbitrary action of orbital landmarks  //
    bool fixed_one = false;
    {
        auto _fixing_tmr = make_scoped_timer<"orbprob.fixing">(stats_);

        // TODO: Set a limit to the amount of orbits we look at...
        for (const auto& orbit : orbits_members) {
            // Check whether this orbit is a valid landmark. The eliminated actions are excluded alongside the orbit: they are already gone from the
            // task as far as this pass is concerned, and letting them reach the goal would hide landmarks. The facts marked for elimination need no
            // such care: 'b' (which is currently the only step before this that eliminates some facts) never strands a surviving action with an
            // eliminated precondition (it drags the preconditions of every relevant action into the relevant set), and the twins dropped just above
            // are reached exactly when their surviving partner is
            if (check_landmark_(orbit | global_.eliminated_actions)) {
                unsigned int fixed_act = *orbit.begin();
                // If this is a valid symmetric landmark, any action is equal to the other, so we simply fix the first one in the orbit
                global_.fixed_actions.add(fixed_act);
                // Fixing an action would change symmetries, we cannot fix more than one action per preprocessing loop
                logger_[DEBUG] << std::format("SYMM: Fixing action {} during orbital probing", fixed_act);
                fixed_one = true;
                break;
            }

            if (global_limits::time_reached()) [[unlikely]] {
                throw EarlyExit("orbital probing (looking for an action to fix) ({})", EarlyExit::TIMELIMIT);
            }
        }
    }

    return fixed_one;
}

/// Dumps the symmetry graph to Graphviz: facts are circles, actions are squares, fill color follows the bliss color
void Solver::symm_write_graph_(const std::string& path, const std::vector<unsigned int>& fact_colors, const std::vector<unsigned int>& action_colors,
                               const std::vector<unsigned int>& local_fact, const std::vector<unsigned int>& local_act) {
    /// Qualitative palette, indexed by bliss vertex color (wraps around if colors exceed its size)
    auto palette_of = [](unsigned int color) -> std::string_view {
        static constexpr std::array<std::string_view, 12> palette{
            "#e8e8e8", "#a6cee3", "#b2df8a", "#fb9a99", "#fdbf6f", "#cab2d6", "#1f78b4", "#33a02c", "#e31a1c", "#ff7f00", "#6a3d9a", "#b15928",
        };
        return palette.at(color % palette.size());
    };

    std::ofstream out(path);
    out << "digraph symm {\n  rankdir=LR;\n  node [style=filled, fontname=\"monospace\"];\n";
    for (unsigned int v = 0; v < local_fact.size(); v++) {
        const auto fact = local_fact[v];
        std::string var_name = inst_.goal[fact] ? "G" : "P";
        out << std::format("  {}{} [shape=circle, fillcolor=\"{}\"];\n", var_name, fact, palette_of(fact_colors[v]));
    }
    for (unsigned int v = 0; v < local_act.size(); v++) {
        out << std::format("  A{} [shape=square, fillcolor=\"{}\"];\n", local_act[v], palette_of(action_colors[v]));
    }
    for (const auto act_i : local_act) {
        for (const auto p : inst_.actions[act_i].pre_sparse) {
            if (global_.eliminated_facts[p]) {
                continue;
            }
            std::string var_name = inst_.goal[p] ? "G" : "P";
            out << std::format("  {}{} -> A{};\n", var_name, p, act_i);
        }
        for (const auto p : inst_.actions[act_i].eff_sparse) {
            if (global_.eliminated_facts[p]) {
                continue;
            }
            std::string var_name = inst_.goal[p] ? "G" : "P";
            out << std::format("  A{} -> {}{};\n", act_i, var_name, p);
        }
    }
    out << "}\n";
}