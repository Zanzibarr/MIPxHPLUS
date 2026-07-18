#include <functional>
#include <limits>
#include <list>
#include <map>

#include "cli_descriptions.hpp"
#include "solver.hpp"

namespace {
[[nodiscard]]
constexpr auto hmax(double fact_a, double fact_b) -> double {
    return fact_a > fact_b ? fact_a : fact_b;
}

[[nodiscard]]
constexpr auto hadd(double fact_a, double fact_b) -> double {
    return fact_a + fact_b;
}
}  // namespace

void Solver::primal_greedy_() {
    // Initialize watch preconditions
    std::vector<unsigned int> watch_pre(inst_.m, inst_.n);
    std::vector<std::vector<unsigned int>> watching(inst_.n);

    BinarySet state{inst_.n};

    // Setup initial action candidates (initial actions) and watch preconditions
    std::list<unsigned int> candidates;
    for (unsigned int act_i = 0, initial_actions_counter = 0; act_i < inst_.m; act_i++) {
        if (initial_actions_counter < global_.initial_actions.size() && global_.initial_actions[initial_actions_counter] == act_i) {
            candidates.push_back(act_i);
            initial_actions_counter++;
        } else {
            myassert(!inst_.actions[act_i].pre_sparse.empty(), "Action missing from initial_actions has no preconditions");
            const auto pre = inst_.actions[act_i].pre_sparse[0];
            watch_pre[act_i] = pre;
            watching[pre].push_back(act_i);
        }
    }

    const auto& primal = params_.get<cli_desc::primal_heur, std::string>();
    if (primal == "ghm" || primal == "gha") {
        global_.primal_h_values = std::vector<double>(inst_.n, std::numeric_limits<double>::infinity());
        global_.primal_trail = std::stack<std::pair<unsigned int, double>>{};
        global_.primal_action_trail = std::stack<std::tuple<unsigned int, int, double>>{};
        global_.primal_pq = priority_queue<double>{inst_.n};
        global_.primal_used_actions = BinarySet(inst_.m);
        global_.primal_trail_flags = BinarySet(inst_.n);
        primal_hmaxadd_init_values_((primal == "ghm") ? hmax : hadd);

        if (primal == "ghm") {
            // hmax optimization: initialize PCF (argmax precondition) per action
            global_.primal_hmax_pcf.assign(inst_.m, -1);
            global_.primal_pcf_hmax.resize(inst_.m, 0.0);
            for (unsigned int act_i = 0; act_i < inst_.m; ++act_i) {
                double max_val = -1;
                for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                    if (global_.primal_h_values[pre] > max_val) {
                        max_val = global_.primal_h_values[pre];
                        global_.primal_hmax_pcf[act_i] = static_cast<int>(pre);
                    }
                }
                global_.primal_pcf_hmax[act_i] = (max_val >= 0) ? max_val : 0.0;
            }
            global_.primal_action_trail_flags = BinarySet(inst_.m);
        }
    }

    std::map<std::string, greedychoice_function> greedychoice_functions{{"gc", &Solver::primal_greedychoice_cost_},
                                                                        {"gcxe", &Solver::primal_greedychoice_cxe_},
                                                                        {"ghm", &Solver::primal_greedychoice_hmax_},
                                                                        {"gha", &Solver::primal_greedychoice_hadd_}};
    auto greedy_choice = std::bind_front(greedychoice_functions[primal], this);

    std::vector<unsigned int> solution;
    double incumbent{0};

    while (!state.superset_of(inst_.goal)) {
        if (candidates.empty()) [[unlikely]] {
            throw EarlyExit("computing the primal heuristic", EarlyExit::INFEASIBLE);
        }

        const auto& [found, choice] = greedy_choice(candidates, state);
        if (!found) [[unlikely]] {
            throw EarlyExit("computing the primal heuristic", EarlyExit::INFEASIBLE);
        }

        candidates.remove(choice);

        // Compute new effects
        std::vector<unsigned int> new_eff(inst_.actions[choice].eff_sparse.begin(), inst_.actions[choice].eff_sparse.end());
        std::erase_if(new_eff, [&state](const auto val) { return state[val]; });
        state |= new_eff;

        // Update candidates via watch preconditions
        for (const auto& eff : new_eff) {
            for (size_t i = 0; i < watching[eff].size();) {
                const auto act_i = watching[eff][i];

                // Try to find a new unsatisfied precondition to watch
                bool moved = false;
                for (const auto& pre : inst_.actions[act_i].pre_sparse) {
                    if (!state[pre]) {
                        watch_pre[act_i] = pre;
                        watching[pre].push_back(act_i);
                        moved = true;
                        break;
                    }
                }

                // Swap-erase from watching[eff] regardless (this fact is now reached, so no action is watchin it as its watch_pre)
                watching[eff][i] = watching[eff].back();
                watching[eff].pop_back();

                if (!moved) {
                    // All preconditions satisfied -> add to candidates if not already there
                    // and if its effects are not already subsumed by the current state
                    if (std::ranges::find(candidates, act_i) == candidates.end() && !bs_contains(state, inst_.actions[act_i].eff_sparse)) {
                        candidates.push_back(act_i);
                    }
                }
                // Don't increment i: swap-erase shifts the next element into position i
            }
        }

        // Purge candidates whose effects are now fully subsumed by the state
        candidates.remove_if(
            [&](unsigned int act_i) { return bs_contains(state, inst_.actions[act_i].eff_sparse) && !global_.fixed_actions[act_i]; });

        solution.push_back(choice);
        incumbent += inst_.actions[choice].cost;

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("computing the primal heuristic", EarlyExit::TIMELIMIT);
        }
    }

    try_update_best_incumbent_(solution, incumbent);
}