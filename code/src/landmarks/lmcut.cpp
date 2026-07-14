#include <map>

#include "cli_descriptions.hpp"
#include "solver.hpp"
#include "utils.hpp"

void Solver::solve_lmcut_() {
    const char minimization = params_.get<cli_desc::lmcut_min, std::string>()[0];

    std::map<char, lmcut_hmax_function> hmax_functions{{'a', &Solver::lmcut_hmax_arbitrary_}, {'i', &Solver::lmcut_hmax_inverse_},
                                                       {'v', &Solver::lmcut_hmax_vdm_},       {'r', &Solver::lmcut_hmax_random_},
                                                       {'z', &Solver::lmcut_hmax_gzd_},       {'b', &Solver::lmcut_hmax_bd_},
                                                       {'d', &Solver::lmcut_hmax_gzd_bd_}};

    for (auto choice : params_.get<cli_desc::lmcut_pcf, std::string>()) {
        auto _lmc_timer = scoped_timer("lmcut.single");

        const auto& [landmarks, lmcut_value] = lmcut_lmcut_(hmax_functions[choice], minimization);

        // TODO: If lmcut_value is infinite, then the problem is infeasible
        // TODO: Use fixed actions from the preprocessing as I'd use used actions in the integer separator (remember to keep track of the cost of
        // those actions)

        for (const auto& landmark : landmarks) {
            global_.landmarks.push_back(landmark);
            stats_.gauge_record<"lmcut.lm.avgsize">(static_cast<double>(landmark.size()));
        }
        stats_.gauge_record<"lmcut.size">(static_cast<double>(landmarks.size()));

        if (!landmarks.empty()) {
            logger_[DEBUG] << std::format("Computed a lm-cut value of {}", lmcut_value);
            try_update_best_bound_(lmcut_value);
        }

        if (global_limits::time_reached()) {
            throw EarlyExit("computing LM-Cut", EarlyExit::TIMELIMIT);
        }
    }
}

auto Solver::lmcut_lmcut_(const lmcut_hmax_function& hmax, char minimization) -> std::pair<std::vector<std::vector<unsigned int>>, double> {
    lmcut_init_();
    return lmcut_compute_private_(hmax, minimization);
}

auto Solver::lmcut_int_separation_(const std::vector<unsigned int>& used_actions, const lmcut_hmax_function& hmax, char minimization)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>> {
    lmcut_init_();

    // Set reduced costs of used actions to 0
    for (const auto& idx : used_actions) {
        local_.lmcut_reduced_costs[idx] = 0;
    }

    const auto& [landmarks, lmcut_val] = lmcut_compute_private_(hmax, minimization);

    return {!landmarks.empty(), landmarks};
}

auto Solver::lmcut_relax_separation_(const std::vector<double>& actions_weights, const lmcut_hmax_function& hmax, char minimization)
    -> std::pair<bool, std::vector<std::vector<unsigned int>>> {
    lmcut_init_();

    // actions_weights may carry more entries than actions (e.g. first-adder variables); only the first inst_.m are action variables
    myassert(actions_weights.size() >= inst_.m, "Not enough weights to cover all the actions");
    for (unsigned int i = 0; i < inst_.m; i++) {
        local_.lmcut_reduced_costs[i] *= (1 - actions_weights[i]);
    }

    auto [landmarks, lmcut_val] = lmcut_compute_private_(hmax, minimization);

    std::erase_if(landmarks, [&actions_weights](const std::vector<unsigned int>& landmark) {
        double sum = 0;
        for (const auto& act_i : landmark) {
            sum += actions_weights[act_i];
        }
        return sum >= 1 - constants::lm_relax_violation;  // TODO: Set to is_gr_or_eq_double(sum, 1 - constants::lm_relax_violation) once the new
                                                          // implementation has been proven identical to the old one
    });

    return {!landmarks.empty(), landmarks};
}

auto Solver::lmcut_hmax_arbitrary_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double> {
    int pcf{-1};
    double hmax{-1};
    for (const auto& pre : preconditions) {
        if (is_gr_strict_double(local_.lmcut_hmax_values[pre], hmax)) {
            hmax = local_.lmcut_hmax_values[pre];
            pcf = static_cast<int>(pre);
        }
    }
    return {pcf, hmax};
}

auto Solver::lmcut_hmax_inverse_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double> {
    int pcf{-1};
    double hmax{-1};
    for (const auto& pre : preconditions) {
        if (is_gr_or_eq_double(local_.lmcut_hmax_values[pre], hmax)) {
            hmax = local_.lmcut_hmax_values[pre];
            pcf = static_cast<int>(pre);
        }
    }
    return {pcf, hmax};
}

auto Solver::lmcut_hmax_vdm_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double> {
    int pcf{-1};
    int count{0};
    double hmax{-1};
    double min_decrease{std::numeric_limits<double>::infinity()};
    for (const auto& pre : preconditions) {
        if (is_gr_strict_double(local_.lmcut_hmax_values[pre], hmax)) {
            hmax = local_.lmcut_hmax_values[pre];
            pcf = static_cast<int>(pre);
            min_decrease = local_.lmcut_initial_hmax_values[pre] - local_.lmcut_hmax_values[pre];
            count = 1;
        } else if (is_same_double(hmax, local_.lmcut_hmax_values[pre])) {
            // First level tie-breaking: pcf with the smallest value decrease since the first hmax iteration
            if (is_lw_strict_double(local_.lmcut_initial_hmax_values[pre] - local_.lmcut_hmax_values[pre], min_decrease)) {
                pcf = static_cast<int>(pre);
                min_decrease = local_.lmcut_initial_hmax_values[pre] - local_.lmcut_hmax_values[pre];
                count = 1;
            } else if (is_same_double(local_.lmcut_initial_hmax_values[pre] - local_.lmcut_hmax_values[pre], min_decrease)) {
                // Second level tie-breaking: random
                count++;
                std::uniform_int_distribution<int> dist(1, count);
                if (dist(rng_()) == 1) {
                    pcf = static_cast<int>(pre);
                }
            }
        }
    }
    return {pcf, hmax};
}

auto Solver::lmcut_hmax_random_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double> {
    int pcf{-1};
    int count{0};
    double hmax{-1};
    for (const auto& pre : preconditions) {
        if (is_gr_strict_double(local_.lmcut_hmax_values[pre], hmax)) {
            hmax = local_.lmcut_hmax_values[pre];
            pcf = static_cast<int>(pre);
            count = 1;
        } else if (is_same_double(hmax, local_.lmcut_hmax_values[pre])) {
            count++;
            std::uniform_int_distribution<int> dist(1, count);
            if (dist(rng_()) == 1) {
                pcf = static_cast<int>(pre);
            }
        }
    }

    return {pcf, hmax};
}

auto Solver::lmcut_hmax_gzd_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double> {
    int pcf{-1};
    double hmax{-1};
    bool pcf_in_zone{false};
    for (const auto& pre : preconditions) {
        if (is_gr_strict_double(local_.lmcut_hmax_values[pre], hmax)) {
            hmax = local_.lmcut_hmax_values[pre];
            pcf = static_cast<int>(pre);
            pcf_in_zone = local_.lmcut_goal_section.contains(pre);
        } else if (is_same_double(hmax, local_.lmcut_hmax_values[pre])) {
            const bool pre_in_zone{local_.lmcut_goal_section.contains(pre)};
            if (pre_in_zone && !pcf_in_zone) {
                pcf = static_cast<int>(pre);
                pcf_in_zone = true;
            }
        }
    }
    return {pcf, hmax};
}

auto Solver::lmcut_hmax_bd_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double> {
    const auto is_border = [&](unsigned int fact) {
        return std::none_of(global_.act_with_eff[fact].begin(), global_.act_with_eff[fact].end(),
                            [&](unsigned int act_i) { return is_lw_or_eq_double(local_.lmcut_reduced_costs[act_i], 0); });
    };
    int pcf{-1};
    double hmax{-1};
    bool pcf_is_border{false};
    for (const auto& pre : preconditions) {
        if (is_gr_strict_double(local_.lmcut_hmax_values[pre], hmax)) {
            hmax = local_.lmcut_hmax_values[pre];
            pcf = static_cast<int>(pre);
            pcf_is_border = is_border(pre);
        } else if (is_same_double(hmax, local_.lmcut_hmax_values[pre])) {
            if (is_border(pre) && !pcf_is_border) {
                pcf = static_cast<int>(pre);
                pcf_is_border = true;
            }
        }
    }
    return {pcf, hmax};
}

auto Solver::lmcut_hmax_gzd_bd_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double> {
    const auto is_border = [&](unsigned int fact) {
        return std::none_of(global_.act_with_eff[fact].begin(), global_.act_with_eff[fact].end(),
                            [&](unsigned int act_i) { return is_lw_or_eq_double(local_.lmcut_reduced_costs[act_i], 0); });
    };
    int pcf{-1};
    double hmax{-1};
    bool pcf_in_zone{false};
    bool pcf_is_border{false};
    for (const auto& pre : preconditions) {
        if (is_gr_strict_double(local_.lmcut_hmax_values[pre], hmax)) {
            hmax = local_.lmcut_hmax_values[pre];
            pcf = static_cast<int>(pre);
            pcf_in_zone = local_.lmcut_goal_section.contains(pre);
            pcf_is_border = is_border(pre);
        } else if (is_same_double(hmax, local_.lmcut_hmax_values[pre])) {
            const bool pre_in_zone{local_.lmcut_goal_section.contains(pre)};
            const bool pre_is_border{is_border(pre)};
            // GZD: prefer pre if it's in V* and current pcf is not
            if (pre_in_zone && !pcf_in_zone) {
                pcf = static_cast<int>(pre);
                pcf_in_zone = true;
                pcf_is_border = pre_is_border;
                // BD: break remaining ties (both in zone or both not) by border status
            } else if (pre_in_zone == pcf_in_zone && pre_is_border && !pcf_is_border) {
                pcf = static_cast<int>(pre);
                pcf_is_border = true;
            }
        }
    }
    return {pcf, hmax};
}

void Solver::lmcut_init_() {
    local_.lmcut_pcf = std::vector<int>(inst_.m);
    local_.lmcut_hmax_values = std::vector<double>(inst_.n, std::numeric_limits<double>::infinity());
    local_.lmcut_initial_hmax_values = std::vector<double>(inst_.n, std::numeric_limits<double>::infinity());
    local_.lmcut_pcf_hmax = std::vector<double>(inst_.m);
    local_.lmcut_reduced_costs = std::vector<double>(inst_.m);
    local_.lmcut_goal_section.clear();

    for (unsigned int i = 0; i < inst_.m; i++) {
        if (inst_.actions[i].pre_sparse.empty()) {
            local_.lmcut_pcf[i] = static_cast<int>(inst_.n);
            local_.lmcut_pcf_hmax[i] = 0;
        } else {
            local_.lmcut_pcf[i] = -1;
            local_.lmcut_pcf_hmax[i] = std::numeric_limits<double>::infinity();
        }
        local_.lmcut_reduced_costs[i] = static_cast<double>(inst_.actions[i].cost);
    }
}

auto Solver::lmcut_compute_private_(const lmcut_hmax_function& hmax, char minimization) -> std::pair<std::vector<std::vector<unsigned int>>, double> {
    double lmcut_value{0};
    std::vector<std::vector<unsigned int>> landmarks;

    lmcut_update_hmax_values_(global_.initial_actions, hmax);

    local_.lmcut_initial_hmax_values = std::vector<double>(local_.lmcut_hmax_values.begin(), local_.lmcut_hmax_values.end());

    while (is_gr_strict_double((this->*hmax)(global_.goal_sparse).second, 0)) {
        const auto& [cut, val] = lmcut_compute_cut_(hmax, minimization);
        // check_landmark_(cut);  // This is an (expensive) integrity check...
        lmcut_value += val;
        lmcut_update_hmax_values_(cut, hmax);
        landmarks.push_back(cut);
        if (global_limits::time_reached()) {
            // This gets called also in the CPLEX callbacks... C code cannot handle this exception, so we just return what we computed so far
            return {landmarks, lmcut_value};
        }
    }

    return {landmarks, lmcut_value};
}

void Solver::lmcut_update_and_enqueue_effects_values_(priority_queue<double>& queue, unsigned int act_i) {
    const double new_cost{local_.lmcut_pcf_hmax[act_i] + local_.lmcut_reduced_costs[act_i]};
    for (const auto& eff : inst_.actions[act_i].eff_sparse) {
        // h_max(eff) = min_a{cost(a) + max{h_max{pre}}}
        if (is_gr_or_eq_double(new_cost, local_.lmcut_hmax_values[eff])) {
            continue;
        }

        local_.lmcut_hmax_values[eff] = is_lw_or_eq_double(new_cost, 0) ? 0 : new_cost;  // Fix numerical issues for close-to-0 values
        if (queue.has(eff)) {
            queue.change(eff, new_cost);
        } else {
            queue.push(eff, new_cost);
        }
    }
}

void Solver::lmcut_compute_goal_section_(const lmcut_hmax_function& hmax) {
    std::unordered_set<unsigned int> goal_section;
    std::deque<int> queue;

    // Simulate a 0-cost action with precondition the goal state -> set its pcf as starting goal_section
    int goal_pcf{(this->*hmax)(global_.goal_sparse).first};
    goal_section.insert(static_cast<unsigned int>(goal_pcf));
    queue.push_back(goal_pcf);

    std::unordered_set<unsigned int> explored;

    // Compute the goal section
    while (!queue.empty()) {
        const auto fact{queue.front()};
        queue.pop_front();

        for (const auto& act_i : global_.act_with_eff[static_cast<size_t>(fact)]) {
            // If I already explored this action or if it has no pcf, skip...
            if (explored.contains(act_i) || local_.lmcut_pcf[act_i] == -1) {
                continue;
            }

            explored.insert(act_i);

            // If it is a 0 reduced-cost (non-initial) action, than its pcf is also in the goal zone
            // non-initial check: I cannot add to the queue the source node, since no action could ever achieve it
            if (is_lw_or_eq_double(local_.lmcut_reduced_costs[act_i], 0) && local_.lmcut_pcf[act_i] != static_cast<int>(inst_.n)) {
                if (!goal_section.contains(static_cast<unsigned int>(local_.lmcut_pcf[act_i]))) {
                    goal_section.insert(static_cast<unsigned int>(local_.lmcut_pcf[act_i]));
                    queue.push_back(local_.lmcut_pcf[act_i]);
                }
            }
        }
    }

    local_.lmcut_goal_section = goal_section;
}

void Solver::lmcut_update_hmax_values_(const std::vector<unsigned int>& changed_actions, const lmcut_hmax_function& hmax) {
    priority_queue<double> queue{inst_.n};
    for (const auto& act_i : changed_actions) {
        lmcut_update_and_enqueue_effects_values_(queue, act_i);
    }

    while (!queue.empty()) {
        const auto fact{static_cast<int>(queue.top())};
        queue.pop();

        for (const auto& act_i : global_.act_with_pre[static_cast<size_t>(fact)]) {
            // If this action's pcf is not 'fact', than since we are lowering the hmax values, the hmax won't change for this action... skip
            if (local_.lmcut_pcf[act_i] != -1 && local_.lmcut_pcf[act_i] != fact) {
                continue;
            }

            const double old_hmax{local_.lmcut_pcf_hmax[act_i]};

            // Compute hmax and the pcf
            const auto& [act_pcf, act_hmax]{(this->*hmax)(inst_.actions[act_i].pre_sparse)};

            // If this action has no pcf or it's infinite, skip
            if (act_pcf == -1 || act_hmax == std::numeric_limits<double>::infinity()) {
                continue;
            }

            // Update the pcf
            local_.lmcut_pcf[act_i] = act_pcf;
            local_.lmcut_pcf_hmax[act_i] = is_lw_or_eq_double(act_hmax, 0) ? 0 : act_hmax;  // Fix numerical issues for close-to-0 values

            // If the hmax of this action hasnt changed, skip
            if (is_same_double(local_.lmcut_pcf_hmax[act_i], old_hmax)) {
                continue;
            }

            lmcut_update_and_enqueue_effects_values_(queue, act_i);
        }
    }
}

auto Solver::lmcut_compute_cut_(const lmcut_hmax_function& hmax, char minimization) -> std::pair<std::vector<unsigned int>, double> {
    lmcut_compute_goal_section_(hmax);

    // Compute the pre_goal section and the cut
    std::vector<unsigned int> cut;
    BinarySet pre_goal_section(inst_.n);
    std::unordered_set<unsigned int> explored;
    std::deque<int> queue;

    const auto& lmcut_opt = params_.get<cli_desc::lmcut_opt, std::string>();
    const auto& check_update_cut_pregoal = [&](unsigned int act_i) -> void {
        explored.insert(act_i);

        if (is_gr_strict_double(local_.lmcut_reduced_costs[act_i], 0)) {
            if (lmcut_opt == "0") {
                bool added = false;
                for (const auto& eff : inst_.actions[act_i].eff_sparse) {
                    if (local_.lmcut_goal_section.contains(eff)) {
                        if (added) {
                            continue;
                        }
                        cut.push_back(act_i);
                        added = true;
                    } else if (!pre_goal_section[eff]) {
                        pre_goal_section.add(eff);
                        queue.push_back(static_cast<int>(eff));
                    }
                }

            } else if (lmcut_opt == "strict-eff") {
                // TODO Salvagnin, Zanella: "Tighter Bounds for h+ MIP Planning via LM-Cut Strengthening and Cut Generation"

                // Case 1: This is the only action from the pre_goal_section that achieves a fact p not in the goal_section... actions added to
                // the cut because of this fact are not necessary for the validity of the landmark (the current action is a landmark for p (with
                // the current pcf) hence adding actions that generated from p (or from successive iterations generated from p) would only weaken
                // the landmark)... now pre_goal and goal sections are not summing up to the totality of the facts, but we can consider as
                // partition (pre_goal_section, P \ pre_goal_section) instead of (pre_goal_section, goal_section), which would still produce a
                // valid landmark since it's a cut for a valid partition of the facts
                //
                // Case 2: There are other actions from the pre_goal_section (that don't cross the cut) that achieve fact p... then they are gonna
                // add it to the pre_goal_section: the validity of the partition is preserved

                // First pass: check if this action crosses the cut (has an effect in the goal section).
                // Done as a separate pass so that if we return early, no facts are incorrectly added
                // to pre_goal_section.
                for (const auto& eff : inst_.actions[act_i].eff_sparse) {
                    if (local_.lmcut_goal_section.contains(eff)) {
                        cut.push_back(act_i);
                        return;
                    }
                }

                // Second pass: add effects to pre_goal_section in sorted eff_sparse order (deterministic).
                for (const auto& eff : inst_.actions[act_i].eff_sparse) {
                    if (!pre_goal_section[eff]) {
                        pre_goal_section.add(eff);
                        queue.push_back(static_cast<int>(eff));
                    }
                }
            } else {
                logger_[FATAL] << std::format("Unhandled {} option: {}", cli_desc::lmcut_opt.view(), lmcut_opt);
            }

        } else {
            for (const auto& eff : inst_.actions[act_i].eff_sparse) {
                myassert(!local_.lmcut_goal_section.contains(eff), "0-cost action crosses pre-goal/goal partition");
                if (!pre_goal_section[eff]) {
                    pre_goal_section.add(eff);
                    queue.push_back(static_cast<int>(eff));
                }
            }
        }
    };

    for (const auto& act_i : global_.initial_actions) {
        check_update_cut_pregoal(act_i);
    }

    while (!queue.empty()) {
        const auto fact{queue.front()};
        queue.pop_front();

        for (const auto& act_i : global_.act_with_pre[static_cast<unsigned int>(fact)]) {
            if (local_.lmcut_pcf[act_i] != fact || explored.contains(act_i) || bs_contains(pre_goal_section, inst_.actions[act_i].eff_sparse)) {
                continue;
            }
            check_update_cut_pregoal(act_i);
        }
    }

    // TODO Salvagnin, Zanella: "Tighter Bounds for h+ MIP Planning via LM-Cut Strengthening and Cut Generation"
    if (minimization == 'g') {
        // Note that this MUST be done after the cut has been computed 'cause before the pre_goal might still change...
        landmark_minimalization_greedy_(cut, pre_goal_section);
    }

    if (minimization == 'c') {
        std::vector<unsigned int> unapplicable_actions;
        for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
            if (!bs_contains(pre_goal_section, inst_.actions[act_i].pre_sparse)) {
                unapplicable_actions.push_back(act_i);
            }
        }

        // Further try to minimize the landmark
        landmark_minimalization_(cut, unapplicable_actions, pre_goal_section);
    }

    double min_reduced_cost = std::numeric_limits<double>::infinity();
    for (const auto& act_i : cut) {
        min_reduced_cost = std::min(min_reduced_cost, local_.lmcut_reduced_costs[act_i]);
    }

    for (const auto& act_i : cut) {
        local_.lmcut_reduced_costs[act_i] -= min_reduced_cost;
        if (is_lw_or_eq_double(local_.lmcut_reduced_costs[act_i], 0)) {
            local_.lmcut_reduced_costs[act_i] = 0;  // Fix numerical issues for close-to-0 values
        }
    }

    return {cut, min_reduced_cost};
}
