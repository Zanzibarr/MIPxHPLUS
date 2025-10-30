#include <list>

#include "relax_callback.hpp"

[[nodiscard]]
static inline double compute_r1(const hplus::instance& inst, const std::vector<double>& relax_point) {
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

    return r1;
}

[[nodiscard]]
static inline double compute_r2(const hplus::instance& inst, const std::vector<double>& relax_point) {
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

    return r2;
}

[[nodiscard]]
static inline std::pair<double, double> compute_r1_r2(const hplus::instance& inst, const std::vector<double>& relax_point) {
    return {compute_r1(inst, relax_point), compute_r2(inst, relax_point)};
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
    const auto [r1, r2] = compute_r1_r2(inst, relax_point);

    // Get number of nodes used to solve the flmdet model
    int nodes{CPXgetnodecnt(env, lp)};

    // Print statistics
    LOG_DEBUG << "(nodes: " << std::setw(4) << nodes << ") R1 / R2 / FLMDET : " << std::fixed << std::setprecision(5) << r1 << " / " << std::fixed
              << std::setprecision(5) << r2 << " / " << std::fixed << std::setprecision(5) << cutval << (r1 <= HPLUS_EPSILON ? " *" : "");

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
