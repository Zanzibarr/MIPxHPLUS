/**
 * @file algorithms.cpp
 * @brief Algorithms to solve the delete-free relaxation of the planning task
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include "algorithms.hpp"

#include <algorithm>  // For std::transform
#include <list>       // For std::list
#include <numeric>    // For std::accumulate
#include <random>     // For random number generators
#include <set>        // For std::set
#include <stack>      // For std::stack

#if HPLUS_INTCHECK == 0
#define INTCHECK_PQ false
#endif
#include "pq.hxx"  // For priority_queue

// ##################################################################### //
// ############################ CPLEX UTILS ############################ //
// ##################################################################### //

void cpx_init(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::environment& env, const logger& log, bool log_file = true) {
    PRINT_VERBOSE(log, "Initializing CPLEX.");
    int cpxerror;
    cpxenv = CPXopenCPLEX(&cpxerror);
    // FIXME: Create a macro that checks the cplex status returned by a cplex function: handle memory issues for meaningful output in cluster testing
    ASSERT_LOG(log, !cpxerror);
    cpxlp = CPXcreateprob(cpxenv, &cpxerror, env.run_name.c_str());
    ASSERT_LOG(log, !cpxerror);
    // log file
    ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_ScreenOutput, CPX_OFF));
    if (log_file) ASSERT_LOG(log, !CPXsetlogfilename(cpxenv, (HPLUS_CPLEX_OUTPUT_DIR "/log/" + env.run_name + ".log").c_str(), "w"));
    ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPX_PARAM_CLONELOG, -1));
    ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_MIP_Display, 3));
    // ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_MIP_Limits_Solutions, 1));
    // tolerance
    ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_MIP_Tolerances_MIPGap, 0));
    // memory/size limits
    ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_MIP_Limits_TreeMemory, 12000));
    ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_WorkMem, 4096));
    ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_MIP_Strategy_File, 3));
    // terminate condition
    ASSERT_LOG(log, !CPXsetterminate(cpxenv, &global_terminate));
}

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp) {
    CPXfreeprob(cpxenv, &cpxlp);
    CPXcloseCPLEX(&cpxenv);
}

[[nodiscard]]
bool parse_cpx_status(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Parsing CPLEX status code.");
    switch (const int cpxstatus{CPXgetstat(cpxenv, cpxlp)}) {
        case CPXMIP_TIME_LIM_FEAS:  // exceeded time limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_ABORT_FEAS:  // terminated by user, found solution
            env.sol_s = solution_status::FEAS;
            return true;
        case CPXMIP_TIME_LIM_INFEAS:  // exceeded time limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_ABORT_INFEAS:  // terminated by user, not found solution
            if (!env.warm_start) env.sol_s = solution_status::NOTFOUND;
            return false;
        case CPXMIP_INFEASIBLE:  // proven to be unfeasible
            env.sol_s = solution_status::INFEAS;
            return false;
        case CPXMIP_OPTIMAL_TOL:  // found optimal within the tollerance
            PRINT_WARN(log, "Found optimal within the tolerance.");
            [[fallthrough]];
        case CPXMIP_OPTIMAL:  // found optimal
            env.sol_s = solution_status::OPT;
            return true;
        default:  // unhandled status
            log.raise_error("Error in parse_cpx_status: unhandled cplex status: %d.", cpxstatus);
            return false;
    }
}

// ##################################################################### //
// ############################# HEURISTICS ############################ //
// ##################################################################### //

static void greedycost(hplus::instance& inst, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Running greedycost algorithm.");

    unsigned int timestamp{0};
    // greedy choice
    const auto find_best_act = [&inst, &timestamp](const std::list<size_t>& candidates) {
        size_t choice{0};
        int best_cost{std::numeric_limits<int>::max()};
        for (const auto& act_i : candidates) {
            if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp) return std::pair(true, act_i);
            if (inst.act_f[act_i]) {
                choice = act_i;
                best_cost = -1;
                continue;
            }

            if (best_cost < 0) continue;

            if (inst.actions[act_i].cost >= static_cast<unsigned int>(best_cost)) continue;

            choice = act_i;
            best_cost = inst.actions[act_i].cost;
        }
        return std::pair(true, choice);
    };

    hplus::solution heur_sol;
    heur_sol.plan.reserve(inst.m_opt);
    heur_sol.cost = 0;

    binary_set state{inst.n};  // initial state is empty

    // binary_set searcher for faster actions lookup
    bs_searcher feasible_actions{inst.n};
    for (const auto& act_i : inst.act_rem) feasible_actions.add(act_i, inst.actions[act_i].pre);

    std::vector<size_t> tmp{feasible_actions.find_subsets(binary_set(inst.n))};
    std::list<size_t> candidates{tmp.begin(), tmp.end()};

    while (!state.contains(inst.goal)) {
        if (candidates.empty()) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        const auto [found, choice]{find_best_act(candidates)};
        if (!found) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        candidates.remove(choice);

        // add new actions to the candidates
        const auto& new_state = state | inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff - state) {
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (new_state.contains(inst.actions[act_i].pre) && std::find(candidates.begin(), candidates.end(), act_i) == candidates.end())
                    candidates.push_back(act_i);
            }
        }

        // purge unnecessary actions from candidates
        std::vector<size_t> purged_actions;
        for (const auto& act_i : candidates) {
            if (new_state.contains(inst.actions[act_i].eff) && !inst.act_f[act_i]) purged_actions.push_back(act_i);
        }
        for (const auto& act_i : purged_actions) candidates.remove(act_i);
        for (const auto& act_i : inst.act_inv[choice]) candidates.remove(act_i);

        heur_sol.plan.push_back(choice);
        heur_sol.cost += inst.actions[choice].cost;
        state = new_state;
        timestamp++;

        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

#if HPLUS_INTCHECK
    for (size_t i = 0; i < inst.m; i++) {
        if (inst.act_e[i]) ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) == heur_sol.plan.end());
        if (inst.act_f[i]) ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) != heur_sol.plan.end());
        if (inst.act_t[i] < 0) continue;
        ASSERT_LOG(log, inst.act_f[i]);
        ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) != heur_sol.plan.end());
        ASSERT_LOG(log, heur_sol.plan[inst.act_t[i]] == i);
    }
#endif

    hplus::update_sol(inst, heur_sol, log);
    env.sol_s = solution_status::FEAS;
}

static void greedycxe(hplus::instance& inst, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Running greedycxe algorithm.");

    unsigned int timestamp{0};
    // greedy choice
    const auto find_best_act = [&inst, &timestamp](const std::list<size_t>& candidates, const binary_set& state) {
        size_t choice{0};
        bool found{false};
        double best_cxe{std::numeric_limits<double>::infinity()};
        for (const auto& act_i : candidates) {
            if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp) return std::pair(true, act_i);
            if (inst.act_f[act_i]) {
                choice = act_i;
                best_cxe = -1;
                found = true;
                continue;
            }

            if (best_cxe < 0) continue;

            unsigned int neff{0};
            for (const auto& var_i : inst.actions[act_i].eff_sparse) {
                if (!state[var_i]) neff++;
            }
            if (neff == 0) continue;
            const double cxe{static_cast<double>(inst.actions[act_i].cost) / neff};
            if (cxe >= best_cxe) continue;

            found = true;
            choice = act_i;
            best_cxe = cxe;
        }
        return std::pair(found, choice);
    };

    hplus::solution heur_sol;
    heur_sol.plan.reserve(inst.m_opt);
    heur_sol.cost = 0;

    binary_set state{inst.n};  // initial state is empty

    // binary_set searcher for faster actions lookup
    bs_searcher feasible_actions{inst.n};
    for (const auto& act_i : inst.act_rem) feasible_actions.add(act_i, inst.actions[act_i].pre);

    std::vector<size_t> tmp{feasible_actions.find_subsets(binary_set(inst.n))};
    std::list<size_t> candidates{tmp.begin(), tmp.end()};

    while (!state.contains(inst.goal)) {
        if (candidates.empty()) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        const auto [found, choice]{find_best_act(candidates, state)};
        if (!found) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        candidates.remove(choice);

        // add new actions to the candidates
        const auto& new_state = state | inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff - state) {
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (new_state.contains(inst.actions[act_i].pre) && std::find(candidates.begin(), candidates.end(), act_i) == candidates.end())
                    candidates.push_back(act_i);
            }
        }

        // purge unnecessary actions from candidates
        std::vector<size_t> purged_actions;
        for (const auto& act_i : candidates) {
            if (new_state.contains(inst.actions[act_i].eff) && !inst.act_f[act_i]) purged_actions.push_back(act_i);
        }
        for (const auto& act_i : purged_actions) candidates.remove(act_i);
        for (const auto& act_i : inst.act_inv[choice]) candidates.remove(act_i);

        heur_sol.plan.push_back(choice);
        heur_sol.cost += inst.actions[choice].cost;
        state = new_state;
        timestamp++;

        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

#if HPLUS_INTCHECK
    for (size_t i = 0; i < inst.m; i++) {
        if (inst.act_e[i]) ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) == heur_sol.plan.end());
        if (inst.act_f[i]) ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) != heur_sol.plan.end());
        if (inst.act_t[i] < 0) continue;
        ASSERT_LOG(log, inst.act_f[i]);
        ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) != heur_sol.plan.end());
        ASSERT_LOG(log, heur_sol.plan[inst.act_t[i]] == i);
    }
#endif

    hplus::update_sol(inst, heur_sol, log);
    env.sol_s = solution_status::FEAS;
}

static void randheur(hplus::instance& inst, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Running rand algorithm.");

    unsigned int timestamp{0};
    // greedy choice
    const auto find_best_act = [&inst, &timestamp](const std::list<size_t>& candidates) {
        size_t choice{0};
        bool found{false};
        for (const auto& act_i : candidates) {
            if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp) return std::pair(true, act_i);
            if (inst.act_f[act_i]) {
                choice = act_i;
                found = true;
            }
        }
        if (found) return std::pair(true, choice);
        auto it = candidates.begin();
        std::advance(it, rand() % candidates.size());
        return std::pair(true, *it);
    };

    hplus::solution heur_sol;
    heur_sol.plan.reserve(inst.m_opt);
    heur_sol.cost = 0;

    binary_set state{inst.n};  // initial state is empty

    // binary_set searcher for faster actions lookup
    bs_searcher feasible_actions{inst.n};
    for (const auto& act_i : inst.act_rem) feasible_actions.add(act_i, inst.actions[act_i].pre);

    std::vector<size_t> tmp{feasible_actions.find_subsets(binary_set(inst.n))};
    std::list<size_t> candidates{tmp.begin(), tmp.end()};

    while (!state.contains(inst.goal)) {
        if (candidates.empty()) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        const auto [found, choice]{find_best_act(candidates)};
        if (!found) [[unlikely]] {
            env.sol_s = solution_status::INFEAS;
            return;
        }

        candidates.remove(choice);

        // add new actions to the candidates
        const auto& new_state = state | inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff - state) {
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (new_state.contains(inst.actions[act_i].pre) && std::find(candidates.begin(), candidates.end(), act_i) == candidates.end())
                    candidates.push_back(act_i);
            }
        }

        // purge unnecessary actions from candidates
        std::vector<size_t> purged_actions;
        for (const auto& act_i : candidates) {
            if (new_state.contains(inst.actions[act_i].eff) && !inst.act_f[act_i]) purged_actions.push_back(act_i);
        }
        for (const auto& act_i : purged_actions) candidates.remove(act_i);
        for (const auto& act_i : inst.act_inv[choice]) candidates.remove(act_i);

        heur_sol.plan.push_back(choice);
        heur_sol.cost += inst.actions[choice].cost;
        state = new_state;
        timestamp++;

        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

#if HPLUS_INTCHECK
    for (size_t i = 0; i < inst.m; i++) {
        if (inst.act_e[i]) ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) == heur_sol.plan.end());
        if (inst.act_f[i]) ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) != heur_sol.plan.end());
        if (inst.act_t[i] < 0) continue;
        ASSERT_LOG(log, inst.act_f[i]);
        ASSERT_LOG(log, std::find(heur_sol.plan.begin(), heur_sol.plan.end(), i) != heur_sol.plan.end());
        ASSERT_LOG(log, heur_sol.plan[inst.act_t[i]] == i);
    }
#endif

    hplus::update_sol(inst, heur_sol, log);
    env.sol_s = solution_status::FEAS;
}

static void randr(hplus::instance& inst, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Running randr algorithm.");
    // TODO: (maybe) Choose on CLI
    constexpr size_t repetitions{100};
    // TODO: Threads (if using threads, make hplus::update_sol thread safe.)
    for (size_t _ = 0; _ < repetitions; _++) {
        randheur(inst, env, log);
        if (env.sol_s == solution_status::INFEAS) [[unlikely]]
            return;
    }
}

[[nodiscard]]
static double evaluate_htype_state(const std::vector<size_t>& state, const std::vector<double>& values, double (*h_eqtype)(double, double)) {
    double state_hcost{0};
    for (const auto& p : state) state_hcost = h_eqtype(state_hcost, values[p]);
    return state_hcost;
}

[[nodiscard]]
static double evaluate_htype_state(const binary_set& state, const std::vector<double>& values, double (*h_eqtype)(double, double)) {
    double state_hcost{0};
    for (const auto& p : state) state_hcost = h_eqtype(state_hcost, values[p]);
    return state_hcost;
}

static void update_htype_values(const hplus::instance& inst, const binary_set& state, std::vector<double>& values, priority_queue<double>& pq,
                                const binary_set& used_actions, double (*h_eqtype)(double, double),
                                std::stack<std::pair<size_t, double>>* trail = nullptr) {
    binary_set trail_flags{inst.n};
    for (const auto& p : state) {
        if (trail != nullptr && !trail_flags[p]) {
            trail->emplace(p, values[p]);
            trail_flags.add(p);
        }
        values[p] = 0;
        pq.push(p, 0);
    }

    while (!pq.empty()) {
        const size_t p{pq.top()};
        pq.pop();
        for (const auto& act_i : inst.act_with_pre[p]) {
            if (used_actions[act_i]) continue;

            const double cost_pre{evaluate_htype_state(inst.actions[act_i].pre_sparse, values, h_eqtype)};
            if (cost_pre >= std::numeric_limits<double>::infinity()) continue;

            const double new_cost{cost_pre + inst.actions[act_i].cost};
            for (const auto& p_eff : inst.actions[act_i].eff_sparse) {
                if (new_cost >= values[p_eff]) continue;

                if (trail != nullptr && !trail_flags[p_eff]) {
                    trail->emplace(p_eff, values[p_eff]);
                    trail_flags.add(p_eff);
                }
                values[p_eff] = new_cost;
                if (pq.has(p_eff))
                    pq.change(p_eff, new_cost);
                else
                    pq.push(p_eff, new_cost);
            }
        }
    }
}

static void init_htype(const hplus::instance& inst, const std::list<size_t>& initial_actions, std::vector<double>& values, priority_queue<double>& pq,
                       double (*h_eqtype)(double, double)) {
    for (const auto& act_i : initial_actions) {
        // preconditions of these variables are already met -> hmax/hadd (act.pre) = 0 -> need only the cost of the action to set the hmax/hadd of the
        // effects
        const double cost{static_cast<double>(inst.actions[act_i].cost)};
        for (const auto& p : inst.actions[act_i].eff_sparse) {
            if (cost >= values[p]) continue;

            values[p] = cost;
            if (pq.has(p))
                pq.change(p, cost);
            else
                pq.push(p, cost);
        }
    }
    update_htype_values(inst, binary_set{inst.n}, values, pq, binary_set{inst.m}, h_eqtype);
}

[[nodiscard]]
static bool htype(const hplus::instance& inst, hplus::solution& sol, double (*h_eqtype)(double, double), const logger& log) {
    // Reset solution (just to be sure)
    sol.plan.clear();
    sol.plan.reserve(inst.m_opt);
    sol.cost = 0;

    // Initialize helpers
    std::vector<double> values(inst.n, std::numeric_limits<double>::infinity());
    binary_set state{inst.n}, used_actions{inst.m};
    priority_queue<double> pq{inst.n};

    unsigned int timestamp{0};
    std::stack<std::pair<size_t, double>> trail;  // leave a trail for the action simulation to be reverted
    const auto find_best_act = [&inst, &h_eqtype, &used_actions, &trail, &pq, &timestamp, &log](
                                   const std::list<size_t>& candidates, std::vector<double>& values, const binary_set& state) {
        size_t choice{0};
        bool found{false};
        double best_goal_cost{std::numeric_limits<double>::infinity()};
        double current_goal_cost{evaluate_htype_state(inst.goal, values, h_eqtype)};
        for (const auto& act_i : candidates) {
            if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp) return std::pair(true, act_i);
            if (inst.act_f[act_i]) {
                choice = act_i;
                best_goal_cost = -1;
                found = true;
                continue;
            }

            if (best_goal_cost < 0) continue;

            update_htype_values(inst, inst.actions[act_i].eff - state, values, pq, used_actions, h_eqtype, &trail);

            double goal_cost{h_eqtype(1, 1) == 1 ? /*hmax*/ evaluate_htype_state(inst.goal, values, h_eqtype) : current_goal_cost};
            // reset values according to the trail
            while (!trail.empty()) {
                const auto& [idx, value] = trail.top();
                trail.pop();
                if (inst.goal[idx] && h_eqtype(1, 1) != 1) {  // hadd
                    goal_cost -= (value - values[idx]);
                }
                values[idx] = value;
            }

            if (goal_cost >= best_goal_cost) continue;

            choice = act_i;
            best_goal_cost = goal_cost;
            found = true;
        }
        return std::pair(found, choice);
    };

    // binary_set searcher for faster actions lookup
    bs_searcher feasible_actions{inst.n};
    for (const auto& act_i : inst.act_rem) feasible_actions.add(act_i, inst.actions[act_i].pre);

    // initialize values and priority queue (needs to be done manually since here the initial state will always be empty)
    std::vector<size_t> tmp{feasible_actions.find_subsets(binary_set(inst.n))};
    std::list<size_t> candidates{tmp.begin(), tmp.end()};
    init_htype(inst, candidates, values, pq, h_eqtype);

    while (!state.contains(inst.goal)) {
        if (candidates.empty()) [[unlikely]]
            return false;

        const auto [found, choice]{find_best_act(candidates, values, state)};
        if (!found) [[unlikely]]
            return false;

        used_actions.add(choice);
        candidates.remove(choice);
        update_htype_values(inst, inst.actions[choice].eff - state, values, pq, used_actions, h_eqtype);

        // add new actions to the candidates
        const auto& new_state = state | inst.actions[choice].eff;
        for (const auto& p : inst.actions[choice].eff - state) {
            for (const auto& act_i : inst.act_with_pre[p]) {
                if (new_state.contains(inst.actions[act_i].pre) && std::find(candidates.begin(), candidates.end(), act_i) == candidates.end())
                    candidates.push_back(act_i);
            }
        }

        // purge unnecessary actions from candidates
        std::vector<size_t> purged_actions;
        for (const auto& act_i : candidates) {
            if (new_state.contains(inst.actions[act_i].eff) && !inst.act_f[act_i]) purged_actions.push_back(act_i);
        }
        for (const auto& act_i : purged_actions) candidates.remove(act_i);
        for (const auto& act_i : inst.act_inv[choice]) candidates.remove(act_i);

        sol.plan.push_back(choice);
        sol.cost += inst.actions[choice].cost;
        state = new_state;
        timestamp++;

        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

#if HPLUS_INTCHECK
    for (size_t i = 0; i < inst.m; i++) {
        if (inst.act_e[i]) ASSERT_LOG(log, std::find(sol.plan.begin(), sol.plan.end(), i) == sol.plan.end());
        if (inst.act_f[i]) ASSERT_LOG(log, std::find(sol.plan.begin(), sol.plan.end(), i) != sol.plan.end());
        if (inst.act_t[i] < 0) continue;
        ASSERT_LOG(log, inst.act_f[i]);
        ASSERT_LOG(log, std::find(sol.plan.begin(), sol.plan.end(), i) != sol.plan.end());
        ASSERT_LOG(log, sol.plan[inst.act_t[i]] == i);
    }
#endif

    return true;
}

static void hmax(hplus::instance& inst, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Running hmax algorithm.");

    hplus::solution heur_sol;
    heur_sol.plan.reserve(inst.m_opt);
    heur_sol.cost = 0;

    if (!htype(inst, heur_sol, [](const double a, const double b) { return (a > b ? a : b); }, log)) {
        env.sol_s = solution_status::INFEAS;
        return;
    }

    hplus::update_sol(inst, heur_sol, log);
    env.sol_s = solution_status::FEAS;
}

static void hadd(hplus::instance& inst, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Running hadd algorithm.");

    hplus::solution heur_sol;
    heur_sol.plan.reserve(inst.m_opt);
    heur_sol.cost = 0;

    if (!htype(inst, heur_sol, [](const double a, const double b) { return a + b; }, log)) {
        env.sol_s = solution_status::INFEAS;
        return;
    }

    hplus::update_sol(inst, heur_sol, log);
    env.sol_s = solution_status::FEAS;
}

static void localsearch(hplus::instance& inst, void (*heuristic)(hplus::instance&, hplus::environment&, const logger&), hplus::environment& env,
                        const logger& log) {
    PRINT_VERBOSE(log, "Running local-search algorithm.");
    heuristic(inst, env, log);

    if (env.sol_s == solution_status::INFEAS) [[unlikely]]
        return;

    hplus::solution heur_sol{.plan = inst.best_sol.plan, .cost = inst.best_sol.cost};

    // TODO: Local search on top of the initial solution
    todo(log, "Local Search");

    hplus::update_sol(inst, heur_sol, log);
}

// ##################################################################### //
// ################################ IMAI ############################### //
// ##################################################################### //

void cpx_build_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Building imai's model.");

    const auto stopchk1 = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    // ====================================================== //
    // =================== TIGHTER BOUNDS =================== //
    // ====================================================== //

    unsigned int max_steps{static_cast<unsigned int>(inst.m_opt)};
    if (env.tight_bounds) {
        // number of variables
        if (inst.n_opt < max_steps) max_steps = inst.n_opt;

        // max number of steps to reach heuristic
        if (env.heur != "none") {
            unsigned int min_act_cost{inst.actions[0].cost + 1};  // +1 to avoid it being 0
            unsigned int n_act_zerocost{0};
            for (const auto& act_i : inst.act_rem) {
                if (inst.actions[act_i].cost == 0)
                    n_act_zerocost++;
                else if (inst.actions[act_i].cost < min_act_cost)
                    min_act_cost = inst.actions[act_i].cost;
            }
            const unsigned int nsteps{static_cast<unsigned int>(std::ceil(static_cast<double>(inst.best_sol.cost) / min_act_cost)) + n_act_zerocost};
            if (nsteps < max_steps) max_steps = nsteps;
        }
    }
    stopchk1();

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //
    // (section 3 of Imai's paper)

    size_t curr_col{0};

    double* objs{new double[inst.m_opt]};
    double* lbs{new double[inst.m_opt]};
    double* ubs{new double[inst.m_opt]};
    char* types{new char[inst.m_opt]};

    auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
        delete[] types;
        types = nullptr;
        delete[] ubs;
        ubs = nullptr;
        delete[] lbs;
        lbs = nullptr;
        delete[] objs;
        objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];
    };

    const auto stopchk2 = [&objs, &lbs, &ubs, &types]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] types;
            types = nullptr;
            delete[] ubs;
            ubs = nullptr;
            delete[] lbs;
            lbs = nullptr;
            delete[] objs;
            objs = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // -------- actions ------- //
    const size_t act_start{curr_col};
    size_t count{0};
    for (const auto& act_i : inst.act_rem) {
        objs[count] = static_cast<double>(inst.actions[act_i].cost);
        lbs[count] = (inst.act_f[act_i] ? 1 : 0);
        ubs[count] = 1;
        types[count++] = 'B';
    }
    INTCHECK_ASSERT_LOG(log, count == inst.m_opt);
    curr_col += inst.m_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // --- action timestamps -- //
    const size_t tact_start{curr_col};
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        objs[count] = 0;
        lbs[count] = (inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : 0);
        ubs[count] = (inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : max_steps);
        types[count++] = 'I';
    }
    INTCHECK_ASSERT_LOG(log, count == inst.m_opt);
    curr_col += inst.m_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    resize_cpx_arrays(inst.n_opt);

    // ------- variables ------ //
    const size_t var_start{curr_col};
    count = 0;
    for (const auto& var_i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = ((inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0);
        ubs[count] = 1;
        types[count++] = 'B';
    }
    INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
    curr_col += inst.n_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // -- variable timestamps - //
    const size_t tvar_start{curr_col};
    count = 0;
    for (const auto& i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = (inst.var_t[i] >= 0 ? inst.var_t[i] : 1);  // 1 since with no initial state, each variable needs to be archieved first
        ubs[count] = (inst.var_t[i] >= 0 ? inst.var_t[i] : max_steps);
        types[count++] = 'I';
    }
    INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
    curr_col += inst.n_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // --- first archievers --- //
    // FIXME: Find a way to add only necessary first archievers...
    const size_t fa_start{curr_col};
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        size_t count_var{0};
        for (const auto& var_i : inst.var_rem) {
            objs[count_var] = 0;
            lbs[count_var] = (inst.fadd_f[act_i][var_i] ? 1 : 0);
            ubs[count_var] = ((!inst.actions[act_i].eff[var_i] || inst.fadd_e[act_i][var_i]) ? 0 : 1);
            types[count_var++] = 'B';
        }
        INTCHECK_ASSERT_LOG(log, count_var == inst.n_opt);
        curr_col += inst.n_opt;
        ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
        count++;
        stopchk2();
    }

    delete[] types;
    types = nullptr;
    delete[] ubs;
    ubs = nullptr;
    delete[] lbs;
    lbs = nullptr;
    delete[] objs;
    objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //
    // (section 3 of Imai's paper)

    // accessing cplex variables
    const auto get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
    const auto get_tact_idx = [&inst, &tact_start](size_t idx) { return static_cast<int>(tact_start + inst.act_opt_conv[idx]); };
    const auto get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
    const auto get_tvar_idx = [&inst, &tvar_start](size_t idx) { return static_cast<int>(tvar_start + inst.var_opt_conv[idx]); };
    const auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_idx) {
        return static_cast<int>(fa_start + inst.act_opt_conv[act_idx] * inst.n_opt + inst.var_opt_conv[var_idx]);
    };

    int* ind_c1{new int[inst.m_opt + 1]};
    double* val_c1{new double[inst.m_opt + 1]};
    int ind_c2_4[2], ind_c5[3];
    double val_c2_4[2], val_c5[3];
    constexpr char sense_l{'L'}, sense_e{'E'};
    constexpr double rhs_c1_2_4{0};
    const double rhs_c5{static_cast<double>(max_steps)};
    constexpr int begin{0};

    std::vector<int*> ind_c3(inst.n_opt);
    std::vector<double*> val_c3(inst.n_opt);
    std::vector<int> nnz_c3(inst.n_opt);
    std::vector<double> rhs_c3(inst.n_opt);

    const auto stopchk3 = [&inst, &ind_c1, &val_c1, &ind_c3, &val_c3]() {
        if (CHECK_STOP()) [[unlikely]] {
            for (size_t var_i = 0; var_i < inst.n_opt; var_i++) {
                delete[] ind_c3[var_i];
                ind_c3[var_i] = nullptr;
                delete[] val_c3[var_i];
                val_c3[var_i] = nullptr;
            }
            delete[] val_c1;
            val_c1 = nullptr;
            delete[] ind_c1;
            ind_c1 = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    for (const auto& i : inst.var_rem) {
        ind_c3[inst.var_opt_conv[i]] = new int[inst.m_opt + 1];
        val_c3[inst.var_opt_conv[i]] = new double[inst.m_opt + 1];
        nnz_c3[inst.var_opt_conv[i]] = 0;
        rhs_c3[inst.var_opt_conv[i]] = 0;
        ind_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = get_var_idx(i);
        val_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = 1;
        nnz_c3[inst.var_opt_conv[i]]++;
    }
    stopchk3();

    for (const auto& act_i : inst.act_rem) {
        constexpr int nnz_c2_4{2};
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            // constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
            ind_c1[0] = get_act_idx(act_i);
            val_c1[0] = 1;
            ind_c1[1] = get_var_idx(var_i);
            val_c1[1] = -1;
            int nnz0{2};
            // (section 4.6 of Imai's paper)
            for (const auto& inverse_action : inst.act_inv[act_i]) {
                if (inst.actions[inverse_action].eff[var_i]) {
                    ind_c1[nnz0] = get_fa_idx(inverse_action, var_i);
                    val_c1[nnz0++] = 1;
                }
            }
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz0, &rhs_c1_2_4, &sense_l, &begin, ind_c1, val_c1, nullptr, nullptr));
            // constraint 4: t_vj <= t_a, vj in pre(a)
            ind_c2_4[0] = get_tvar_idx(var_i);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_tact_idx(act_i);
            val_c2_4[1] = -1;
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sense_l, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
        }
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            constexpr int nnz_c5{3};
            // constraint 2: z_avj <= x_a, vj in eff(a)
            ind_c2_4[0] = get_fa_idx(act_i, var_i);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_act_idx(act_i);
            val_c2_4[1] = -1;
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sense_l, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
            // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
            ind_c5[0] = get_tact_idx(act_i);
            val_c5[0] = 1;
            ind_c5[1] = get_tvar_idx(var_i);
            val_c5[1] = -1;
            ind_c5[2] = get_fa_idx(act_i, var_i);
            val_c5[2] = max_steps + 1;
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c5, &rhs_c5, &sense_l, &begin, ind_c5, val_c5, nullptr, nullptr));
            // constraint 3: I(v_j) + sum(z_avj) = y_vj
            ind_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = get_fa_idx(act_i, var_i);
            val_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = -1;
            nnz_c3[inst.var_opt_conv[var_i]]++;
        }
        stopchk3();
    }

    for (size_t var_i = 0; var_i < inst.n_opt; var_i++)
        ASSERT_LOG(log,
                   !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c3[var_i], &rhs_c3[var_i], &sense_e, &begin, ind_c3[var_i], val_c3[var_i], nullptr, nullptr));

    for (size_t var_i = 0; var_i < inst.n_opt; var_i++) {
        delete[] ind_c3[var_i];
        ind_c3[var_i] = nullptr;
        delete[] val_c3[var_i];
        val_c3[var_i] = nullptr;
    }

    if (env.tight_bounds) {
        double rhs{static_cast<double>(max_steps)};
        size_t nnz{0};
        for (const auto& act_i : inst.act_rem) {
            if (inst.act_f[act_i]) {
                rhs--;
                continue;
            }
            ind_c1[nnz] = get_act_idx(act_i);
            val_c1[nnz++] = 1;
        }
        ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs, &sense_l, &begin, ind_c1, val_c1, nullptr, nullptr));
    }

    delete[] val_c1;
    val_c1 = nullptr;
    delete[] ind_c1;
    ind_c1 = nullptr;

    // ASSERT_LOG(log, !CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR"/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

static void cpx_post_warmstart_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env,
                                    const logger& log) {
    ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

    binary_set state{inst.n};

    const auto& warm_start{inst.best_sol.plan};

    const size_t ncols{static_cast<size_t>(CPXgetnumcols(cpxenv, cpxlp))};
    int* cpx_sol_ind{new int[ncols]};
    double* cpx_sol_val{new double[ncols]};
    for (size_t i = 0; i < ncols; i++) {
        cpx_sol_ind[i] = i;
        cpx_sol_val[i] = 0;
    }

    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};
    unsigned int timestamp{0};

#if HPLUS_INTCHECK
    binary_set fixed_act_check{inst.act_f}, fixed_var_check{inst.var_f}, fixed_t_act_check{inst.m}, fixed_t_var_check{inst.n};
    std::vector<binary_set> fixed_fadd_check(inst.m);
    for (size_t i = 0; i < inst.m; i++) fixed_fadd_check[i] = inst.fadd_f[i];
    for (size_t i = 0; i < inst.m; i++)
        if (inst.act_t[i] >= 0) fixed_t_act_check.add(i);
    for (size_t i = 0; i < inst.n; i++)
        if (inst.var_t[i] >= 0) fixed_t_var_check.add(i);
#endif

    for (const auto& act_i : warm_start) {
#if HPLUS_INTCHECK
        ASSERT_LOG(log, !(inst.act_t[act_i] >= 0 && timestamp != static_cast<unsigned int>(inst.act_t[act_i])));
        ASSERT_LOG(log, !inst.act_e[act_i]);
        fixed_act_check.remove(act_i);
        fixed_t_act_check.remove(act_i);
#endif
        size_t cpx_act_idx = inst.act_opt_conv[act_i];
        cpx_sol_ind[cpx_act_idx] = static_cast<int>(cpx_act_idx);
        cpx_sol_val[cpx_act_idx] = 1;
        size_t cpx_tact_idx = inst.m_opt + inst.act_opt_conv[act_i];
        cpx_sol_ind[cpx_tact_idx] = static_cast<int>(cpx_tact_idx);
        cpx_sol_val[cpx_tact_idx] = static_cast<int>(timestamp);
        timestamp++;
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            if (state[var_i]) continue;

#if HPLUS_INTCHECK
            ASSERT_LOG(log, !(inst.var_t[var_i] >= 0 && timestamp != static_cast<unsigned int>(inst.var_t[var_i])));
            ASSERT_LOG(log, !inst.var_e[var_i]);
            ASSERT_LOG(log, !inst.fadd_e[act_i][var_i]);
            fixed_var_check.remove(var_i);
            fixed_t_var_check.remove(var_i);
            fixed_fadd_check[act_i].remove(var_i);
#endif

            size_t cpx_var_idx = 2 * inst.m_opt + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_var_idx] = static_cast<int>(cpx_var_idx);
            cpx_sol_val[cpx_var_idx] = 1;
            size_t cpx_tvar_idx = 2 * inst.m_opt + inst.n_opt + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_tvar_idx] = static_cast<int>(cpx_tvar_idx);
            cpx_sol_val[cpx_tvar_idx] = static_cast<int>(timestamp);
            size_t cpx_fad_idx = 2 * inst.m_opt + inst.n_opt * (2 + inst.act_opt_conv[act_i]) + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_fad_idx] = static_cast<int>(cpx_fad_idx);
            cpx_sol_val[cpx_fad_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
    }

#if HPLUS_INTCHECK
    ASSERT_LOG(log, fixed_act_check.empty());
    ASSERT_LOG(log, fixed_var_check.empty());
    ASSERT_LOG(log, fixed_t_act_check.empty());
    ASSERT_LOG(log, fixed_t_var_check.empty());
    for (size_t i = 0; i < inst.m; i++) ASSERT_LOG(log, fixed_fadd_check[i].empty());
#endif
    ASSERT_LOG(log, !CPXaddmipstarts(cpxenv, cpxlp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));

    delete[] cpx_sol_ind;
    cpx_sol_ind = nullptr;
    delete[] cpx_sol_val;
    cpx_sol_val = nullptr;
}

static void store_imai_sol(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Storing imai's solution.");

    // get cplex result (interested only in the sequence of actions [0/inst.m_opt-1] used and its ordering [inst.m_opt/2nact-1])
    double* plan{new double[2 * inst.m_opt]};
    ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, 2 * inst.m_opt - 1));

    // convert to std collections for easier parsing
    std::vector<std::pair<double, size_t>> cpx_result;
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (plan[i] > HPLUS_CPX_INT_ROUNDING) cpx_result.emplace_back(plan[inst.m_opt + i], i);
    }
    delete[] plan;
    plan = nullptr;

    // sort cpx_result based on actions timestamps
    std::sort(cpx_result.begin(), cpx_result.end(),
              [](const std::pair<double, size_t>& x, const std::pair<double, size_t>& y) { return x.first < y.first; });

    // get solution from sorted cpx_result
    std::vector<size_t> solution;
    std::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution),
                   [inst](const std::pair<double, size_t>& p) { return inst.act_cpxtoidx[p.second]; });

    // store solution
    hplus::solution imai_sol{
        solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [inst](const unsigned int acc, const size_t index) {
            return acc + inst.actions[index].cost;
        }))};
    hplus::update_sol(inst, imai_sol, log);
}

// ##################################################################### //
// ############################## RANKOOH ############################## //
// ##################################################################### //

static void cpx_build_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, [[maybe_unused]] const hplus::environment& env,
                              const logger& log) {
    PRINT_VERBOSE(log, "Building rankooh's model.");

    const auto stopchk1 = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    // ====================================================== //
    // ================= VERTEX ELIMINATION ================= //
    // ====================================================== //

    struct node {
        size_t id, deg;
        node(const size_t id, const size_t deg) : id(id), deg(deg) {}
    };
    struct compare_node {
        bool operator()(const node& n1, const node& n2) const { return n1.deg > n2.deg; }
    };
    struct triangle {
        size_t first, second, third;
        triangle(const size_t first, const size_t second, const size_t third) : first(first), second(second), third(third) {}
    };

    std::vector<std::set<size_t>> graph(inst.n);
    std::vector<binary_set> cumulative_graph(inst.n, binary_set(inst.n));
    std::vector<triangle> triangles_list;

    priority_queue<size_t> nodes_queue(2 * inst.n);
    std::vector<size_t> degree_counter(inst.n, 0);

    // G_0
    for (const auto& act_i : inst.act_rem) {
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            for (const auto& var_j : inst.actions[act_i].eff_sparse) {
                if (var_i == var_j) [[unlikely]]
                    continue;

                // added is true if the element was actually inserted in the set
                if (graph[var_i].insert(var_j).second) {
                    degree_counter[var_i] += 1;
                    degree_counter[var_j] += 1;
                }
            }
            cumulative_graph[var_i] |= (inst.actions[act_i].eff);
        }
    }
    stopchk1();

    for (size_t node_i = 0; node_i < inst.n; node_i++) {
        if (degree_counter[node_i] > 0) [[likely]]
            nodes_queue.push(node_i, degree_counter[node_i]);
    }

    // G_i (min degree heuristics)
    for ([[maybe_unused]] size_t _ = inst.var_rem.size(); _--;) {
        if (nodes_queue.empty()) [[unlikely]]
            break;
        size_t idx{nodes_queue.top()};
        nodes_queue.pop();

        // graph structure:
        // | \       > |
        // p -> idx -> q
        // | /       > |

        std::set<size_t> new_nodes;

        for (const auto& p : inst.var_rem) {
            if (graph[p].find(idx) == graph[p].end()) continue;

            for (const auto& q : graph[idx]) {
                if (p == q) [[unlikely]]
                    continue;

                // add edge p - q
                // check.second is true if the element was actually inserted in
                // the set
                if (graph[p].insert(q).second) {
                    degree_counter[p] += 1;
                    degree_counter[q] += 1;
                }

                // update the overall graph
                cumulative_graph[p].add(q);
            }

            // remove the edge p - idx
            graph[p].erase(idx);
            degree_counter[p] -= 1;
            new_nodes.insert(p);

            // update triangles list
            for (const auto& q : graph[idx]) {
                if (p != q) [[likely]]
                    triangles_list.emplace_back(p, idx, q);
            }
        }

        // remove the edge idx - q
        for (const auto& q : graph[idx]) {
            degree_counter[q] -= 1;
            new_nodes.insert(q);
        }
        graph[idx].clear();
        degree_counter[idx] = 0;

        // Update the priority queue
        for (const auto& x : new_nodes) {
            if (degree_counter[x] > 0 && nodes_queue.has(x)) nodes_queue.change(x, degree_counter[x]);
        }

#if HPLUS_INTCHECK  // care: this takes HUGE amount of time
        for (size_t node_i = 0; node_i < inst.n; node_i++) {
            size_t i_cnt{0};
            i_cnt += graph[node_i].size();
            for (size_t tmp_j = 0; tmp_j < inst.n; tmp_j++) {
                for (const auto& tmp_k : graph[tmp_j]) {
                    if (tmp_k == node_i) i_cnt += 1;
                }
            }
            ASSERT_LOG(log, i_cnt == degree_counter[node_i]);
        }
#endif
        stopchk1();
    }

    // ====================================================== //
    // =================== TIGHTER BOUNDS =================== //
    // ====================================================== //

    unsigned int max_steps{static_cast<unsigned int>(inst.m_opt)};
    if (env.tight_bounds) {
        // number of variables
        if (inst.n_opt < max_steps) max_steps = inst.n_opt;

        // max number of steps to reach heuristic
        if (env.heur != "none") {
            unsigned int min_act_cost{inst.actions[0].cost + 1};  // +1 to avoid it being 0
            unsigned int n_act_zerocost{0};
            for (const auto& act_i : inst.act_rem) {
                if (inst.actions[act_i].cost == 0)
                    n_act_zerocost++;
                else if (inst.actions[act_i].cost < min_act_cost)
                    min_act_cost = inst.actions[act_i].cost;
            }
            const unsigned int nsteps{static_cast<unsigned int>(std::ceil(static_cast<double>(inst.best_sol.cost) / min_act_cost)) + n_act_zerocost};
            if (nsteps < max_steps) max_steps = nsteps;
        }
    }
    stopchk1();

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    size_t curr_col{0};
    double* objs{new double[inst.m_opt]};
    double* lbs{new double[inst.m_opt]};
    double* ubs{new double[inst.m_opt]};
    char* types{new char[inst.m_opt]};

    const auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
        delete[] types;
        types = nullptr;
        delete[] ubs;
        ubs = nullptr;
        delete[] lbs;
        lbs = nullptr;
        delete[] objs;
        objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];
    };

    const auto stopchk2 = [&objs, &lbs, &ubs, &types]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] types;
            types = nullptr;
            delete[] ubs;
            ubs = nullptr;
            delete[] lbs;
            lbs = nullptr;
            delete[] objs;
            objs = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // -------- actions ------- //
    const size_t act_start{curr_col};
    size_t count{0};
    for (const auto& act_i : inst.act_rem) {
        objs[count] = static_cast<double>(inst.actions[act_i].cost);
        lbs[count] = (inst.act_f[act_i] ? 1 : 0);
        ubs[count] = 1;
        types[count++] = 'B';
    }
    INTCHECK_ASSERT_LOG(log, count == inst.m_opt);

    curr_col += inst.m_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    resize_cpx_arrays(inst.n_opt);

    // --- first archievers --- //
    // FIXME: Find a way to add only necessary first archievers...
    const size_t fa_start{curr_col};
    std::vector<size_t> fa_individual_start(inst.m_opt);
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        fa_individual_start[count] = count * inst.n_opt;
        size_t count_var{0};
        for (const auto& var_i : inst.var_rem) {
            objs[count_var] = 0;
            lbs[count_var] = inst.fadd_f[act_i][var_i] ? 1 : 0;
            ubs[count_var] = (!inst.actions[act_i].eff[var_i] || inst.fadd_e[act_i][var_i]) ? 0 : 1;
            types[count_var++] = 'B';
        }
        INTCHECK_ASSERT_LOG(log, count_var == inst.n_opt);
        curr_col += inst.n_opt;
        ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
        count++;
        stopchk2();
    }

    // ------- variables ------ //
    const size_t var_start{curr_col};
    count = 0;
    for (const auto& var_i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = (inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }

    INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
    curr_col += inst.n_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    // vertex elimination graph edges
    const size_t veg_edges_start{curr_col};
    for (const auto& var_i : inst.var_rem) {
        count = 0;
        for (const auto& var_j : inst.var_rem) {
            objs[count] = 0;
            lbs[count] = 0;
            ubs[count] = cumulative_graph[var_i][var_j] ? 1 : 0;
            types[count++] = 'B';
        }
        INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
        ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
        stopchk2();
    }
    curr_col += inst.n_opt * inst.n_opt;

    delete[] types;
    types = nullptr;
    delete[] ubs;
    ubs = nullptr;
    delete[] lbs;
    lbs = nullptr;
    delete[] objs;
    objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const auto get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
    const auto get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
    const auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_idx) {
        return static_cast<int>(fa_start + inst.act_opt_conv[act_idx] * inst.n_opt + inst.var_opt_conv[var_idx]);
    };
    const auto get_veg_idx = [&inst, &veg_edges_start](size_t idx_i, size_t idx_j) {
        return static_cast<int>(veg_edges_start + inst.var_opt_conv[idx_i] * inst.n_opt + inst.var_opt_conv[idx_j]);
    };

    int* ind{new int[inst.m_opt + 1]};
    double* val{new double[inst.m_opt + 1]};
    int nnz{0};
    constexpr char sense_e{'E'}, sense_l{'L'};
    constexpr double rhs_0{0}, rhs_1{1};
    constexpr int begin{0};

    const auto stopchk3 = [&ind, &val]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] val;
            val = nullptr;
            delete[] ind;
            ind = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // p = \sum_{a\in A,p\in eff(a)}(fadd(a, p)), \forall p \in V
    for (const auto& var_i : inst.var_rem) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        bool fixed = false;
        for (const auto& act_i : inst.act_with_eff[var_i]) {
            // if one first adder is fixed, then also the variable should be fixed
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                fixed = true;
                ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                break;
            }
            // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            ind[nnz] = get_fa_idx(act_i, var_i);
            val[nnz++] = -1;
        }
        // if we fixed the variable due to a fixed first adder (note also that if we had a fixed first adder, we already have all other first
        // adders for that effect eliminated), we don't need the constraint we're adding.
        if (fixed) continue;

        // if nnz == 1, then we'd have p = 0, meaning we could simply fix this variable to 0
        if (nnz == 1) {
            const char fix = 'B';
            const double zero = 0;
            ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &zero));
        } else
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr));
        stopchk3();
    }

    // \sum_{a\in A, p\in eff(a), q\in pre(a)}(fadd(a, p)) <= q, \forall p,q\in V
    for (const auto& var_i : inst.var_rem) {
        for (const auto& var_j : inst.var_rem) {
            nnz = 0;
            ind[nnz] = get_var_idx(var_j);
            val[nnz++] = -1;
            bool fixed = false;
            for (const auto& act_i : inst.act_with_eff[var_i]) {
                if (!inst.actions[act_i].pre[var_j]) continue;
                // if the first adder is fixed, than we have 1 <= q, hence we can directly fix q
                if (inst.fadd_f[act_i][var_i]) {
                    const char fix = 'B';
                    const double one = 1;
                    fixed = true;
                    ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                    break;
                }
                // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
                else if (inst.fadd_e[act_i][var_i])
                    continue;
                ind[nnz] = get_fa_idx(act_i, var_i);
                val[nnz++] = 1;
            }
            // since we have a fixed first adder, all other first adder for that effect are eliminated, so we don't need the constraint
            if (fixed) continue;
            // if nnz == 1 than we have -p <= 0, hence it's always true, we can ignore this constraint
            if (nnz != 1) ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr));
            stopchk3();
        }
    }

    if (env.tight_bounds) {
        double rhs{static_cast<double>(max_steps)};
        nnz = 0;
        for (const auto& act_i : inst.act_rem) {
            if (inst.act_f[act_i]) {
                rhs--;
                continue;
            }
            ind[nnz] = get_act_idx(act_i);
            val[nnz++] = 1;
        }
        ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs, &sense_l, &begin, ind, val, nullptr, nullptr));
    }

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    int ind_c5_c6_c7[2], ind_c8[3];
    double val_c5_c6_c7[2], val_c8[3];

    // fadd(a, p) <= a, \forall a\in A, p\in eff(a)
    for (const auto& act_i : inst.act_rem) {
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            ind_c5_c6_c7[0] = get_act_idx(act_i);
            val_c5_c6_c7[0] = -1;
            // if the first adder was fixed, we can directly fix the action insthead of adding the constraint
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind_c5_c6_c7, &fix, &one));
                continue;
            }
            // if the first adder was eliminated, we can skip the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            ind_c5_c6_c7[1] = get_fa_idx(act_i, var_i);
            val_c5_c6_c7[1] = 1;
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
        }
        stopchk1();
    }

    // fadd(a, q) <= veg(p, q) \forall a\in A, \forall p\in pre(a), \forall q\in eff(a) (V.E.G.)
    for (const auto& act_i : inst.act_rem) {
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            for (const auto& var_j : inst.actions[act_i].eff_sparse) {
                ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
                val_c5_c6_c7[0] = -1;
                ind_c5_c6_c7[1] = get_fa_idx(act_i, var_j);
                val_c5_c6_c7[1] = 1;
                // if the first adder was fixed, we can directly fix also the VEG variable
                if (inst.fadd_f[act_i][var_j]) {
                    const char fix = 'B';
                    const double one = 1;
                    ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind_c5_c6_c7, &fix, &one));
                    continue;
                }
                // if the VEG variable was eliminated, we can directly eliminate the first adder too
                else if (!cumulative_graph[var_i][var_j]) {
                    const char fix = 'B';
                    const double zero = 0;
                    ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, &(ind_c5_c6_c7[1]), &fix, &zero));
                    continue;
                }
                ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
            }
            stopchk1();
        }
    }

    // veg(p, q) + veg(q, p) <= 1 (V.E.G.)
    for (const auto& var_i : inst.var_rem) {
        for (const auto& var_j : cumulative_graph[var_i]) {
            // if either VEG variable was eliminated, we can skip the constraint
            if (!cumulative_graph[var_i][var_j] || !cumulative_graph[var_j][var_i]) continue;
            ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
            val_c5_c6_c7[0] = 1;
            ind_c5_c6_c7[1] = get_veg_idx(var_j, var_i);
            val_c5_c6_c7[1] = 1;
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
            stopchk1();
        }
    }

    // veg(a, b) + veg(b, c) <= veg(a, c) (V.E.G.)
    for (const auto& [a, b, c] : triangles_list) {
        ind_c8[0] = get_veg_idx(a, b);
        val_c8[0] = 1;
        ind_c8[1] = get_veg_idx(b, c);
        val_c8[1] = 1;
        ind_c8[2] = get_veg_idx(a, c);
        val_c8[2] = -1;
        // if veg(a, c) is eliminated, we can simply eliminate the other two VEG varliables
        if (!cumulative_graph[a][c]) {
            char fix[2];
            fix[0] = 'B';
            fix[1] = 'B';
            double zero[2];
            zero[0] = 0;
            zero[1] = 0;
            ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 2, ind_c8, fix, zero));
            continue;
        }
        ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr));
        stopchk1();
    }

    // ASSERT_LOG(log, !CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

static void cpx_post_warmstart_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env,
                                       const logger& log) {
    ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

    binary_set state{inst.n};

    const auto& warm_start{inst.best_sol.plan};

    const size_t ncols{static_cast<size_t>(CPXgetnumcols(cpxenv, cpxlp))};
    int* cpx_sol_ind{new int[ncols]};
    double* cpx_sol_val{new double[ncols]};
    for (size_t i = 0; i < ncols; i++) {
        cpx_sol_ind[i] = i;
        cpx_sol_val[i] = 0;
    }

    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};

#if HPLUS_INTCHECK
    binary_set fixed_act_check{inst.act_f}, fixed_var_check{inst.var_f}, fixed_t_act_check{inst.m}, fixed_t_var_check{inst.n};
    std::vector<binary_set> fixed_fadd_check(inst.m);
    for (size_t i = 0; i < inst.m; i++) fixed_fadd_check[i] = inst.fadd_f[i];
    for (size_t i = 0; i < inst.m; i++)
        if (inst.act_t[i] >= 0) fixed_t_act_check.add(i);
    for (size_t i = 0; i < inst.n; i++)
        if (inst.var_t[i] >= 0) fixed_t_var_check.add(i);
    unsigned int timestamp{0};
#endif

    for (const auto& act_i : warm_start) {
#if HPLUS_INTCHECK
        ASSERT_LOG(log, !(inst.act_t[act_i] >= 0 && timestamp != static_cast<unsigned int>(inst.act_t[act_i])));
        ASSERT_LOG(log, !inst.act_e[act_i]);
        fixed_act_check.remove(act_i);
        fixed_t_act_check.remove(act_i);
        timestamp++;
#endif
        size_t cpx_act_idx = inst.act_opt_conv[act_i];
        cpx_sol_ind[cpx_act_idx] = static_cast<int>(cpx_act_idx);
        cpx_sol_val[cpx_act_idx] = 1;
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            if (state[var_i]) continue;

#if HPLUS_INTCHECK
            ASSERT_LOG(log, !(inst.var_t[var_i] >= 0 && timestamp != static_cast<unsigned int>(inst.var_t[var_i])));
            ASSERT_LOG(log, !inst.var_e[var_i]);
            ASSERT_LOG(log, !inst.fadd_e[act_i][var_i]);
            fixed_var_check.remove(var_i);
            fixed_t_var_check.remove(var_i);
            fixed_fadd_check[act_i].remove(var_i);
#endif

            size_t cpx_var_idx = inst.m_opt * (1 + inst.n_opt) + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_var_idx] = static_cast<int>(cpx_var_idx);
            cpx_sol_val[cpx_var_idx] = 1;
            size_t cpx_fad_idx = inst.m_opt + inst.act_opt_conv[act_i] * inst.n_opt + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_fad_idx] = static_cast<int>(cpx_fad_idx);
            cpx_sol_val[cpx_fad_idx] = 1;
            for (const auto& var_j : inst.actions[act_i].pre_sparse) {
                size_t cpx_veg_idx = inst.m_opt * (1 + inst.n_opt) + inst.n_opt * (1 + inst.var_opt_conv[var_j]) + inst.var_opt_conv[var_i];
                cpx_sol_ind[cpx_veg_idx] = static_cast<int>(cpx_veg_idx);
                cpx_sol_val[cpx_veg_idx] = 1;
            }
        }
        state |= inst.actions[act_i].eff;
    }

#if HPLUS_INTCHECK
    ASSERT_LOG(log, fixed_act_check.empty());
    ASSERT_LOG(log, fixed_var_check.empty());
    ASSERT_LOG(log, fixed_t_act_check.empty());
    ASSERT_LOG(log, fixed_t_var_check.empty());
    for (size_t i = 0; i < inst.m; i++) ASSERT_LOG(log, fixed_fadd_check[i].empty());
#endif

    ASSERT_LOG(log, !CPXaddmipstarts(cpxenv, cpxlp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));
    delete[] cpx_sol_ind;
    cpx_sol_ind = nullptr;
    delete[] cpx_sol_val;
    cpx_sol_val = nullptr;
}

static void store_rankooh_sol(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Storing rankooh's solution.");

    double* plan{new double[inst.m_opt + inst.m_opt * inst.n_opt]};
    ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt + inst.m_opt * inst.n_opt - 1));

    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (size_t act_i_cpx = 0, fadd_i = inst.m_opt; act_i_cpx < inst.m_opt; act_i_cpx++, fadd_i += inst.n_opt) {
        bool set_zero{true};
        for (size_t var_i_cpx = 0; var_i_cpx < inst.n_opt; var_i_cpx++) {
            if (plan[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
                INTCHECK_ASSERT_LOG(log, plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
                set_zero = false;
                break;
            }
        }
        if (set_zero) plan[act_i_cpx] = 0;
    }

    // convert to std collections for easier parsing
    std::vector<size_t> cpx_result;
    cpx_result.reserve(inst.m_opt);
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (plan[i] > HPLUS_CPX_INT_ROUNDING) cpx_result.push_back(inst.act_cpxtoidx[i]);
    }
    delete[] plan;
    plan = nullptr;

    std::vector<size_t> solution;
    solution.reserve(inst.m_opt);
    binary_set remaining{cpx_result.size(), true}, state{inst.n};

    while (!remaining.empty()) {
        bool intcheck{false};
        for (const auto& i : remaining) {
            if (!state.contains(inst.actions[cpx_result[i]].pre)) continue;

            remaining.remove(i);
            state |= inst.actions[cpx_result[i]].eff;
            solution.push_back(cpx_result[i]);
            intcheck = true;
        }
        ASSERT_LOG(log, intcheck);
    }

    // store solution
    hplus::solution rankooh_sol{
        solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [&inst](const unsigned int acc, const size_t index) {
            return acc + inst.actions[index].cost;
        }))};
    hplus::update_sol(inst, rankooh_sol, log);
}

// ##################################################################### //
// ############################ DYNAMIC TIME ########################### //
// ##################################################################### //

// straight from Johnson's paper
static void unblock(size_t u, std::vector<bool>& blocked, std::vector<std::set<size_t>>& block_map) {
    blocked[u] = false;
    for (int w : block_map[u])
        if (blocked[w]) unblock(w, blocked, block_map);
    block_map[u].clear();
}

// straight from Johnson's paper
static bool circuit(size_t v, size_t start, std::vector<std::vector<size_t>>& graph, std::vector<bool>& blocked,
                    std::vector<std::set<size_t>>& block_map, std::stack<size_t>& path, std::vector<std::vector<size_t>>& cycles) {
    bool found_cycle = false;
    path.push(v);
    blocked[v] = true;
    for (auto w : graph[v]) {
        if (w == start) {
            // found a cycle
            std::stack<size_t> copy = path;
            std::vector<size_t> cycle;
            while (!copy.empty()) {
                cycle.push_back(copy.top());
                copy.pop();
            }
            std::reverse(cycle.begin(), cycle.end());
            cycles.push_back(cycle);
            found_cycle = true;
        } else if (!blocked[w] && circuit(w, start, graph, blocked, block_map, path, cycles))
            found_cycle = true;
    }

    if (found_cycle)
        unblock(v, blocked, block_map);
    else
        for (auto w : graph[v]) block_map[w].insert(v);

    path.pop();
    return found_cycle;
}

typedef struct {
    hplus::instance& inst;
    hplus::environment& env;
    hplus::statistics& stats;
    const logger& log;
} cpx_callback_user_handle;

// TODO: Remake with hybrid dynamic-rankooh
static int CPXPUBLIC cpx_dynamic_time_callback(CPXCALLBACKCONTEXTptr context, CPXLONG context_id, void* user_handle) {
    auto& [inst, env, stats, log] = *static_cast<cpx_callback_user_handle*>(user_handle);

    ASSERT_LOG(log, context_id == CPX_CALLBACKCONTEXT_CANDIDATE);
    double start_time{env.timer.get_time()};

    // get candidate point
    double* xstar{new double[inst.m_opt + inst.m_opt * inst.n_opt]};
    double cost{CPX_INFBOUND};
    ASSERT_LOG(log, !CPXcallbackgetcandidatepoint(context, xstar, 0, inst.m_opt + inst.m_opt * inst.n_opt - 1, &cost));

    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything (null cost))
    std::vector<binary_set> cpx_var_archieved(inst.m_opt, binary_set(inst.n_opt));
    for (size_t act_i_cpx = 0, fadd_i = inst.m_opt; act_i_cpx < inst.m_opt; act_i_cpx++, fadd_i += inst.n_opt) {
        bool set_zero = true;
        for (size_t var_i_cpx = 0; var_i_cpx < inst.n_opt; var_i_cpx++) {
            if (xstar[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
                INTCHECK_ASSERT_LOG(log, xstar[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
                set_zero = false;
                cpx_var_archieved[act_i_cpx].add(var_i_cpx);
            }
        }
        if (set_zero) xstar[act_i_cpx] = 0;
    }

    // split actions among used and unused
    std::vector<size_t> actions_used;
    std::vector<size_t> actions_unused;
    for (size_t act_i_cpx = 0; act_i_cpx < inst.m_opt; act_i_cpx++) {
        if (xstar[act_i_cpx] < HPLUS_CPX_INT_ROUNDING)
            actions_unused.push_back(act_i_cpx);
        else
            actions_used.push_back(act_i_cpx);
    }
    delete[] xstar;
    xstar = nullptr;

    // split used actions among reachable and unreachable
    std::vector<size_t> actions_reachable;
    std::vector<size_t> actions_unreachable{actions_used};
    binary_set current_state{inst.n};
    bool loop;
    do {
        loop = false;
        std::vector<size_t> new_reached_actions;
        for (auto act_i_cpx : actions_unreachable) {
            size_t act_i = inst.act_cpxtoidx[act_i_cpx];
            if (current_state.contains(inst.actions[act_i].pre)) {
                loop = true;
                current_state |= inst.actions[act_i].eff;
                new_reached_actions.push_back(act_i_cpx);
                actions_reachable.push_back(act_i_cpx);
            }
        }
        auto it = std::set_difference(actions_unreachable.begin(), actions_unreachable.end(), new_reached_actions.begin(), new_reached_actions.end(),
                                      actions_unreachable.begin());
        actions_unreachable.resize(it - actions_unreachable.begin());
    } while (loop);

    // solution is feasible
    if (actions_unreachable.empty()) return 0;

#if HPLUS_VERBOSE >= 20
    int itcount = 0;
    ASSERT_LOG(log, !CPXcallbackgetinfoint(context, CPXCALLBACKINFO_ITCOUNT, &itcount));
    if (itcount % 10 == 0) {
        double lower_bound = CPX_INFBOUND;
        ASSERT_LOG(log, !CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &lower_bound));
        double incumbent = CPX_INFBOUND;
        ASSERT_LOG(log, !CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_SOL, &incumbent));
        double gap = (1 - lower_bound / incumbent) * 100;
        log.print_info("Pruned infeasible solution - cost : %7d - incumbent : %d - gap : %6.2f%%.", (int)cost, (int)incumbent, gap);
    }
#endif

    std::vector<std::vector<size_t>> act_with_pre(inst.n_opt);
    for (auto var_i : inst.var_rem)
        for (auto act_i_cpx : actions_unreachable)
            if (inst.actions[inst.act_cpxtoidx[act_i_cpx]].pre[var_i]) act_with_pre[inst.var_opt_conv[var_i]].push_back(act_i_cpx);

    // build graph G=<A, {(ai, aj) ai, aj in A | ai is first archiever (in the current solution) to a precondition of aj}>
    std::vector<std::set<size_t>> graph(inst.m_opt);
    for (auto act_i_cpx : actions_unreachable)
        for (auto var_i_cpx : cpx_var_archieved[act_i_cpx]) graph[act_i_cpx].insert(act_with_pre[var_i_cpx].begin(), act_with_pre[var_i_cpx].end());

    // find all simple cycles (Johnson's paper)
    size_t n = graph.size();
    std::vector<bool> blocked(n, false);
    std::vector<std::set<size_t>> bock_map(n);
    std::stack<size_t> path;
    std::vector<std::vector<size_t>> cycles;

    for (auto act_i_cpx : actions_unreachable) {
        // find subgraph with vertices >= act_i_cpx
        std::vector<std::vector<size_t>> subgraph(n);
        for (size_t u = act_i_cpx; u < n; u++)
            for (auto v : graph[u])
                if (v >= act_i_cpx) subgraph[u].push_back(v);

        // run circuit function to find all cycles starting at "act_i_cpx"
        circuit(act_i_cpx, act_i_cpx, subgraph, blocked, bock_map, path, cycles);

        // remove act_i_cpx vertex from the graph
        for (size_t u = 0; u < n; u++) graph[u].erase(act_i_cpx);
    }

    int* ind = new int[inst.n_opt];  // the size is inst.n_opt since we know that there can't be more that one first archiever selected per
                                     // variable, hence the used first archievers are at most inst.n_opt
    double* val = new double[inst.n_opt];
    int nnz = 0;
    double rhs = 0;
    const int izero = 0;
    const char sense_l = 'L';

    std::vector<size_t> var_idx_presim(inst.n_opt);
    size_t var_i_cpx = 0;
    for (auto var_i : inst.var_rem) {
        var_idx_presim[var_i_cpx] = var_i;
        var_i_cpx++;
    }

    // Simple Cycle Cuts
    for (auto cycle : cycles) {
        nnz = 0;

        // first archievers in the cycle
        for (size_t i = 0; i < cycle.size() - 1; i++) {
            for (auto var_i_cpx : cpx_var_archieved[cycle[i]]) {
                if (inst.actions[inst.act_cpxtoidx[cycle[i + 1]]].pre[var_idx_presim[var_i_cpx]]) {
                    ind[nnz] = inst.m_opt + cycle[i] * inst.n_opt + var_i_cpx;  // variable associated to that first archiever
                    val[nnz++] = 1;
                }
            }
        }
        for (auto var_i_cpx : cpx_var_archieved[cycle[cycle.size() - 1]]) {
            if (inst.actions[inst.act_cpxtoidx[cycle[0]]].pre[var_idx_presim[var_i_cpx]]) {
                ind[nnz] = inst.m_opt + cycle[cycle.size() - 1] * inst.n_opt + var_i_cpx;  // variable associated to that first archiever
                val[nnz++] = 1;
            }
        }

        // cannot have all selected (at least one must come from outside the cycle)
        rhs = nnz - 1;

        // TODO: try insthead of forcing a first archiever to be removed, to force an outside first archiever going into a variable in the cycle:
        // 1/size(gamma) * sum_{a\in gamma}(xa) <= sum_{fa\in{first archievers of variables in the cycle}}(x_fa)
        ASSERT_LOG(log, !CPXcallbackrejectcandidate(context, 1, nnz, &rhs, &sense_l, &izero, ind, val));
    }

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    pthread_mutex_lock(&(stats.callback_time_mutex));
    stats.callback += env.timer.get_time() - start_time;
    pthread_mutex_unlock(&(stats.callback_time_mutex));

    return 0;
}

// TODO: Remake with hybrid dynamic-rankooh
static void cpx_build_dynamic_time(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env,
                                   const logger& log) {
    PRINT_VERBOSE(log, "Building dynamic time model.");

    const auto stopchk1 = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    // ====================================================== //
    // =================== TIGHTER BOUNDS =================== //
    // ====================================================== //

    unsigned int max_steps{static_cast<unsigned int>(inst.m_opt)};
    if (env.tight_bounds) {
        // number of variables
        if (inst.n_opt < max_steps) max_steps = inst.n_opt;

        // max number of steps to reach heuristic
        if (env.heur != "none") {
            unsigned int min_act_cost{inst.actions[0].cost + 1};  // +1 to avoid it being 0
            unsigned int n_act_zerocost{0};
            for (const auto& act_i : inst.act_rem) {
                if (inst.actions[act_i].cost == 0)
                    n_act_zerocost++;
                else if (inst.actions[act_i].cost < min_act_cost)
                    min_act_cost = inst.actions[act_i].cost;
            }
            const unsigned int nsteps{static_cast<unsigned int>(std::ceil(static_cast<double>(inst.best_sol.cost) / min_act_cost)) + n_act_zerocost};
            if (nsteps < max_steps) max_steps = nsteps;
        }
    }
    stopchk1();

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    size_t curr_col{0};
    double* objs{new double[inst.m_opt]};
    double* lbs{new double[inst.m_opt]};
    double* ubs{new double[inst.m_opt]};
    char* types{new char[inst.m_opt]};

    const auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
        delete[] types;
        types = nullptr;
        delete[] ubs;
        ubs = nullptr;
        delete[] lbs;
        lbs = nullptr;
        delete[] objs;
        objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];
    };

    const auto stopchk2 = [&objs, &lbs, &ubs, &types]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] types;
            types = nullptr;
            delete[] ubs;
            ubs = nullptr;
            delete[] lbs;
            lbs = nullptr;
            delete[] objs;
            objs = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // -------- actions ------- //
    const size_t act_start{curr_col};
    size_t count{0};
    for (const auto& act_i : inst.act_rem) {
        objs[count] = static_cast<double>(inst.actions[act_i].cost);
        lbs[count] = (inst.act_f[act_i] ? 1 : 0);
        ubs[count] = 1;
        types[count++] = 'B';
    }
    INTCHECK_ASSERT_LOG(log, count == inst.m_opt);

    curr_col += inst.m_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    resize_cpx_arrays(inst.n_opt);

    // --- first archievers --- //
    // FIXME: Find a way to add only necessary first archievers...
    const size_t fa_start{curr_col};
    std::vector<size_t> fa_individual_start(inst.m_opt);
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        fa_individual_start[count] = count * inst.n_opt;
        size_t count_var{0};
        for (const auto& var_i : inst.var_rem) {
            objs[count_var] = 0;
            lbs[count_var] = inst.fadd_f[act_i][var_i] ? 1 : 0;
            ubs[count_var] = (!inst.actions[act_i].eff[var_i] || inst.fadd_e[act_i][var_i]) ? 0 : 1;
            types[count_var++] = 'B';
        }
        INTCHECK_ASSERT_LOG(log, count_var == inst.n_opt);
        curr_col += inst.n_opt;
        ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
        count++;
        stopchk2();
    }

    // ------- variables ------ //
    const size_t var_start{curr_col};
    count = 0;
    for (const auto& var_i : inst.var_rem) {
        objs[count] = 0;
        lbs[count] = (inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }

    INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
    curr_col += inst.n_opt;

    ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
    stopchk2();

    delete[] types;
    types = nullptr;
    delete[] ubs;
    ubs = nullptr;
    delete[] lbs;
    lbs = nullptr;
    delete[] objs;
    objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const auto get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
    const auto get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
    const auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_idx) {
        return static_cast<int>(fa_start + inst.act_opt_conv[act_idx] * inst.n_opt + inst.var_opt_conv[var_idx]);
    };

    int* ind{new int[inst.m_opt + 1]};
    double* val{new double[inst.m_opt + 1]};
    int nnz{0};
    constexpr char sense_e{'E'}, sense_l{'L'};
    constexpr double rhs_0{0}, rhs_1{1};
    constexpr int begin{0};

    const auto stopchk3 = [&ind, &val]() {
        if (CHECK_STOP()) [[unlikely]] {
            delete[] val;
            val = nullptr;
            delete[] ind;
            ind = nullptr;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // p = \sum_{a\in A,p\in eff(a)}(fadd(a, p)), \forall p \in V
    for (const auto& var_i : inst.var_rem) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        bool fixed = false;
        for (const auto& act_i : inst.act_with_eff[var_i]) {
            // if one first adder is fixed, then also the variable should be fixed
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                fixed = true;
                ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                break;
            }
            // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            ind[nnz] = get_fa_idx(act_i, var_i);
            val[nnz++] = -1;
        }
        // if we fixed the variable due to a fixed first adder (note also that if we had a fixed first adder, we already have all other first
        // adders for that effect eliminated), we don't need the constraint we're adding.
        if (fixed) continue;

        // if nnz == 1, then we'd have p = 0, meaning we could simply fix this variable to 0
        if (nnz == 1) {
            const char fix = 'B';
            const double zero = 0;
            ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &zero));
        } else
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr));
        stopchk3();
    }

    // \sum_{a\in A, p\in eff(a), q\in pre(a)}(fadd(a, p)) <= q, \forall p,q\in V
    for (const auto& var_i : inst.var_rem) {
        for (const auto& var_j : inst.var_rem) {
            nnz = 0;
            ind[nnz] = get_var_idx(var_j);
            val[nnz++] = -1;
            bool fixed = false;
            for (const auto& act_i : inst.act_with_eff[var_i]) {
                if (!inst.actions[act_i].pre[var_j]) continue;
                // if the first adder is fixed, than we have 1 <= q, hence we can directly fix q
                if (inst.fadd_f[act_i][var_i]) {
                    const char fix = 'B';
                    const double one = 1;
                    fixed = true;
                    ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind, &fix, &one));
                    break;
                }
                // if the first adder we're about to add to the constraint was eliminated, it's useless to the constraint
                else if (inst.fadd_e[act_i][var_i])
                    continue;
                ind[nnz] = get_fa_idx(act_i, var_i);
                val[nnz++] = 1;
            }
            // since we have a fixed first adder, all other first adder for that effect are eliminated, so we don't need the constraint
            if (fixed) continue;
            // if nnz == 1 than we have -p <= 0, hence it's always true, we can ignore this constraint
            if (nnz != 1) ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr));
            stopchk3();
        }
    }

    // inverse actions constraint
    for (const auto& act_i : inst.act_rem) {
        const auto& inverse_actions = inst.act_inv[act_i];
        if (inverse_actions.empty()) continue;
        nnz = 0;
        ind[nnz] = get_act_idx(act_i);
        val[nnz++] = 1;
        for (const auto& act_j : inverse_actions) {
            ind[nnz] = get_act_idx(act_j);
            val[nnz++] = 1.0 / inverse_actions.size();
        }
        ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_1, &sense_l, &begin, ind, val, nullptr, nullptr));
    }

    if (env.tight_bounds) {
        double rhs{static_cast<double>(max_steps)};
        nnz = 0;
        for (const auto& act_i : inst.act_rem) {
            if (inst.act_f[act_i]) {
                rhs--;
                continue;
            }
            ind[nnz] = get_act_idx(act_i);
            val[nnz++] = 1;
        }
        ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs, &sense_l, &begin, ind, val, nullptr, nullptr));
    }

    delete[] val;
    val = nullptr;
    delete[] ind;
    ind = nullptr;

    int ind_c5_c6_c7[2];
    double val_c5_c6_c7[2];

    // fadd(a, p) <= a, \forall a\in A, p\in eff(a)
    for (const auto& act_i : inst.act_rem) {
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            ind_c5_c6_c7[0] = get_act_idx(act_i);
            val_c5_c6_c7[0] = -1;
            // if the first adder was fixed, we can directly fix the action insthead of adding the constraint
            if (inst.fadd_f[act_i][var_i]) {
                const char fix = 'B';
                const double one = 1;
                ASSERT_LOG(log, !CPXchgbds(cpxenv, cpxlp, 1, ind_c5_c6_c7, &fix, &one));
                continue;
            }
            // if the first adder was eliminated, we can skip the constraint
            else if (inst.fadd_e[act_i][var_i])
                continue;
            ind_c5_c6_c7[1] = get_fa_idx(act_i, var_i);
            val_c5_c6_c7[1] = 1;
            ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
        }
        stopchk1();
    }

    // ASSERT_LOG(log, !CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

// TODO: Remake with hybrid dynamic-rankooh
static void cpx_post_warmstart_dynamic_time(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env,
                                            const logger& log) {
    ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

    binary_set state{inst.n};

    const auto& warm_start{inst.best_sol.plan};

    const size_t ncols{static_cast<size_t>(CPXgetnumcols(cpxenv, cpxlp))};
    int* cpx_sol_ind{new int[ncols]};
    double* cpx_sol_val{new double[ncols]};
    for (size_t i = 0; i < ncols; i++) {
        cpx_sol_ind[i] = i;
        cpx_sol_val[i] = 0;
    }

    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};

#if HPLUS_INTCHECK
    binary_set fixed_act_check{inst.act_f}, fixed_var_check{inst.var_f}, fixed_t_act_check{inst.m}, fixed_t_var_check{inst.n};
    std::vector<binary_set> fixed_fadd_check(inst.m);
    for (size_t i = 0; i < inst.m; i++) fixed_fadd_check[i] = inst.fadd_f[i];
    for (size_t i = 0; i < inst.m; i++)
        if (inst.act_t[i] >= 0) fixed_t_act_check.add(i);
    for (size_t i = 0; i < inst.n; i++)
        if (inst.var_t[i] >= 0) fixed_t_var_check.add(i);
    unsigned int timestamp{0};
#endif

    for (const auto& act_i : warm_start) {
#if HPLUS_INTCHECK
        ASSERT_LOG(log, !(inst.act_t[act_i] >= 0 && timestamp != static_cast<unsigned int>(inst.act_t[act_i])));
        ASSERT_LOG(log, !inst.act_e[act_i]);
        fixed_act_check.remove(act_i);
        fixed_t_act_check.remove(act_i);
        timestamp++;
#endif
        size_t cpx_act_idx = inst.act_opt_conv[act_i];
        cpx_sol_ind[cpx_act_idx] = static_cast<int>(cpx_act_idx);
        cpx_sol_val[cpx_act_idx] = 1;
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            if (state[var_i]) continue;

#if HPLUS_INTCHECK
            ASSERT_LOG(log, !(inst.var_t[var_i] >= 0 && timestamp != static_cast<unsigned int>(inst.var_t[var_i])));
            ASSERT_LOG(log, !inst.var_e[var_i]);
            ASSERT_LOG(log, !inst.fadd_e[act_i][var_i]);
            fixed_var_check.remove(var_i);
            fixed_t_var_check.remove(var_i);
#endif

            size_t cpx_var_idx = inst.m_opt * (1 + inst.n_opt) + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_var_idx] = static_cast<int>(cpx_var_idx);
            cpx_sol_val[cpx_var_idx] = 1;
            size_t cpx_fad_idx = inst.m_opt + inst.act_opt_conv[act_i] * inst.n_opt + inst.var_opt_conv[var_i];
            cpx_sol_ind[cpx_fad_idx] = static_cast<int>(cpx_fad_idx);
            cpx_sol_val[cpx_fad_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
    }

#if HPLUS_INTCHECK
    ASSERT_LOG(log, fixed_act_check.empty());
    ASSERT_LOG(log, fixed_var_check.empty());
    ASSERT_LOG(log, fixed_t_act_check.empty());
    ASSERT_LOG(log, fixed_t_var_check.empty());
    for (size_t i = 0; i < inst.m; i++) ASSERT_LOG(log, fixed_fadd_check[i].empty());
#endif

    ASSERT_LOG(log, !CPXaddmipstarts(cpxenv, cpxlp, 1, ncols, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));
    delete[] cpx_sol_ind;
    cpx_sol_ind = nullptr;
    delete[] cpx_sol_val;
    cpx_sol_val = nullptr;
}

static void store_dynamic_time_sol(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Storing the dynamic time solution.");

    double* plan{new double[inst.m_opt + inst.m_opt * inst.n_opt]};
    ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt + inst.m_opt * inst.n_opt - 1));

    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (size_t act_i_cpx = 0, fadd_i = inst.m_opt; act_i_cpx < inst.m_opt; act_i_cpx++, fadd_i += inst.n_opt) {
        bool set_zero{true};
        for (size_t var_i_cpx = 0; var_i_cpx < inst.n_opt; var_i_cpx++) {
            if (plan[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
                INTCHECK_ASSERT_LOG(log, plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
                set_zero = false;
                break;
            }
        }
        if (set_zero) plan[act_i_cpx] = 0;
    }

    // convert to std collections for easier parsing
    std::vector<size_t> cpx_result;
    cpx_result.reserve(inst.m_opt);
    for (size_t i = 0; i < inst.m_opt; i++) {
        if (plan[i] > HPLUS_CPX_INT_ROUNDING) cpx_result.push_back(inst.act_cpxtoidx[i]);
    }
    delete[] plan;
    plan = nullptr;

    std::vector<size_t> solution;
    solution.reserve(inst.m_opt);
    binary_set remaining{cpx_result.size(), true}, state{inst.n};

    while (!remaining.empty()) {
        bool intcheck{false};
        for (const auto& i : remaining) {
            if (!state.contains(inst.actions[cpx_result[i]].pre)) continue;

            remaining.remove(i);
            state |= inst.actions[cpx_result[i]].eff;
            solution.push_back(cpx_result[i]);
            intcheck = true;
        }
        ASSERT_LOG(log, intcheck);
    }

    // store solution
    hplus::solution dynamic_t_sol{
        solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [&inst](const unsigned int acc, const size_t index) {
            return acc + inst.actions[index].cost;
        }))};
    hplus::update_sol(inst, dynamic_t_sol, log);
}

// ##################################################################### //
// ######################### ALGORITHM HANDLERS ######################## //
// ##################################################################### //

void run_heur(hplus::instance& inst, hplus::environment& env, const logger& log) {
    srand(time(nullptr));
    if (env.heur == "greedycost")
        greedycost(inst, env, log);
    else if (env.heur == "greedycxe")
        greedycxe(inst, env, log);
    else if (env.heur == "rand")
        randheur(inst, env, log);
    else if (env.heur == "randr")
        randr(inst, env, log);
    else if (env.heur == "hmax")
        hmax(inst, env, log);
    else if (env.heur == "hadd")
        hadd(inst, env, log);
    else if (env.heur == "local-greedycost")
        localsearch(inst, greedycost, env, log);
    else if (env.heur == "local-greedycxe")
        localsearch(inst, greedycxe, env, log);
    else if (env.heur == "local-rand")
        localsearch(inst, randheur, env, log);
    else if (env.heur == "local-randr")
        localsearch(inst, randr, env, log);
    else if (env.heur == "local-hmax")
        localsearch(inst, hmax, env, log);
    else if (env.heur == "local-hadd")
        localsearch(inst, hadd, env, log);
    else
        log.raise_error("The heuristic specified (%s) is not on the list of possible heuristics... Please read the Readme.md for instructions.",
                        env.heur.c_str());
}

void run_model(hplus::instance& inst, hplus::environment& env, hplus::statistics& stats, const logger& log) {
    auto stopchk = [&env]() {
        if (CHECK_STOP()) {
            env.exec_s = exec_status::STOP_TL;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // ~~~~~~~~~~~~ MODEL BUILDING ~~~~~~~~~~~ //
    PRINT_INFO(log, "Building model.");
    env.exec_s = exec_status::MODEL_BUILD;

    stats.build = static_cast<double>(env.time_limit) - env.timer.get_time();
    double start_time = env.timer.get_time();

    CPXENVptr cpxenv = nullptr;
    CPXLPptr cpxlp = nullptr;

    cpx_init(cpxenv, cpxlp, env, log);
    stopchk();

    if (env.alg == HPLUS_CLI_ALG_IMAI)
        cpx_build_imai(cpxenv, cpxlp, inst, env, log);
    else if (env.alg == HPLUS_CLI_ALG_RANKOOH)
        cpx_build_rankooh(cpxenv, cpxlp, inst, env, log);
    else if (env.alg == HPLUS_CLI_ALG_DYNAMIC_TIME)
        cpx_build_dynamic_time(cpxenv, cpxlp, inst, env, log);
    stopchk();

    // time limit
    if (static_cast<double>(env.time_limit) > env.timer.get_time()) {
        ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_TimeLimit, static_cast<double>(env.time_limit) - env.timer.get_time()));
    } else
        throw timelimit_exception("Reached the time limit");

    if (env.warm_start) {  // Post warm starto to CPLEX

        PRINT_INFO(log, "Posting warm start.");

        if (env.alg == HPLUS_CLI_ALG_IMAI)
            cpx_post_warmstart_imai(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_RANKOOH)
            cpx_post_warmstart_rankooh(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_DYNAMIC_TIME)
            cpx_post_warmstart_dynamic_time(cpxenv, cpxlp, inst, env, log);
    }

    cpx_callback_user_handle callback_data{.inst = inst, .env = env, .stats = stats, .log = log};
    if (env.alg == HPLUS_CLI_ALG_DYNAMIC_TIME)
        ASSERT_LOG(log, !CPXcallbacksetfunc(cpxenv, cpxlp, CPX_CALLBACKCONTEXT_CANDIDATE, cpx_dynamic_time_callback, &callback_data));

    stats.build = env.timer.get_time() - start_time;

    // ~~~~~~~~~~~ MODEL EXECUTION ~~~~~~~~~~~ //

    PRINT_INFO(log, "Running CPLEX.");
    env.exec_s = exec_status::CPX_EXEC;

    stats.execution = static_cast<double>(env.time_limit) - env.timer.get_time();
    start_time = env.timer.get_time();

    ASSERT_LOG(log, !CPXmipopt(cpxenv, cpxlp));

    if (parse_cpx_status(cpxenv, cpxlp, env, log)) {  // If CPLEX has found a solution
        if (env.alg == HPLUS_CLI_ALG_IMAI)
            store_imai_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_RANKOOH)
            store_rankooh_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_DYNAMIC_TIME)
            store_dynamic_time_sol(cpxenv, cpxlp, inst, log);
    }

    cpx_close(cpxenv, cpxlp);

    stats.execution = env.timer.get_time() - start_time;
}
