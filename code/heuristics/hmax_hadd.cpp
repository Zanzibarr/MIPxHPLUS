#include "hmax_hadd.hpp"

#include <list>

#if HPLUS_INTCHECK == 0
#define INTCHECK_PQ false
#endif
#include "pq.hxx"

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

            const double cost_pre{evaluate_htype_state(inst.actions[act_i].pre, values, h_eqtype)};
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
    const auto find_best_act = [&inst, &h_eqtype, &used_actions, &trail, &pq, &timestamp](const std::list<size_t>& candidates,
                                                                                          std::vector<double>& values, const binary_set& state) {
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

void hmax::run(hplus::instance& inst, hplus::environment& env, const logger& log) {
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

void hadd::run(hplus::instance& inst, hplus::environment& env, const logger& log) {
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