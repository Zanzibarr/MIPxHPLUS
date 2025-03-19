/**
 * @file hplus_instance.cpp
 * @brief Methods implementation of the hplus_instance.hpp interface
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include "hplus_instance.hpp"

#include <algorithm>  // For std::set_difference
#include <fstream>    // For std::ifstream

void hplus::print_stats(const statistics& stats, const logger& log) {
#if HPLUS_VERBOSE < 5
    return;
#endif
    log.print("\n\n--------------------------------------------------");
    log.print("-----------------   Statistics   -----------------");
    log.print("--------------------------------------------------\n");
    log.print(" >>  Parsing time                  %10.3fs  <<", stats.parsing);
    log.print(" >>  Problem simplification time   %10.3fs  <<", stats.optimization);
    log.print(" >>  Heuristic time                %10.3fs  <<", stats.heuristic);
    log.print(" >>  Model building time           %10.3fs  <<", stats.build);
    log.print(" >>  CPLEX execution time          %10.3fs  <<", stats.execution);
    log.print(" >>  CPLEX callback time           %10.3fs  <<", stats.callback);
    log.print(" >>  Total time                    %10.3fs  <<", stats.total);
    log.print("\n\n");
}

static void init(hplus::instance& inst) {
    inst = hplus::instance{.equal_costs = false,
                           .n = 0,
                           .m = 0,
                           .n_opt = 0,
                           .m_opt = 0,
                           .var_ranges = std::vector<int>(0),
                           .actions = std::vector<hplus::action>(0),
                           .goal = binary_set(),
                           .best_sol = hplus::solution{std::vector<size_t>(), std::numeric_limits<unsigned int>::max()},
                           .var_e = binary_set(),
                           .var_f = binary_set(),
                           .act_e = binary_set(),
                           .act_f = binary_set(),
                           .fadd_e = std::vector<binary_set>(0),
                           .fadd_f = std::vector<binary_set>(0),
                           .var_t = std::vector<int>(0),
                           .act_t = std::vector<int>(0),
                           .act_inv = std::vector<std::vector<size_t>>(0),
                           .var_rem = std::vector<size_t>(0),
                           .act_rem = std::vector<size_t>(0),
                           .var_opt_conv = std::vector<size_t>(0),
                           .act_opt_conv = std::vector<size_t>(0),
                           .act_cpxtoidx = std::vector<size_t>(0),
                           .act_with_eff = std::vector<std::vector<size_t>>(0),
                           .act_with_pre = std::vector<std::vector<size_t>>(0)};
}

[[nodiscard]]
static bool parse_inst_file(hplus::instance& inst, hplus::environment& env, hplus::statistics& stats, const logger& log) {
    // ====================================================== //
    // ================== PARSING SAS FILE ================== //
    // ====================================================== //

    PRINT_INFO(log, "Parsing SAS file.");

    std::ifstream ifs(env.input_file.c_str(), std::ifstream::in);
    ASSERT_LOG(log, ifs.good());

    stats.parsing = static_cast<double>(env.time_limit) - env.timer.get_time();
    std::string line;

    // * version section
    std::getline(ifs, line);  // begin_version
    if (line != "begin_version") [[unlikely]]
        log.raise_error("Corrupted file");
    std::getline(ifs, line);  // version_number (ignored)
    if (!isint(line)) [[unlikely]]
        log.raise_error("Corrupted file");
    std::getline(ifs, line);  // end_version
    if (line != "end_version") [[unlikely]]
        log.raise_error("Corrupted file");

    // * metric section
    std::getline(ifs, line);  // begin_metric
    if (line != "begin_metric") [[unlikely]]
        log.raise_error("Corrupted file");
    std::getline(ifs, line);  // metric
    if (!isint(line, 0, 1)) [[unlikely]]
        log.raise_error("Corrupted file");
    inst.equal_costs = stoi(line) == 0;
    std::getline(ifs, line);  // end_metric
    if (line != "end_metric") [[unlikely]]
        log.raise_error("Corrupted file");

    // * variables section
    PRINT_WARN(log, "Ignoring axiom layers.");
    std::getline(ifs, line);  // inst.n
    if (!isint(line, 0)) [[unlikely]]
        log.raise_error("Corrupted file");
    inst.n = stoi(line);
    inst.var_ranges = std::vector<int>(inst.n);
    for (size_t var_i = 0; var_i < inst.n; var_i++) {
        // process each variable
        std::getline(ifs, line);  // begin_variable
        if (line != "begin_variable") [[unlikely]]
            log.raise_error("Corrupted file");
        std::getline(ifs, line);  // variable name (ignored)
        std::getline(ifs, line);  // axiom layer (ignored)
        if (line != "-1") [[unlikely]]
            log.raise_error("Axiom layer is not -1, this software is not made for this instance.");
        std::getline(ifs, line);  // range of variable
        if (!isint(line, 0)) [[unlikely]]
            log.raise_error("Corrupted file");
        const int range{stoi(line)};
        inst.var_ranges[var_i] = range;
        for (int j = 0; j < range; j++) std::getline(ifs, line);  // name for variable value (ignored)
        std::getline(ifs, line);                                  // end_variable
        if (line != "end_variable") [[unlikely]]
            log.raise_error("Corrupted file");
    }

    // * mutex section (ignored)
    PRINT_WARN(log, "Ignoring mutex section.");
    std::getline(ifs, line);  // number of mutex groups
    if (!isint(line, 0)) [[unlikely]]
        log.raise_error("Corrupted file");
    const size_t nmgroups{static_cast<size_t>(stoi(line))};
    for (size_t i = 0; i < nmgroups; i++) {
        std::getline(ifs, line);  // begin_mutex_group
        if (line != "begin_mutex_group") [[unlikely]]
            log.raise_error("Corrupted file");
        while (line != "end_mutex_group") {
            std::getline(ifs,
                         line);  // reach end_mutex_group (ignore all content)
            if (line == "begin_state") [[unlikely]]
                log.raise_error("Corrupted file");
        }
    }

    // * initial state section
    std::getline(ifs, line);  // begin_state
    if (line != "begin_state") [[unlikely]]
        log.raise_error("Corrupted file");
    std::vector<int> tmp_istate(inst.n);
    for (size_t var_i = 0; var_i < inst.n; var_i++) {
        std::getline(ifs, line);  // initial value of var_i
        if (!isint(line, 0, inst.var_ranges[var_i] - 1)) [[unlikely]]
            log.raise_error("Corrupted file");
        const int val{stoi(line)};
        tmp_istate[var_i] = val;
    }
    std::getline(ifs, line);  // end_state
    if (line != "end_state") [[unlikely]]
        log.raise_error("Corrupted file");

    // * goal state section
    std::getline(ifs, line);  // begin_goal
    if (line != "begin_goal") [[unlikely]]
        log.raise_error("Corrupted file");
    std::vector<int> tmp_goal(inst.n, -1);
    std::getline(ifs, line);  // number of goals
    if (!isint(line, 0, static_cast<int>(inst.n))) [[unlikely]]
        log.raise_error("Corrupted file");
    const size_t ngoals{static_cast<size_t>(stoi(line))};
    for (size_t _ = 0; _ < ngoals; _++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(ifs, line);  // pair 'variable goal'
        tokens = split_string(line, ' ');
        if (tokens.size() != 2) [[unlikely]]
            log.raise_error("Corrupted file");
        if (!isint(tokens[0], 0, static_cast<int>(inst.n) - 1)) [[unlikely]]
            log.raise_error("Corrupted file");  // variable index
        const size_t var{static_cast<size_t>(stoi(tokens[0]))};
        if (!isint(tokens[1], 0, inst.var_ranges[var] - 1)) [[unlikely]]
            log.raise_error("Corrupted file");  // variable goal
        const int value{stoi(tokens[1])};
        tmp_goal[var] = value;
    }
    std::getline(ifs, line);  // end_goal
    if (line != "end_goal") [[unlikely]]
        log.raise_error("Corrupted file");

    // * operator (actions) section
    int checkcosts{-1};
    bool equalcosts_check{true};
    PRINT_WARN(log, "Ignoring effect conditions.");
    std::getline(ifs, line);  // n_act
    if (!isint(line, 0)) [[unlikely]]
        log.raise_error("Corrupted file");
    inst.m = stoi(line);
    inst.actions = std::vector<hplus::action>(inst.m);
    std::vector<std::vector<std::pair<size_t, size_t>>> tmp_act_pre(inst.m), tmp_act_eff(inst.m);
    for (size_t act_i = 0; act_i < inst.m; act_i++) {
        // process each action
        std::getline(ifs, line);  // begin_operator
        if (line != "begin_operator") [[unlikely]]
            log.raise_error("Corrupted file");
        std::getline(ifs, line);  // symbolic action name
        std::string name{line};
        std::vector<int> act_pre(inst.n, -1);
        std::getline(ifs, line);  // number of prevail conditions
        if (!isint(line, 0, static_cast<int>(inst.n))) [[unlikely]]
            log.raise_error("Corrupted file");
        const size_t n_pre{static_cast<size_t>(stoi(line))};
        for (size_t pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(ifs, line);  // pair 'variable value'
            tokens = split_string(line, ' ');
            if (tokens.size() != 2) [[unlikely]]
                log.raise_error("Corrupted file");
            if (!isint(tokens[0], 0, static_cast<int>(inst.n) - 1)) [[unlikely]]
                log.raise_error("Corrupted file");  // variable index
            const size_t var{static_cast<size_t>(stoi(tokens[0]))};
            if (!isint(tokens[1], 0, inst.var_ranges[var] - 1)) [[unlikely]]
                log.raise_error("Corrupted file");  // variable value
            const int value{stoi(tokens[1])};
            act_pre[var] = value;
        }
        std::getline(ifs, line);  // number of effects
        if (!isint(line, 0)) [[unlikely]]
            log.raise_error("Corrupted file");
        const size_t n_eff{static_cast<size_t>(stoi(line))};
        std::vector<int> act_eff(inst.n, -1);
        for (size_t eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(ifs, line);  // effect line
            std::vector<std::string> tokens;
            tokens = split_string(line, ' ');
            if (tokens.size() != 4) [[unlikely]]
                log.raise_error("This program won't handle effect conditions.");  // not expecting effect conditions
            if (!isint(tokens[0], 0, 0)) [[unlikely]]
                log.raise_error("This program won't handle effect conditions.");  // number of effect conditions (ignored and check to be 0)
            if (!isint(tokens[1], 0, static_cast<int>(inst.n) - 1)) [[unlikely]]
                log.raise_error("Corrupted file");  // variable affected by the action
            const size_t var{static_cast<size_t>(stoi(tokens[1]))};
            if (!isint(tokens[2], -1, inst.var_ranges[var] - 1)) [[unlikely]]
                log.raise_error("Corrupted file");  // precondition of the variable
            const int pre_val{stoi(tokens[2])};
            if (!isint(tokens[3], 0, inst.var_ranges[var] - 1)) [[unlikely]]
                log.raise_error("Corrupted file");  // effect of the variable
            const int eff_val{stoi(tokens[3])};
            if (pre_val >= 0) act_pre[var] = pre_val;
            act_eff[var] = eff_val;
        }
        std::getline(ifs, line);  // action cost
        if (!isint(line, 0)) [[unlikely]]
            log.raise_error("Corrupted file");
        unsigned int cost{1};
        if (!inst.equal_costs) {
            cost = stoi(line);
            if (checkcosts == -1)
                checkcosts = cost;
            else if (static_cast<unsigned int>(checkcosts) != cost)
                equalcosts_check = false;
        }
        std::getline(ifs, line);  // end_operator
        if (line != "end_operator") [[unlikely]]
            log.raise_error("Corrupted file");
        inst.actions[act_i] = hplus::action{.pre = binary_set(),
                                            .eff = binary_set(),
                                            .pre_sparse = std::vector<size_t>(),
                                            .eff_sparse = std::vector<size_t>(),
                                            .cost = cost,
                                            .name = name};

        for (size_t i = 0; i < inst.n; i++) {
            if (act_pre[i] >= 0) tmp_act_pre[act_i].emplace_back(i, act_pre[i]);
            if (act_eff[i] >= 0) tmp_act_eff[act_i].emplace_back(i, act_eff[i]);
        }
    }
    inst.equal_costs = equalcosts_check;

    PRINT_WARN(log, "Ignoring axiom section.");

    ifs.close();

    if (inst.m == 0) {
        for (size_t i = 0; i < inst.n; i++) {
            if (tmp_istate[i] == tmp_goal[i] || tmp_goal[i] < 0) continue;
            stats.parsing = env.timer.get_time();
            return false;  // no actions and goal isn't the starting point ->
                           // infeasible
        }
    }

    // ====================================================== //
    // ================== BINARY EXPANSION ================== //
    // ====================================================== //

    std::vector<int> var_active_state(inst.n, -1);

    const auto is_deletefree = [&inst, &tmp_act_eff, &var_active_state]() {
        for (const auto& r : inst.var_ranges) {
            if (r != 2) return false;
        }
        for (const auto& act_eff : tmp_act_eff) {
            for (const auto& [var, val] : act_eff) {
                if (var_active_state[var] == -1)
                    var_active_state[var] = val;
                else if (var_active_state[var] != static_cast<int>(val))
                    return false;
            }
        }
        return true;
    };
    binary_set istate;
    if (is_deletefree()) {
        PRINT_DEBUG(log, "Detected delete-free instance, skipping binary expansion.");
        istate = binary_set(inst.n);
        inst.goal = binary_set(inst.n);
        for (size_t var_i = 0; var_i < inst.n; var_i++) {
            if (tmp_istate[var_i] == var_active_state[var_i]) istate.add(var_i);
            if (tmp_goal[var_i] >= 0 && tmp_goal[var_i] == var_active_state[var_i]) inst.goal.add(var_i);
        }
        for (size_t act_i = 0; act_i < inst.m; act_i++) {
            binary_set act_pre{inst.n}, act_eff{inst.n};
            for (const auto& [var, val] : tmp_act_pre[act_i]) {
                if (var_active_state[var] == static_cast<int>(val)) act_pre.add(var);
            }
            for (const auto& [var, val] : tmp_act_eff[act_i]) act_eff.add(var);
            inst.actions[act_i].pre = act_pre;
            inst.actions[act_i].pre_sparse = act_pre.sparse();
            inst.actions[act_i].eff = act_eff;
            inst.actions[act_i].eff_sparse = act_eff.sparse();
        }
    } else {
        // binary expansion
        PRINT_VERBOSE(log, "Performing binary expansion.");
        size_t n_exp{0};
        std::vector<size_t> offsets(inst.n);
        for (size_t i = 0; i < inst.n; i++) {
            offsets[i] = n_exp;
            n_exp += inst.var_ranges[i];
        }
        istate = binary_set(n_exp);
        inst.goal = binary_set(n_exp);
        for (size_t i = 0; i < inst.n; i++) {
            istate.add(offsets[i] + tmp_istate[i]);
            if (tmp_goal[i] >= 0) inst.goal.add(offsets[i] + tmp_goal[i]);
        }
        inst.n = n_exp;
        for (size_t i = 0; i < inst.m; i++) {
            binary_set act_pre{inst.n}, act_eff{inst.n};
            for (const auto& [var, val] : tmp_act_pre[i]) act_pre.add(offsets[var] + val);
            for (const auto& [var, val] : tmp_act_eff[i]) act_eff.add(offsets[var] + val);
            inst.actions[i].pre = act_pre;
            inst.actions[i].pre_sparse = act_pre.sparse();
            inst.actions[i].eff = act_eff;
            inst.actions[i].eff_sparse = act_eff.sparse();
        }
    }

    // ====================================================== //
    // ================ INITIAL STATE REMOVAL =============== //
    // ====================================================== //

    PRINT_VERBOSE(log, "Removing initial state variables.");
    std::vector<size_t> istate_offsets(inst.n);
    size_t n_opt{inst.n};
    for (size_t i = 0, c = 0; i < inst.n; i++) {
        if (istate[i]) {
            c++;
            n_opt--;
        }
        istate_offsets[i] = c;
    }
    inst.n = n_opt;
    binary_set goal_opt{inst.n};
    for (const auto& var : inst.goal) {
        if (!istate[var]) goal_opt.add(var - istate_offsets[var]);
    }
    inst.goal = goal_opt;
    for (size_t i = 0; i < inst.m; i++) {
        binary_set act_pre{inst.n}, act_eff{inst.n};
        for (const auto& var : inst.actions[i].pre_sparse) {
            if (!istate[var]) act_pre.add(var - istate_offsets[var]);
        }
        for (const auto& var : inst.actions[i].eff_sparse) {
            if (!istate[var]) act_eff.add(var - istate_offsets[var]);
        }
        inst.actions[i].pre = act_pre;
        inst.actions[i].pre_sparse = act_pre.sparse();
        inst.actions[i].eff = act_eff;
        inst.actions[i].eff_sparse = act_eff.sparse();
    }

    stats.parsing = env.timer.get_time();

    // ====================================================== //
    // ================ OPTIMIZE ENVIRONMENT ================ //
    // ====================================================== //
    if (inst.equal_costs && env.heur == "greedycost") {
        env.heur = "greedycxe";
        PRINT_WARN(log, "Detected instance with equal costs actions, switching to 'greedycxe' heuristic.");
    }

    // return
    const bool is_infeasible = (  // add here other fast feasibility checks
        inst.m == 0 && !inst.goal.empty());
    return !is_infeasible;
}

static void init_instance_opt(hplus::instance& inst) {
    inst.n_opt = inst.n;
    inst.m_opt = inst.m;
    inst.var_e = binary_set(inst.n);
    inst.var_f = binary_set(inst.n);
    inst.act_e = binary_set(inst.m);
    inst.act_f = binary_set(inst.m);
    inst.fadd_e = std::vector<binary_set>(inst.m, binary_set(inst.n));
    inst.fadd_f = std::vector<binary_set>(inst.m, binary_set(inst.n));
    inst.var_t = std::vector<int>(inst.n, -1);
    inst.act_t = std::vector<int>(inst.m, -1);
    inst.act_inv = std::vector<std::vector<size_t>>(inst.m, std::vector<size_t>());
    inst.var_rem = (!inst.var_e).sparse();
    inst.act_rem = (!inst.act_e).sparse();
    inst.var_opt_conv = std::vector<size_t>(inst.n);
    for (size_t idx = 0; idx < inst.n; idx++) inst.var_opt_conv[idx] = idx;
    inst.act_opt_conv = std::vector<size_t>(inst.m);
    for (size_t idx = 0; idx < inst.m; idx++) inst.act_opt_conv[idx] = idx;
    inst.act_cpxtoidx = std::vector<size_t>(inst.m);
    for (size_t idx = 0; idx < inst.m; idx++) inst.act_cpxtoidx[idx] = idx;
}

bool hplus::create_instance(instance& inst, environment& env, statistics& stats, const logger& log) {
    init(inst);
    if (!parse_inst_file(inst, env, stats, log)) {
        PRINT_VERBOSE(log, "The problem is infeasible.");
        env.sol_s = solution_status::INFEAS;
        return false;
    }
    init_instance_opt(inst);

    PRINT_VERBOSE(log, "Created HPLUS_instance.");
    return true;
}

void hplus::update_sol(instance& inst, const solution& sol, const logger& log) {
    const auto& [sol_plan, sol_cost]{sol};
    binary_set dbcheck{inst.m};
    unsigned int costcheck{0};
    ASSERT_LOG(log,
               sol_plan.size() <= inst.m_opt);  // check that there aren't more
                                                // actions that there exists
    binary_set feas_checker{inst.n};
    for (const auto& act_i : sol_plan) {
        ASSERT_LOG(log, act_i < inst.m);   // check that the solution only
                                           // contains existing actions
        ASSERT_LOG(log, !dbcheck[act_i]);  // check that there are no duplicates
        ASSERT_LOG(log,
                   !inst.act_e[act_i])  // check that the action was not eliminated
        dbcheck.add(act_i);
        ASSERT_LOG(log,
                   feas_checker.contains(inst.actions[act_i].pre));  // check if the preconditions
                                                                     // are respected at each step
        feas_checker |= inst.actions[act_i].eff;
        costcheck += inst.actions[act_i].cost;
    }
    ASSERT_LOG(log, feas_checker.contains(inst.goal));  // check if the solution leads to the goal state
    ASSERT_LOG(log,
               costcheck == sol_cost);  // check if the cost is the declared one

    if (sol_cost >= inst.best_sol.cost) return;

    inst.best_sol = solution{sol_plan, sol_cost};

#if HPLUS_VERBOSE >= 10
    log.print_info("Updated best solution - Cost: %10u.", sol_cost);
#endif
}

void hplus::print_sol(const instance& inst, const logger& log) {
    const auto& [sol_plan, sol_cost]{inst.best_sol};
    log.print("Solution cost: %10u", sol_cost);
    // for (const auto& act_i : sol_plan)
    // 	log.print("(%s)", inst.actions[act_i].name.c_str());
}

static void landmark_extraction(hplus::instance& inst, std::vector<binary_set>& landmarks, binary_set& fact_landmarks, binary_set& act_landmarks,
                                const logger& log) {
    PRINT_VERBOSE(log, "Extracting landmarks.");

    for (size_t var_i = 0; var_i < inst.n; var_i++) landmarks[var_i].add(inst.n);

    binary_set s_set{inst.n};

    std::deque<size_t> actions_queue;
    for (size_t act_i = 0; act_i < inst.m; act_i++) {
        if (s_set.contains(inst.actions[act_i].pre)) actions_queue.push_back(act_i);
    }

    // list of actions that have as precondition variable p
    std::vector<std::vector<size_t>> act_with_pre(inst.n);
    for (size_t var_i = 0; var_i < inst.n; var_i++) {
        act_with_pre[var_i].reserve(inst.m);
        for (size_t act_i = 0; act_i < inst.m; act_i++) {
            if (inst.actions[act_i].pre[var_i]) act_with_pre[var_i].push_back(act_i);
        }
    }

    while (!actions_queue.empty()) {
        const hplus::action& a{inst.actions[actions_queue.front()]};
        actions_queue.pop_front();

        const auto& pre_sparse{a.pre_sparse};
        const auto& add_sparse{a.eff_sparse};

        for (const auto& var_i : add_sparse) {
            s_set.add(var_i);

            binary_set x{inst.n + 1};
            for (const auto& var_j : add_sparse) x.add(var_j);

            for (const auto& var_j : pre_sparse) {
                // if variable var_i' has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set
                // the flag
                if (landmarks[var_j][inst.n]) {
                    x.add(inst.n);
                    // if x is now full we can exit, since all further unions
                    // won't change x
                    break;
                } else
                    x |= landmarks[var_j];
            }

            // we then check if L[var_i] != X, and if they are the same we skip,
            // if X = P, then (X intersection L[P]) = L[P], hence we can already
            // skip
            if (x[inst.n]) continue;

            // if the set for variable var_i is the full set of variables,
            // the intersection generates back x -> we can skip the intersection
            if (!landmarks[var_i][inst.n]) x &= landmarks[var_i];

            // we already know that x is not the full set now, so if
            // the set for variable var_i is the full set, we know that x is not
            // equal to the set for variable var_i -> we can skip the check
            if (!landmarks[var_i][inst.n] && x == landmarks[var_i]) continue;

            landmarks[var_i] = x;
            for (const auto& act_i : act_with_pre[var_i]) {
                if (s_set.contains(inst.actions[act_i].pre) && std::find(actions_queue.begin(), actions_queue.end(), act_i) == actions_queue.end())
                    actions_queue.push_back(act_i);
            }
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

    // list of actions that have as effect variable p
    std::vector<std::vector<size_t>> act_with_eff(inst.n);
    for (const auto& var_i : !fact_landmarks) {
        act_with_eff[var_i].reserve(inst.m);
        for (size_t act_i = 0; act_i < inst.m; act_i++) {
            if (inst.actions[act_i].eff[var_i]) act_with_eff[var_i].push_back(act_i);
        }
    }

    // popolate fact_landmarks and act_landmarks sets
    for (const auto& var_i : inst.goal) {
        for (const auto& var_j : !fact_landmarks) {
            if (!landmarks[var_i][var_j] && !landmarks[var_i][inst.n]) continue;

            fact_landmarks.add(var_j);
            size_t count{0}, cand_act;
            for (const auto& act_i : act_with_eff[var_j]) {
                cand_act = act_i;
                count++;
                if (count > 1) break;
            }
            if (count == 1) act_landmarks.add(cand_act);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
    inst.var_f |= fact_landmarks;
    inst.act_f |= act_landmarks;
}

static void fadd_extraction(hplus::instance& inst, const std::vector<binary_set>& landmarks, std::vector<binary_set>& first_adders,
                            const logger& log) {
    PRINT_VERBOSE(log, "Extracting first adders.");

    // list of fact landmarks for each variable
    std::vector<std::vector<size_t>> var_flm_sparse(inst.n);
    for (size_t var_i = 0; var_i < inst.n; var_i++) {
        var_flm_sparse[var_i].reserve(inst.n);
        for (size_t var_j = 0; var_j < inst.n; var_j++) {
            if (landmarks[var_i][var_j] || landmarks[var_i][inst.n]) var_flm_sparse[var_i].push_back(var_j);
        }
    }

    // compute the set of first adders
    for (size_t act_i = 0; act_i < inst.m; act_i++) {
        // f_lm_a is the set of fact landmarks of action act_i
        binary_set f_lm_a{inst.n};
        for (const auto& var_i : inst.actions[act_i].pre) {
            for (const auto& i : var_flm_sparse[var_i]) f_lm_a.add(i);
        }
        // first_adders[a] := { p in add(a) s.t. p is not a fact landmark for a
        // }
        first_adders[act_i] |= (inst.actions[act_i].eff & !f_lm_a);
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
    for (size_t act_i = 0; act_i < inst.m; act_i++) inst.fadd_e[act_i] |= (inst.actions[act_i].eff & !first_adders[act_i]);
}

static void relevance_analysis(hplus::instance& inst, const binary_set& fact_landmarks, const std::vector<binary_set>& first_adders,
                               const logger& log) {
    PRINT_VERBOSE(log, "Relevance analysis.");

    binary_set relevant_variables{inst.n};
    binary_set relevant_actions{inst.m};

    // compute first round of relevand variables and actions
    for (size_t act_i = 0; act_i < inst.m; act_i++) {
        if (first_adders[act_i].intersects(inst.goal)) {
            relevant_variables |= inst.actions[act_i].pre;
            relevant_actions.add(act_i);
        }
    }
    if (CHECK_STOP()) [[unlikely]]
        throw timelimit_exception("Reached time limit.");

    // list of actions yet to check
    auto cand_actions_sparse{(!relevant_actions).sparse()};

    // keep looking for other relevant actions/variables until no more can be
    // found
    bool new_act{true};
    while (new_act) {
        new_act = false;
        std::vector<size_t> new_relevant_actions;
        new_relevant_actions.reserve(inst.m);
        for (const auto& act_i : cand_actions_sparse) {
            if (!inst.actions[act_i].eff.intersects(relevant_variables)) continue;

            relevant_actions.add(act_i);
            relevant_variables |= inst.actions[act_i].pre;
            new_relevant_actions.push_back(act_i);
            new_act = true;
        }
        const auto it{std::set_difference(cand_actions_sparse.begin(), cand_actions_sparse.end(), new_relevant_actions.begin(),
                                          new_relevant_actions.end(), cand_actions_sparse.begin())};
        cand_actions_sparse.resize(it - cand_actions_sparse.begin());
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
    relevant_variables |= inst.goal;

    // eliminate actions and variables that are not relevant (or landmarks)
    inst.var_e |= (!relevant_variables - fact_landmarks);
    inst.act_e |= !relevant_actions;
}

static void dominated_actions_elimination(hplus::instance& inst, const std::vector<binary_set>& landmarks,
                                          const std::vector<binary_set>& first_adders, const logger& log) {
    PRINT_VERBOSE(log, "Extracting dominated actions.");

    const auto& rem_var{(!inst.var_e).sparse()};
    std::vector<binary_set> act_flm(inst.m, binary_set(inst.n));
    std::vector<std::vector<size_t>> var_flm_sparse(inst.n);

    // compute the landmarks for each variable remaining
    for (const auto& var_i : rem_var) {
        var_flm_sparse[var_i].reserve(inst.n);
        for (const auto& var_j : rem_var) {
            if (landmarks[var_i][var_j] || landmarks[var_i][inst.n]) var_flm_sparse[var_i].push_back(var_j);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

    // compute the landmarks for each action remaining
    for (size_t act_i = 0; act_i < inst.m; act_i++) {
        for (const auto& var_i : inst.actions[act_i].pre_sparse) {
            for (const auto& i : var_flm_sparse[var_i]) act_flm[act_i].add(i);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }

    // find efficently all actions that satisfy point 1) of Proposition 4 of in Imai's Paper
    const auto& rem_act{(!inst.act_e).sparse()};
    bs_searcher candidates{inst.n};
    for (const auto& act_i : rem_act) {
        if (inst.act_f[act_i]) continue;
        candidates.add(act_i, first_adders[act_i]);
    }

    binary_set dominated_actions{inst.m};

    // find all dominated actions and eliminate them
    for (const auto& dominant_act : rem_act) {
        if (dominated_actions[dominant_act]) continue;

        for (const auto& dominated_act : candidates.find_subsets(first_adders[dominant_act])) {
            if (dominant_act == dominated_act || inst.actions[dominant_act].cost > inst.actions[dominated_act].cost ||
                !act_flm[dominated_act].contains(inst.actions[dominant_act].pre)) [[likely]]
                continue;

            dominated_actions.add(dominated_act);
            inst.act_e.add(dominated_act);
            candidates.remove(dominated_act, first_adders[dominated_act]);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
}

static void immediate_action_application(hplus::instance& inst, const binary_set& act_landmarks, const logger& log) {
    PRINT_VERBOSE(log, "Immediate action application.");

    binary_set current_state{inst.n}, used_actions{inst.m};
    const auto& rem_act{(!inst.act_e).sparse()};

    // find efficiently all actions that satisfy point 2) of the Definition 1 in
    // section 4.6 of Imai's paper
    bs_searcher subset_finder{inst.n};
    for (const auto& act_i : rem_act) subset_finder.add(act_i, inst.actions[act_i].pre - inst.var_e);

    // keep looking until no more actions can be applied
    unsigned int counter{0};
    bool found_next_action{true};
    while (found_next_action) {
        found_next_action = false;

        for (const auto& act_i : subset_finder.find_subsets(current_state)) {
            if (used_actions[act_i]) continue;

            const auto& pre{inst.actions[act_i].pre - inst.var_e};
            const auto& eff{inst.actions[act_i].eff - inst.var_e};

            if (!act_landmarks[act_i] && inst.actions[act_i].cost != 0) [[likely]] {
                subset_finder.remove(act_i, pre);
                continue;
            }

            used_actions.add(act_i);
            inst.act_f.add(act_i);
            inst.act_t[act_i] = counter;
            counter++;
            for (const auto& var_i : eff) {
                if (current_state[var_i]) continue;

                inst.var_t[var_i] = counter;
                inst.fadd_f[act_i].add(var_i);
                for (const auto& act_j : rem_act) {
                    if (act_i == act_j) continue;
                    inst.fadd_e[act_j].add(var_i);
                }
            }
            inst.var_f |= eff;
            current_state |= eff;
            subset_finder.remove(act_i, pre);
            found_next_action = true;
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
}

static void inverse_actions_extraction(hplus::instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Extracting inverse actions.");

    bs_searcher subset_finder{inst.n};

    // find efficiently all actions that satisfy point 2) of the Definition 1 in
    // section 4.6 of Imai's paper
    const auto& rem_act{(!inst.act_e).sparse()};
    for (const auto& act_i : rem_act) subset_finder.add(act_i, inst.actions[act_i].eff);

    for (const auto& act_i : rem_act) {
        const auto& pre{inst.actions[act_i].pre};
        const auto& eff{inst.actions[act_i].eff};
        for (const auto& act_j : subset_finder.find_subsets(pre)) {
            if (!inst.actions[act_j].pre.contains(eff)) [[likely]]
                continue;

            if (!inst.act_f[act_i]) inst.act_inv[act_i].push_back(act_j);
            if (!inst.act_f[act_j]) inst.act_inv[act_j].push_back(act_i);
        }
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    }
}

static void finish_opt(hplus::instance& inst, const logger& log) {
    inst.var_rem = (!inst.var_e).sparse();
    inst.act_rem = (!inst.act_e).sparse();
    size_t count{0};
    for (const auto& var_i : inst.var_rem) inst.var_opt_conv[var_i] = count++;
    inst.n_opt = count;
    count = 0;
    for (const auto& act_i : inst.act_rem) {
        inst.act_opt_conv[act_i] = count;
        inst.act_cpxtoidx[count++] = act_i;
    }
    inst.m_opt = count;
    for (size_t act_i = 0; act_i < inst.m; act_i++) {
        inst.actions[act_i].pre -= inst.var_e;
        inst.actions[act_i].eff -= inst.var_e;
        inst.actions[act_i].pre_sparse = inst.actions[act_i].pre.sparse();
        inst.actions[act_i].eff_sparse = inst.actions[act_i].eff.sparse();
    }
    INTCHECK_ASSERT_LOG(log, !inst.var_f.intersects(inst.var_e));
    INTCHECK_ASSERT_LOG(log, !inst.act_f.intersects(inst.act_e));
    for (size_t i = 0; i < inst.m; i++) {
        INTCHECK_ASSERT_LOG(log, !inst.fadd_f[i].intersects(inst.fadd_e[i]));
        INTCHECK_ASSERT_LOG(log, !(inst.act_e[i] && inst.act_t[i] >= 0));
    }
    for (size_t i = 0; i < inst.n; i++) INTCHECK_ASSERT_LOG(log, !(inst.var_e[i] && inst.var_t[i] >= 0));
}

void hplus::instance_optimization(instance& inst, const logger& log) {
    std::vector<binary_set> landmarks(inst.n, binary_set(inst.n + 1));
    binary_set fact_landmarks{inst.n};
    binary_set act_landmarks{inst.m};
    std::vector<binary_set> fadd(inst.m, binary_set(inst.n));
    landmark_extraction(inst, landmarks, fact_landmarks, act_landmarks, log);
    fadd_extraction(inst, landmarks, fadd, log);
    relevance_analysis(inst, fact_landmarks, fadd, log);
    dominated_actions_elimination(inst, landmarks, fadd, log);
    immediate_action_application(inst, act_landmarks, log);
    inverse_actions_extraction(inst, log);
    finish_opt(inst, log);
#if HPLUS_VERBOSE >= 100
    size_t count = 0;
    for (const auto& x [[maybe_unused]] : inst.var_f | inst.var_e) count++;
    log.print_info("# variables: %d - %d = %d.", inst.n, count, inst.n - count);
    count = 0;
    for (const auto& x [[maybe_unused]] : inst.act_f | inst.act_e) count++;
    log.print_info("# actions: %d - %d = %d.", inst.m, count, inst.m - count);
#endif
}

void hplus::prepare_faster_actsearch(instance& inst, const logger& log) {
    PRINT_VERBOSE(log, "Initializing data structures for faster actions lookup.");
    inst.act_with_pre = std::vector<std::vector<size_t>>(inst.n);
    inst.act_with_eff = std::vector<std::vector<size_t>>(inst.n);
    for (const auto& var_i : inst.var_rem) {
        inst.act_with_pre[var_i].reserve(inst.m_opt);
        inst.act_with_eff[var_i].reserve(inst.m_opt);
        for (const auto& act_i : inst.act_rem) {
            if (inst.actions[act_i].pre[var_i]) inst.act_with_pre[var_i].push_back(act_i);
            if (inst.actions[act_i].eff[var_i]) inst.act_with_eff[var_i].push_back(act_i);
        }
    }
}