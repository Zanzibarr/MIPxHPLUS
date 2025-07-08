#include <new>

#include "../preprocessing/preprocessing.hpp"
#include "../search_exact/exact.hpp"
#include "../search_heuristics/heuristic.hpp"

void hplus::read_file(execution& exec, instance& inst, statistics& stats) {
    if (BASIC_VERBOSE()) LOG_INFO << "Parsing input file";

    std::ifstream file(exec.file.c_str(), std::ifstream::in);
    if (!file.good()) LOG_ERROR << "Unable to open file " << exec.file;

    stats.parsing = static_cast<double>(exec.timelimit) - GET_TIME();
    exec.exec_s = exec_status::INSTANCE_BUILDING;
    std::string line;

    // * version section
    std::getline(file, line);  // begin_version
    if (line != "begin_version") [[unlikely]]
        LOG_ERROR << "Corrupted file";
    std::getline(file, line);  // version_number (ignored)
    if (!isint(line)) [[unlikely]]
        LOG_ERROR << "Corrupted file";
    std::getline(file, line);  // end_version
    if (line != "end_version") [[unlikely]]
        LOG_ERROR << "Corrupted file";

    // * metric section
    std::getline(file, line);  // begin_metric
    if (line != "begin_metric") [[unlikely]]
        LOG_ERROR << "Corrupted file";
    std::getline(file, line);  // metric
    if (!isint(line, 0, 1)) [[unlikely]]
        LOG_ERROR << "Corrupted file";
    inst.equal_costs = stoi(line) == 0;
    std::getline(file, line);  // end_metric
    if (line != "end_metric") [[unlikely]]
        LOG_ERROR << "Corrupted file";

    // * variables section
    if (BASIC_VERBOSE()) LOG_WARNING << "Ignoring axiom layers";
    std::getline(file, line);  // number of variables
    if (!isint(line, 0)) [[unlikely]]
        LOG_ERROR << "Corrupted file";
    unsigned int num_variables = stoi(line);
    std::vector<int> var_ranges = std::vector<int>(num_variables);
    for (unsigned int var_i = 0; var_i < num_variables; var_i++) {
        // process each variable
        std::getline(file, line);  // begin_variable
        if (line != "begin_variable") [[unlikely]]
            LOG_ERROR << "Corrupted file";
        std::getline(file, line);  // variable name (ignored)
        std::getline(file, line);  // axiom layer (ignored)
        if (line != "-1") [[unlikely]]
            LOG_ERROR << "Axiom layer is not -1, this software is not made for this instance";
        std::getline(file, line);  // range of variable
        if (!isint(line, 0)) [[unlikely]]
            LOG_ERROR << "Corrupted file";
        const int range{stoi(line)};
        var_ranges[var_i] = range;
        for (int j = 0; j < range; j++) std::getline(file, line);  // name for variable value (ignored)
        std::getline(file, line);                                  // end_variable
        if (line != "end_variable") [[unlikely]]
            LOG_ERROR << "Corrupted file";
    }

    // * mutex section (ignored)
    if (BASIC_VERBOSE()) LOG_WARNING << "Ignoring mutex section";
    std::getline(file, line);  // number of mutex groups
    if (!isint(line, 0)) [[unlikely]]
        LOG_ERROR << "Corrupted file";
    const unsigned int nmgroups{static_cast<unsigned int>(stoi(line))};
    for (unsigned int i = 0; i < nmgroups; i++) {
        std::getline(file, line);  // begin_mutex_group
        if (line != "begin_mutex_group") [[unlikely]]
            LOG_ERROR << "Corrupted file";
        while (line != "end_mutex_group") {
            std::getline(file,
                         line);  // reach end_mutex_group (ignore all content)
            if (line == "begin_state") [[unlikely]]
                LOG_ERROR << "Corrupted file";
        }
    }

    // * initial state section
    std::getline(file, line);  // begin_state
    if (line != "begin_state") [[unlikely]]
        LOG_ERROR << "Corrupted file";
    std::vector<int> tmp_istate(num_variables);
    for (unsigned int var_i = 0; var_i < num_variables; var_i++) {
        std::getline(file, line);  // initial value of var_i
        if (!isint(line, 0, var_ranges[var_i] - 1)) [[unlikely]]
            LOG_ERROR << "Corrupted file";
        const int val{stoi(line)};
        tmp_istate[var_i] = val;
    }
    std::getline(file, line);  // end_state
    if (line != "end_state") [[unlikely]]
        LOG_ERROR << "Corrupted file";

    // * goal state section
    std::getline(file, line);  // begin_goal
    if (line != "begin_goal") [[unlikely]]
        LOG_ERROR << "Corrupted file";
    std::vector<int> tmp_goal(num_variables, -1);
    std::getline(file, line);  // number of goals
    if (!isint(line, 0, static_cast<int>(num_variables))) [[unlikely]]
        LOG_ERROR << "Corrupted file";
    const unsigned int ngoals{static_cast<unsigned int>(stoi(line))};
    for (unsigned int _ = 0; _ < ngoals; _++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(file, line);  // pair 'variable goal'
        tokens = split_string(line, ' ');
        if (tokens.size() != 2) [[unlikely]]
            LOG_ERROR << "Corrupted file";
        if (!isint(tokens[0], 0, static_cast<int>(num_variables) - 1)) [[unlikely]]
            LOG_ERROR << "Corrupted file";  // variable index
        const unsigned int var{static_cast<unsigned int>(stoi(tokens[0]))};
        if (!isint(tokens[1], 0, var_ranges[var] - 1)) [[unlikely]]
            LOG_ERROR << "Corrupted file";  // variable goal
        const int value{stoi(tokens[1])};
        tmp_goal[var] = value;
    }
    std::getline(file, line);  // end_goal
    if (line != "end_goal") [[unlikely]]
        LOG_ERROR << "Corrupted file";

    // * operator (actions) section
    int checkcosts{-1};
    bool equalcosts_check{true};
    if (BASIC_VERBOSE()) LOG_WARNING << "Ignoring effect conditions";
    std::getline(file, line);  // n_act
    if (!isint(line, 0)) [[unlikely]]
        LOG_ERROR << "Corrupted file";
    inst.m = stoi(line);
    inst.actions = std::vector<action>(inst.m);
    inst.actions_names = std::vector<std::string>(inst.m);
    std::vector<std::vector<std::pair<unsigned int, unsigned int>>> tmp_act_pre(inst.m), tmp_act_eff(inst.m);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        // process each action
        std::getline(file, line);  // begin_operator
        if (line != "begin_operator") [[unlikely]]
            LOG_ERROR << "Corrupted file";
        std::getline(file, line);  // symbolic action name
        std::string name{line};
        std::vector<int> act_pre(num_variables, -1);
        std::getline(file, line);  // number of prevail conditions
        if (!isint(line, 0, static_cast<int>(num_variables))) [[unlikely]]
            LOG_ERROR << "Corrupted file";
        const unsigned int n_pre{static_cast<unsigned int>(stoi(line))};
        for (unsigned int pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(file, line);  // pair 'variable value'
            tokens = split_string(line, ' ');
            if (tokens.size() != 2) [[unlikely]]
                LOG_ERROR << "Corrupted file";
            if (!isint(tokens[0], 0, static_cast<int>(num_variables) - 1)) [[unlikely]]
                LOG_ERROR << "Corrupted file";  // variable index
            const unsigned int var{static_cast<unsigned int>(stoi(tokens[0]))};
            if (!isint(tokens[1], 0, var_ranges[var] - 1)) [[unlikely]]
                LOG_ERROR << "Corrupted file";  // variable value
            const int value{stoi(tokens[1])};
            act_pre[var] = value;
        }
        std::getline(file, line);  // number of effects
        if (!isint(line, 0)) [[unlikely]]
            LOG_ERROR << "Corrupted file";
        const unsigned int n_eff{static_cast<unsigned int>(stoi(line))};
        std::vector<int> act_eff(num_variables, -1);
        for (unsigned int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(file, line);  // effect line
            std::vector<std::string> tokens;
            tokens = split_string(line, ' ');
            if (tokens.size() != 4) [[unlikely]]
                LOG_ERROR << "This program won't handle effect conditions";  // not expecting effect conditions
            if (!isint(tokens[0], 0, 0)) [[unlikely]]
                LOG_ERROR << "This program won't handle effect conditions";  // number of effect conditions (ignored and check to be 0)
            if (!isint(tokens[1], 0, static_cast<int>(num_variables) - 1)) [[unlikely]]
                LOG_ERROR << "Corrupted file";  // variable affected by the action
            const unsigned int var{static_cast<unsigned int>(stoi(tokens[1]))};
            if (!isint(tokens[2], -1, var_ranges[var] - 1)) [[unlikely]]
                LOG_ERROR << "Corrupted file";  // precondition of the variable
            const int pre_val{stoi(tokens[2])};
            if (!isint(tokens[3], 0, var_ranges[var] - 1)) [[unlikely]]
                LOG_ERROR << "Corrupted file";  // effect of the variable
            const int eff_val{stoi(tokens[3])};
            if (pre_val >= 0) act_pre[var] = pre_val;
            act_eff[var] = eff_val;
        }
        std::getline(file, line);  // action cost
        if (!isint(line, 0)) [[unlikely]]
            LOG_ERROR << "Corrupted file";
        unsigned int cost{1};
        if (!inst.equal_costs) {
            cost = stoi(line);
            if (checkcosts == -1)
                checkcosts = cost;
            else if (static_cast<unsigned int>(checkcosts) != cost)
                equalcosts_check = false;
        }
        std::getline(file, line);  // end_operator
        if (line != "end_operator") [[unlikely]]
            LOG_ERROR << "Corrupted file";
        inst.actions[act_i] = action{.pre = binary_set(),
                                     .eff = binary_set(),
                                     .pre_sparse = std::vector<unsigned int>(),
                                     .eff_sparse = std::vector<unsigned int>(),
                                     .cost = cost};
        inst.actions_names[act_i] = name;

        for (unsigned int i = 0; i < num_variables; i++) {
            if (act_pre[i] >= 0) tmp_act_pre[act_i].emplace_back(i, act_pre[i]);
            if (act_eff[i] >= 0) tmp_act_eff[act_i].emplace_back(i, act_eff[i]);
        }
    }
    inst.equal_costs = equalcosts_check;

    if (BASIC_VERBOSE()) LOG_WARNING << "Ignoring axiom section";

    file.close();

    if (inst.m == 0) {
        for (unsigned int i = 0; i < num_variables; i++) {
            if (tmp_istate[i] == tmp_goal[i] || tmp_goal[i] < 0) continue;
            stats.parsing = GET_TIME();
            inst.sol_s = solution_status::INFEAS;
            stats.status = HPLUS_STATUS_INFEAS;
            return;
        }
    }

    // ====================================================== //
    // ================== BINARY EXPANSION ================== //
    // ====================================================== //

    if (BASIC_VERBOSE()) LOG_INFO << "Performing binary expansion";
    size_t n_exp{0};
    std::vector<size_t> offsets(num_variables);
    for (size_t i = 0; i < num_variables; i++) {
        offsets[i] = n_exp;
        n_exp += var_ranges[i];
    }
    binary_set istate = binary_set(n_exp);
    inst.goal = binary_set(n_exp);
    for (size_t i = 0; i < num_variables; i++) {
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

    // ====================================================== //
    // ================ INITIAL STATE REMOVAL =============== //
    // ====================================================== //

    if (BASIC_VERBOSE()) LOG_INFO << "Removing initial state variables";
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
    inst.nfadd = 0;
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
        inst.nfadd += inst.actions[i].eff_sparse.size();
    }

    inst.nfadd = inst.nfadd;
    stats.n_prep = inst.n;
    stats.m_prep = inst.m;
    stats.nfadd_prep = inst.nfadd;

    stats.parsing = GET_TIME();

    const bool is_infeasible = (  // add here other fast feasibility checks
        inst.m == 0 && !inst.goal.empty());

    if (is_infeasible) {
        inst.sol_s = solution_status::INFEAS;
        stats.status = HPLUS_STATUS_INFEAS;
    }

    return;
}

void hplus::update_sol(const execution& exec, instance& inst, const solution& sol, statistics& stats) {
    const auto& [sol_plan, sol_cost, _]{sol};
    binary_set dbcheck{inst.m};
    unsigned int costcheck{0};
    ASSERT(sol_plan.size() <= inst.m);  // check that there aren't more actions that there exists
    binary_set state{inst.n};
    for (const auto& act_i : sol_plan) {
        ASSERT(act_i < inst.m);   // check that the solution only contains existing actions
        ASSERT(!dbcheck[act_i]);  // check that there are no duplicates
        dbcheck.add(act_i);
        ASSERT(state.contains(inst.actions[act_i].pre));  // check if the preconditions are respected at each step
        state |= inst.actions[act_i].eff;
        costcheck += inst.actions[act_i].cost;
    }
    ASSERT(state.contains(inst.goal));  // check if the solution leads to the goal state
    ASSERT(costcheck == sol_cost);      // check if the cost is the declared one

    if (sol_cost >= inst.sol.cost) return;

    {
        static std::mutex sol_mutex;
        std::lock_guard<std::mutex> lock(sol_mutex);
        if (sol_cost >= inst.sol.cost) return;
        inst.sol.updating = true;
        inst.sol.sequence = sol_plan;
        inst.sol.cost = sol_cost;
        stats.cost = sol_cost;
        inst.sol.updating = false;
        if (BASIC_VERBOSE()) LOG_INFO << "Updated best solution - Cost: " << sol_cost;
    }
}

void hplus::run(execution& exec, instance& inst, statistics& stats) {
    if (inst.sol_s == solution_status::INFEAS) return;
    try {
        auto stopcheck = []() {
            if (CHECK_STOP()) [[unlikely]]
                throw timelimit_exception("Reached time limit.");
        };
        stopcheck();

        // ~~~~~~~~~~~~ PREPROCESSING ~~~~~~~~~~~~ //
        prep::prepare_preprocessing(inst);
        if (exec.prep) {
            exec.exec_s = exec_status::PREPROCESSING;
            prep::preprocess(exec, inst, stats);
        }
        prep::prepare_optimization_helpers(exec, inst, stats);
        stopcheck();

        // ~~~~~~~~~~~~~~ HEURISTIC ~~~~~~~~~~~~~~ //
        if (exec.ws > warmstart::NONE || exec.alg >= algorithm::GC) {
            exec.exec_s = exec_status::HEURISTIC;
            heur::heuristic(exec, inst, stats);
        }
        stopcheck();

        if (inst.sol_s == solution_status::INFEAS) return;

        if (exec.alg >= algorithm::GC) {
            exec.exec_s = exec_status::EXIT;
            return;
        }

        // ~~~~~~~~~~~~~~~~ CPLEX ~~~~~~~~~~~~~~~~ //

        exec.exec_s = exec_status::CPX_EXEC;
        exact::exact(exec, inst, stats);
        stopcheck();

    } catch (timelimit_exception& e) {
    } catch (std::bad_alloc& e) {
        LOG_WARNING << "OUT OF MEMORY";
    }

    exec.exec_s = exec_status::EXIT;
}