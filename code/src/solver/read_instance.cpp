#include "cli_descriptions.hpp"
#include "solver.hpp"

void Solver::read_instance_() {
    const std::string& input_file = params_.get<cli_desc::input, std::string>();
    std::ifstream file(input_file.c_str(), std::ifstream::in);
    if (!file.good()) {
        logger_[FATAL] << "Unable to open file " << input_file;
    }

    std::string line;

    // * version section
    std::getline(file, line);  // begin_version
    if (line != "begin_version") {
        logger_[FATAL] << "Corrupted file: missing <begin_version>";
    }
    std::getline(file, line);  // version_number (ignored)
    if (!isint(line)) {
        logger_[FATAL] << "Corrupted file: <version_number> is not an integer";
    }
    std::getline(file, line);  // end_version
    if (line != "end_version") {
        logger_[FATAL] << "Corrupted file: missing <end_version>";
    }

    // * metric section
    std::getline(file, line);  // begin_metric
    if (line != "begin_metric") {
        logger_[FATAL] << "Corrupted file: missing <begin_metric>";
    }
    std::getline(file, line);  // metric
    if (!isint(line, 0, 1)) {
        logger_[FATAL] << "Corrupted file: <metric> is not 0 or 1";
    }
    inst_.unit_costs = stoi(line) == 0;
    std::getline(file, line);  // end_metric
    if (line != "end_metric") {
        logger_[FATAL] << "Corrupted file: missing <end_metric>";
    }

    // * variables section
    logger_[WARNING] << "Ignoring axiom layers";
    std::getline(file, line);  // number of variables
    if (!isint(line, 0)) {
        logger_[FATAL] << "Corrupted file: number of variables is not a positive integer";
    }
    auto num_variables = static_cast<unsigned int>(stoi(line));
    logger_[DEBUG] << std::format("{} variables (un-expanded)", num_variables);
    std::vector<int> var_ranges = std::vector<int>(num_variables);
    for (unsigned int var_i = 0; var_i < num_variables; var_i++) {
        // process each variable
        std::getline(file, line);  // begin_variable
        if (line != "begin_variable") {
            logger_[FATAL] << "Corrupted file: missing <begin_variable>";
        }
        std::getline(file, line);  // variable name (ignored)
        std::getline(file, line);  // axiom layer (ignored)
        if (line != "-1") {
            logger_[FATAL] << "Axiom layer is not -1, this software is not made for this instance";
        }
        std::getline(file, line);  // range of variable
        if (!isint(line, 0)) {
            logger_[FATAL] << "Corrupted file: range of variable is not a positive integer";
        }
        const int range{stoi(line)};
        var_ranges[var_i] = range;
        for (int j = 0; j < range; j++) {
            std::getline(file, line);  // name for variable value (ignored)
        }
        std::getline(file, line);  // end_variable
        if (line != "end_variable") {
            logger_[FATAL] << "Corrupted file: missing <end_variable>";
        }
    }

    // * mutex section (ignored)
    logger_[WARNING] << "Ignoring mutex section";
    std::getline(file, line);  // number of mutex groups
    if (!isint(line, 0)) {
        logger_[FATAL] << "Corrupted file: number of mutex groups is not a positive integer";
    }
    const unsigned int nmgroups{static_cast<unsigned int>(stoi(line))};
    for (unsigned int i = 0; i < nmgroups; i++) {
        std::getline(file, line);  // begin_mutex_group
        if (line != "begin_mutex_group") {
            logger_[FATAL] << "Corrupted file: missing <begin_mutex_group>";
        }
        while (line != "end_mutex_group") {
            std::getline(file,
                         line);  // reach end_mutex_group (ignore all content)
            if (line == "begin_state") {
                logger_[FATAL] << "Corrupted file: found a <begin_state> before an <end_mutex_group>";
            }
        }
    }

    // * initial state section
    std::getline(file, line);  // begin_state
    if (line != "begin_state") {
        logger_[FATAL] << "Corrupted file: missing <begin_state>";
    }
    std::vector<int> tmp_istate(num_variables);
    for (unsigned int var_i = 0; var_i < num_variables; var_i++) {
        std::getline(file, line);  // initial value of var_i
        if (!isint(line, 0, static_cast<long long>(var_ranges[var_i]) - 1)) {
            logger_[FATAL] << "Corrupted file: invalid value for variable in initial state";
        }
        const int val{stoi(line)};
        tmp_istate[var_i] = val;
    }
    std::getline(file, line);  // end_state
    if (line != "end_state") {
        logger_[FATAL] << "Corrupted file: missing <end_state>";
    }

    // * goal state section
    std::getline(file, line);  // begin_goal
    if (line != "begin_goal") {
        logger_[FATAL] << "Corrupted file: missing <begin_goal>";
    }
    std::vector<int> tmp_goal(num_variables, -1);
    std::getline(file, line);  // number of goals
    if (!isint(line, 0, static_cast<long long>(num_variables))) {
        logger_[FATAL] << "Corrupted file: invalid number of goals";
    }
    const unsigned int ngoals{static_cast<unsigned int>(stoi(line))};
    for (unsigned int _ = 0; _ < ngoals; _++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(file, line);  // pair 'variable goal'
        tokens = split_string(line, ' ');
        if (tokens.size() != 2) {
            logger_[FATAL] << "Corrupted file: expected pair variable goal";
        }
        if (!isint(tokens[0], 0, static_cast<long long>(num_variables) - 1)) {
            logger_[FATAL] << "Corrupted file: invalid variable index in goal state";  // variable index
        }
        const unsigned int var{static_cast<unsigned int>(stoi(tokens[0]))};
        if (!isint(tokens[1], 0, static_cast<long long>(var_ranges[var]) - 1)) {
            logger_[FATAL] << "Corrupted file: invalid variable value in goal state";  // variable goal
        }
        const int value{stoi(tokens[1])};
        tmp_goal[var] = value;
    }
    std::getline(file, line);  // end_goal
    if (line != "end_goal") {
        logger_[FATAL] << "Corrupted file: missing <end_goal>";
    }

    // * operator (actions) section
    logger_[WARNING] << "Ignoring effect conditions";
    std::getline(file, line);  // n_act
    if (!isint(line, 0)) {
        logger_[FATAL] << "Corrupted file: number of actions is not a positive integer";
    }
    inst_.m = static_cast<unsigned int>(stoi(line));
    logger_[DEBUG] << std::format("{} actions", inst_.m);
    inst_.actions = std::vector<Action>(inst_.m);
    inst_.actions_names = std::vector<std::string>(inst_.m);
    std::vector<std::vector<std::pair<unsigned int, unsigned int>>> tmp_act_pre(inst_.m);
    std::vector<std::vector<std::pair<unsigned int, unsigned int>>> tmp_act_eff(inst_.m);

    // Allocated once and reset per action to avoid repeated allocation of size num_variables
    std::vector<int> act_pre(num_variables, -1);
    std::vector<int> act_eff(num_variables, -1);
    std::vector<unsigned int> pre_touched;
    std::vector<unsigned int> eff_touched;
    std::vector<std::string> tokens;

    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        // process each action
        pre_touched.clear();
        eff_touched.clear();
        std::getline(file, line);  // begin_operator
        if (line != "begin_operator") {
            logger_[FATAL] << "Corrupted file: missing <begin_operator>";
        }
        std::getline(file, line);  // symbolic action name
        std::string name{std::move(line)};
        std::getline(file, line);  // number of prevail conditions
        if (!isint(line, 0, static_cast<long long>(num_variables))) {
            logger_[FATAL] << "Corrupted file: invalid number of prevail conditions for action";
        }
        const unsigned int n_pre{static_cast<unsigned int>(stoi(line))};
        tmp_act_pre[act_i].reserve(n_pre);
        for (unsigned int pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::getline(file, line);  // pair 'variable value'
            tokens = split_string(line, ' ');
            if (tokens.size() != 2) {
                logger_[FATAL] << "Corrupted file: expected pair variable value";
            }
            if (!isint(tokens[0], 0, static_cast<long long>(num_variables) - 1)) {
                logger_[FATAL] << "Corrupted file: invalid variable index for action precondition";  // variable index
            }
            const unsigned int var{static_cast<unsigned int>(stoi(tokens[0]))};
            if (!isint(tokens[1], 0, static_cast<long long>(var_ranges[var]) - 1)) {
                logger_[FATAL] << "Corrupted file: invalid variable value for action precondition";  // variable value
            }
            const int value{stoi(tokens[1])};
            act_pre[var] = value;
            insert_sorted(pre_touched, var);
        }
        std::getline(file, line);  // number of effects
        if (!isint(line, 0)) {
            logger_[FATAL] << "Corrupted file: number of effects is not a positive integer";
        }
        const unsigned int n_eff{static_cast<unsigned int>(stoi(line))};
        tmp_act_eff[act_i].reserve(n_eff);
        for (unsigned int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(file, line);  // effect line
            tokens = split_string(line, ' ');
            if (tokens.size() != 4) {
                logger_[FATAL] << "This program won't handle effect conditions";  // not expecting effect conditions
            }
            if (!isint(tokens[0], 0, 0)) {
                logger_[FATAL] << "This program won't handle effect conditions";  // number of effect conditions (ignored and check to be 0)
            }
            if (!isint(tokens[1], 0, static_cast<long long>(num_variables) - 1)) {
                logger_[FATAL] << "Corrupted file: invalid variable index for action effect";  // variable affected by the action
            }
            const unsigned int var{static_cast<unsigned int>(stoi(tokens[1]))};
            if (!isint(tokens[2], -1, static_cast<long long>(var_ranges[var]) - 1)) {
                logger_[FATAL] << "Corrupted file: invalid variable value for action precondition (effects)";  // precondition of the variable
            }
            const int pre_val{stoi(tokens[2])};
            if (!isint(tokens[3], 0, static_cast<long long>(var_ranges[var]) - 1)) {
                logger_[FATAL] << "Corrupted file: invalid variable value for action effect";  // effect of the variable
            }
            const int eff_val{stoi(tokens[3])};
            if (pre_val >= 0) {
                insert_sorted(pre_touched, var);  // insert_sorted deduplicates, so no need to check act_pre[var]
                act_pre[var] = pre_val;
            }
            act_eff[var] = eff_val;
            insert_sorted(eff_touched, var);
        }
        std::getline(file, line);  // action cost
        if (!isint(line, 0)) {
            logger_[FATAL] << "Corrupted file: action cost is not a positive integer";
        }
        unsigned int cost{1};
        if (!inst_.unit_costs) {
            cost = static_cast<unsigned int>(stoi(line));
        }
        std::getline(file, line);  // end_operator
        if (line != "end_operator") {
            logger_[FATAL] << "Corrupted file: missing <end_operator>";
        }
        inst_.actions[act_i] = Action{.pre_sparse = std::vector<unsigned int>(), .eff_sparse = std::vector<unsigned int>(), .cost = cost};
        inst_.actions_names[act_i] = std::move(name);

        // Build sparse from touched indices only — O(n_pre + n_eff) instead of O(num_variables)
        for (const unsigned int var : pre_touched) {
            tmp_act_pre[act_i].emplace_back(var, static_cast<unsigned int>(act_pre[var]));
            act_pre[var] = -1;  // reset for next action
        }
        for (const unsigned int var : eff_touched) {
            tmp_act_eff[act_i].emplace_back(var, static_cast<unsigned int>(act_eff[var]));
            act_eff[var] = -1;  // reset for next action
        }
    }

    logger_[WARNING] << "Ignoring axiom section";

    file.close();

    // ====================================================== //
    // ================== BINARY EXPANSION ================== //
    // ====================================================== //

    logger_[INFO] << ("Performing binary expansion");
    unsigned int n_exp{0};
    std::vector<unsigned int> offsets(num_variables);
    for (size_t i = 0; i < num_variables; i++) {
        offsets[i] = n_exp;
        n_exp += var_ranges[i];
    }
    inst_.start = BinarySet(n_exp);
    inst_.goal = BinarySet(n_exp);
    for (size_t i = 0; i < num_variables; i++) {
        inst_.start.add(offsets[i] + static_cast<unsigned int>(tmp_istate[i]));
        if (tmp_goal[i] >= 0) {
            inst_.goal.add(offsets[i] + static_cast<unsigned int>(tmp_goal[i]));
        }
    }
    inst_.n = n_exp;
    inst_.nfadd = 0;
    logger_[DEBUG] << std::format("{} variables (binary-expanded)", inst_.n);
    for (size_t i = 0; i < inst_.m; i++) {
        std::vector<unsigned int> act_pre_exp;
        std::vector<unsigned int> act_eff_exp;
        act_pre_exp.reserve(tmp_act_pre[i].size());
        act_eff_exp.reserve(tmp_act_eff[i].size());
        for (const auto& [var, val] : tmp_act_pre[i]) {
            act_pre_exp.push_back(offsets[var] + static_cast<unsigned int>(val));
        }
        for (const auto& [var, val] : tmp_act_eff[i]) {
            act_eff_exp.push_back(offsets[var] + static_cast<unsigned int>(val));
        }
        inst_.actions[i].pre_sparse = std::move(act_pre_exp);
        inst_.actions[i].eff_sparse = std::move(act_eff_exp);
        inst_.nfadd += inst_.actions[i].eff_sparse.size();
    }
}