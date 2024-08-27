#include "../include/hplus_instance.hpp"

// ##################################################################### //
// ########################### HPLUS_VARIABLE ########################## //
// ##################################################################### //

HPLUS_variable::HPLUS_variable(const unsigned int range, const std::string& name, const std::vector<std::string>& val_names) {

    #if HPLUS_INTCHECK
    my::assert(val_names.size() == range, "HPLUS_variable:HPLUS_variable failed.");
    #endif

    this -> range_ = range;
    this -> name_ = std::string(name);
    this -> val_names_ = std::vector<std::string>(val_names);

}

unsigned int HPLUS_variable::get_range() const { return this -> range_; }

const std::string& HPLUS_variable::get_name() const { return this -> name_; }

const std::vector<std::string>& HPLUS_variable::get_val_names() const { return this -> val_names_; }

// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

HPLUS_action::HPLUS_action(my::BitField& pre_bf, my::BitField& eff_bf, const unsigned int cost, const std::string& name) {

    this -> pre_ = my::BitField(pre_bf);
    this -> eff_ = my::BitField(eff_bf);
    this -> cost_ = cost;
    this -> name_ = name;

}

const my::BitField& HPLUS_action::get_pre() const { return this -> pre_; }

const my::BitField& HPLUS_action::get_eff() const { return this -> eff_; }

const std::string& HPLUS_action::get_name() const { return this -> name_; }

unsigned int HPLUS_action::get_cost() const { return this -> cost_; }

// ##################################################################### //
// ########################### HPLUS_INSTANCE ########################## //
// ##################################################################### //

HPLUS_instance::HPLUS_instance(const std::string& file_path) {

    this -> version_ = 0;
    this -> use_costs_ = false;
    this -> n_var_ = 0;
    this -> n_act_ = 0;
    this -> nvarstrips_ = 0;

    std::ifstream file(file_path.c_str(), std::ifstream::in);
    my::assert(file.good(), "Opening instance file failed.");
    parse_inst_file_(&file);
    file.close();

    this -> best_nact_ = 0;
    this -> best_cost_ = UINT_MAX;

    HPLUS_env.logger.print_info("Created HPLUS_instance.");

}

int HPLUS_instance::get_version() const { return this -> version_; }

bool HPLUS_instance::unitary_cost() const { return !this -> use_costs_; }

unsigned int HPLUS_instance::get_nvar() const { return this -> n_var_; }

unsigned int HPLUS_instance::get_nact() const { return this -> n_act_; }

unsigned int HPLUS_instance::get_nvar_strips() const { return this -> nvarstrips_; }

const std::vector<HPLUS_variable>& HPLUS_instance::get_variables() const { return this -> variables_; }

const std::vector<HPLUS_action>& HPLUS_instance::get_actions() const { return this -> actions_; }

const my::BitField& HPLUS_instance::get_istate() const { return this -> initial_state_; }

const my::BitField& HPLUS_instance::get_gstate() const { return this -> goal_state_; }

void HPLUS_instance::update_best_solution(const std::vector<unsigned int>& solution, const unsigned int cost) {

    #if HPLUS_INTCHECK
    my::BitField dbcheck = my::BitField(this -> n_act_);
    unsigned int costcheck = 0;
    my::assert(solution.size() <= this -> n_act_, "Solution has more actions that there actually exists.");    // check that there aren't more actions that there exists
    my::BitField feas_checker(this -> nvarstrips_);
    for (auto p : this -> initial_state_) feas_checker.set(p);
    for (auto act_i : solution) {
        my::assert(act_i < this -> n_act_, "Solution contains unexisting action.");     // check that the solution only contains existing actions
        my::assert(!dbcheck[act_i], "Solution contains duplicate action.");            // check that there are no duplicates
        dbcheck.set(act_i);
        my::assert(feas_checker.contains(this -> actions_[act_i].get_pre()), "Preconditions are not respected.");       // check if the preconditions are respected at each step
        feas_checker |= this -> actions_[act_i].get_eff();
        costcheck += this -> actions_[act_i].get_cost();
    }
    my::assert(costcheck == cost, "Declared cost is different from calculated one.");        // check if the cost is the declared one
    #endif

    if (cost >= this -> best_cost_) return;

    this -> best_solution_ = std::vector<unsigned int>(solution);
    this -> best_nact_ = solution.size();
    this -> best_cost_ = cost;

    HPLUS_env.logger.print_info("Updated best solution - Cost: %4d.", this -> best_cost_);

}

void HPLUS_instance::get_best_solution(std::vector<unsigned int>& solution, unsigned int& cost) const {

    solution = std::vector<unsigned int>(this -> best_solution_);
    cost = this -> best_cost_;

}

void HPLUS_instance::print_best_sol() const {

    HPLUS_env.logger.print("Cost of the solution: %d.", this -> best_cost_);
    for(auto act_i : this -> best_solution_) HPLUS_env.logger.print("(%s)", this -> actions_[act_i].get_name().c_str());

}

unsigned int HPLUS_instance::get_best_cost() const { return this -> best_cost_; }

void HPLUS_instance::parse_inst_file_(std::ifstream* ifs) {

    HPLUS_env.logger.print_info("Parsing SAS file.");

    double start_time = HPLUS_env.get_time();
    std::string line;

    // * version section
    std::getline(*ifs, line);   // begin_version
    my::assert(line == "begin_version", "Corrupted file.");
    std::getline(*ifs, line);   // version_number
    my::assert(my::isint(line), "Corrupted file.");
    this -> version_ = std::stoi(line);
    std::getline(*ifs, line);   // end_version
    my::assert(line == "end_version", "Corrupted file.");

    // * metric section
    std::getline(*ifs, line);   // begin_metric
    my::assert(line == "begin_metric", "Corrupted file.");
    std::getline(*ifs, line);   // metric
    my::assert(my::isint(line, 0, 1), "Corrupted file.");
    this -> use_costs_ = stoi(line) == 1;
    std::getline(*ifs, line);   // end_metric
    my::assert(line == "end_metric", "Corrupted file.");

    // * variables section
    HPLUS_env.logger.print_warn("Ignoring axiom layers.");
    std::getline(*ifs, line);   // n_var
    my::assert(my::isint(line, 0), "Corrupted file.");
    this -> n_var_ = std::stoi(line);
    this -> variables_ = std::vector<HPLUS_variable>(this -> n_var_);
    this -> nvarstrips_ = 0;
    for (unsigned int var_i = 0; var_i < this -> n_var_; var_i++) {
        // process each variable
        std::getline(*ifs, line);   // begin_variable
        my::assert(line == "begin_variable", "Corrupted file.");
        std::string name;
        std::getline(*ifs, name);   // variable name
        std::getline(*ifs, line);   // axiom layer (ignored)
        if (line != "-1") HPLUS_env.logger.print_warn("Axiom layer is %s.", line.c_str());
        std::getline(*ifs, line);   // range of variable
        my::assert(my::isint(line, 0), "Corrupted file.");
        unsigned int range = stoi(line);
        this -> nvarstrips_ += range;
        std::vector<std::string> val_names(range);
        for (unsigned int j = 0; j < range; j++) {
            std::getline(*ifs, line);   // name for variable value
            val_names[j] = line;
        }
        this -> variables_[var_i] = HPLUS_variable(range, name, val_names);
        std::getline(*ifs, line);   // end_variable
        my::assert(line == "end_variable", "Corrupted file.");
    }
    
    // * mutex section (ignored)
    HPLUS_env.logger.print_warn("Ignoring mutex section.");
    std::getline(*ifs, line);   // number of mutex groups
    my::assert(my::isint(line, 0), "Corrupted file.");
    unsigned int nmgroups = stoi(line);
    for (unsigned int i = 0; i < nmgroups; i++) {
        std::getline(*ifs, line);   // begin_mutex_group
        my::assert(line == "begin_mutex_group", "Corrupted file.");
        while (line != "end_mutex_group") {
            std::getline(*ifs, line); // reach end_mutex_group (ignore all content)
            my::assert(line != "begin_state", "Corrupted file.");
        }
    }

    // * initial state section
    std::getline(*ifs, line);   // begin_state
    my::assert(line == "begin_state", "Corrupted file.");
    this -> initial_state_ = my::BitField(this -> nvarstrips_);
    for (unsigned int var_i = 0, c = 0; var_i < this -> n_var_; c += this -> variables_[var_i].get_range(), var_i++) {
        std::getline(*ifs, line);   // initial value of var_i
        my::assert(my::isint(line, 0, this -> variables_[var_i].get_range() - 1), "Corrupted file.");
        unsigned int val = stoi(line);
        this -> initial_state_.set(c + val);
    }
    std::getline(*ifs, line);   // end_state
    my::assert(line == "end_state", "Corrupted file.");

    // * goal state section
    std::getline(*ifs, line);   // begin_goal
    my::assert(line == "begin_goal", "Corrupted file.");
    this -> goal_state_ = my::BitField(this -> nvarstrips_);
    std::getline(*ifs, line);   // number of goals
    my::assert(my::isint(line, 0, this -> n_var_), "Corrupted file.");
    unsigned int ngoals = stoi(line);
    for (unsigned int i = 0; i < ngoals; i++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(*ifs, line);   // pair 'variable goal'
        tokens = my::split_string(line, ' ');
        my::assert(tokens.size() == 2, "Corrupted file.");
        my::assert(my::isint(tokens[0], 0, this -> n_var_ - 1), "Corrupted file."); // variable index
        unsigned int var = stoi(tokens[0]);
        my::assert(my::isint(tokens[1], 0, this -> variables_[var].get_range() - 1), "Corrupted file."); // variable goal
        unsigned int value = stoi(tokens[1]);
        unsigned int c = 0;
        for (unsigned int j = 0; j < var; c += this -> variables_[j].get_range(), j++) {}
        this -> goal_state_.set(c + value);
    }
    std::getline(*ifs, line);   // end_goal
    my::assert(line == "end_goal", "Corrupted file.");

    // * operator (actions) section
    HPLUS_env.logger.print_warn("Ignoring effect conditions.");
    std::getline(*ifs, line);   // n_act
    my::assert(my::isint(line, 0), "Corrupted file.");
    this -> n_act_ = stoi(line);
    my::assert(this -> n_act_ <= 1000, "Testing small instances only.");                // TODO: Remove this
    this -> actions_ = std::vector<HPLUS_action>(this -> n_act_);
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        // process each action
        std::getline(*ifs, line);   // begin_operator
        my::assert(line == "begin_operator", "Corrupted file.");
        std::getline(*ifs, line);   // symbolic action name
        std::string name = line;
        my::BitField act_pre(this -> nvarstrips_);
        std::getline(*ifs, line);   // number of prevail conditions
        my::assert(my::isint(line, 0, this -> n_var_), "Corrupted file.");
        unsigned int n_pre = stoi(line);
        for (unsigned int pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(*ifs, line);   // pair 'variable value'
            tokens = my::split_string(line, ' ');
            my::assert(tokens.size() == 2, "Corrupted file.");
            my::assert(my::isint(tokens[0], 0, this -> n_var_ - 1), "Corrupted file."); // variable index
            unsigned int var = stoi(tokens[0]);
            my::assert(my::isint(tokens[1], 0, this -> variables_[var].get_range() - 1), "Corrupted file."); // variable value
            unsigned int value = stoi(tokens[1]);
            unsigned int c = 0;
            for (unsigned int i = 0; i < var; c += this -> variables_[i].get_range(), i++) {}
            act_pre.set(c + value);
        }
        std::getline(*ifs, line);   // number of effects
        my::assert(my::isint(line, 0), "Corrupted file.");
        unsigned int n_eff = stoi(line);
        my::BitField act_eff(this -> nvarstrips_);
        for (unsigned int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(*ifs, line);   // effect line
            std::vector<std::string> tokens;
            tokens = my::split_string(line, ' ');
            my::assert(tokens.size() == 4, "This program won't handle effect conditions."); // not expecting effect conditions
            my::assert(my::isint(tokens[0], 0, 0), "This program won't handle effect conditions."); // number of effect conditions (ignored and check to be 0)
            my::assert(my::isint(tokens[1], 0, this -> n_var_ - 1), "Corrupted file.");   // variable affected by the action
            unsigned int var = stoi(tokens[1]);
            my::assert(my::isint(tokens[2], -1, this -> variables_[var].get_range() - 1), "Corrupted file.");    // precondition of the variable
            int pre_val = stoi(tokens[2]);
            my::assert(my::isint(tokens[3], 0, this -> variables_[var].get_range() - 1), "Corrupted file."); // effect of the variable
            unsigned int eff_val = stoi(tokens[3]);
            unsigned int c = 0;
            for (unsigned int i = 0; i < var; c += this -> variables_[i].get_range(), i++){}
            if (pre_val >= 0) act_pre.set(c + pre_val);
            act_eff.set(c + eff_val);
        }
        std::getline(*ifs, line);   // action cost
        my::assert(my::isint(line), "Corrupted file.");
        unsigned int cost = 1;
        if (this -> use_costs_) cost = stoi(line);
        std::getline(*ifs, line);   // end_operator
        my::assert(line == "end_operator", "Corrupted file.");
        this -> actions_[act_i] = HPLUS_action(act_pre, act_eff, cost, name);
    }

    HPLUS_env.logger.print_warn("Ignoring axiom section.");

    HPLUS_stats.parsing_time = HPLUS_env.get_time() - start_time;
    
}