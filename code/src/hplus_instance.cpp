#include "../include/hplus_instance.hpp"

// ##################################################################### //
// ########################### HPLUS_VARIABLE ########################## //
// ##################################################################### //

HPLUS_variable::HPLUS_variable(const unsigned int range, const std::string& name, const std::vector<std::string>& val_names) {

    #if HPLUS_INTCHECK
    MYASSERT(val_names.size() == range);
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
    this -> bf_size_ = 0;

    std::ifstream file(file_path.c_str(), std::ifstream::in);
    MYASSERT(file.good());
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

unsigned int HPLUS_instance::get_bfsize() const { return this -> bf_size_; }

const std::vector<HPLUS_variable>& HPLUS_instance::get_variables() const { return this -> variables_; }

const std::vector<HPLUS_action>& HPLUS_instance::get_actions() const { return this -> actions_; }

const my::BitField& HPLUS_instance::get_istate() const { return this -> initial_state_; }

const my::BitField& HPLUS_instance::get_gstate() const { return this -> goal_state_; }

// TODO: Maybe make thread safe?
void HPLUS_instance::update_best_solution(const std::vector<unsigned int>& solution, const unsigned int cost) {

    #if HPLUS_INTCHECK
    std::vector<bool> dbcheck(this -> n_act_, false);
    unsigned int costcheck = 0;
    MYASSERT(solution.size() <= this -> n_act_);    // check that there aren't more actions that there exists
    my::BitField feas_checker(this -> bf_size_);
    for (auto p : this -> initial_state_) feas_checker.set(p);
    for (auto act_i : solution) {
        MYASSERT(act_i < this -> n_act_);     // check that the solution only contains existing actions
        MYASSERT(!dbcheck[act_i]);            // check that there are no duplicates
        dbcheck[act_i] = true;
        MYASSERT(feas_checker.contains(this -> actions_[act_i].get_pre()));       // check if the preconditions are respected at each step
        feas_checker.unificate(this -> actions_[act_i].get_eff());
        costcheck += this -> actions_[act_i].get_cost();
    }
    MYASSERT(costcheck == cost);        // check if the cost is the declared one
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
    MYASSERT(line == "begin_version");
    std::getline(*ifs, line);   // version_number
    MYASSERT(my::isint(line));
    this -> version_ = std::stoi(line);
    std::getline(*ifs, line);   // end_version
    MYASSERT(line == "end_version");

    // * metric section
    std::getline(*ifs, line);   // begin_metric
    MYASSERT(line == "begin_metric");
    std::getline(*ifs, line);   // metric
    MYASSERT(my::isint(line, 0, 1));
    this -> use_costs_ = stoi(line) == 1;
    std::getline(*ifs, line);   // end_metric
    MYASSERT(line == "end_metric");

    // * variables section
    HPLUS_env.logger.print_warn("Ignoring axiom layers.");
    std::getline(*ifs, line);   // n_var
    MYASSERT(my::isint(line, 0));
    this -> n_var_ = std::stoi(line);
    this -> variables_ = std::vector<HPLUS_variable>(this -> n_var_);
    this -> bf_size_ = 0;
    for (unsigned int var_i = 0; var_i < this -> n_var_; var_i++) {
        // process each variable
        std::getline(*ifs, line);   // begin_variable
        MYASSERT(line == "begin_variable");
        std::string name;
        std::getline(*ifs, name);   // variable name
        std::getline(*ifs, line);   // axiom layer (ignored)
        if (line != "-1") HPLUS_env.logger.print_warn("Axiom layer is %s.", line.c_str());
        std::getline(*ifs, line);   // range of variable
        MYASSERT(my::isint(line, 0));
        unsigned int range = stoi(line);
        this -> bf_size_ += range;
        std::vector<std::string> val_names(range);
        for (unsigned int j = 0; j < range; j++) {
            std::getline(*ifs, line);   // name for variable value
            val_names[j] = line;
        }
        this -> variables_[var_i] = HPLUS_variable(range, name, val_names);
        std::getline(*ifs, line);   // end_variable
        MYASSERT(line == "end_variable");
    }
    
    // * mutex section (ignored)
    HPLUS_env.logger.print_warn("Ignoring mutex section.");
    std::getline(*ifs, line);   // number of mutex groups
    MYASSERT(my::isint(line, 0));
    unsigned int nmgroups = stoi(line);
    for (unsigned int i = 0; i < nmgroups; i++) {
        std::getline(*ifs, line);   // begin_mutex_group
        MYASSERT(line == "begin_mutex_group");
        while (line != "end_mutex_group") {
            std::getline(*ifs, line); // reach end_mutex_group (ignore all content)
            MYASSERT(line != "begin_state");
        }
    }

    // * initial state section
    std::getline(*ifs, line);   // begin_state
    MYASSERT(line == "begin_state");
    this -> initial_state_ = my::BitField(this -> bf_size_);
    for (unsigned int var_i = 0, c = 0; var_i < this -> n_var_; c += this -> variables_[var_i].get_range(), var_i++) {
        std::getline(*ifs, line);   // initial value of var_i
        MYASSERT(my::isint(line, 0, this -> variables_[var_i].get_range() - 1));
        unsigned int val = stoi(line);
        this -> initial_state_.set(c + val);
    }
    std::getline(*ifs, line);   // end_state
    MYASSERT(line == "end_state");

    // * goal state section
    std::getline(*ifs, line);   // begin_goal
    MYASSERT(line == "begin_goal");
    this -> goal_state_ = my::BitField(this -> bf_size_);
    std::getline(*ifs, line);   // number of goals
    MYASSERT(my::isint(line, 0, this -> n_var_));
    unsigned int ngoals = stoi(line);
    for (unsigned int i = 0; i < ngoals; i++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(*ifs, line);   // pair 'variable goal'
        my::split(line, &tokens, ' ');
        MYASSERT(tokens.size() == 2);
        MYASSERT(my::isint(tokens[0], 0, this -> n_var_ - 1)); // variable index
        unsigned int var = stoi(tokens[0]);
        MYASSERT(my::isint(tokens[1], 0, this -> variables_[var].get_range() - 1)); // variable goal
        unsigned int value = stoi(tokens[1]);
        unsigned int c = 0;
        for (unsigned int j = 0; j < var; c += this -> variables_[j].get_range(), j++) {}
        this -> goal_state_.set(c + value);
    }
    std::getline(*ifs, line);   // end_goal
    MYASSERT(line == "end_goal");

    // * operator (actions) section
    HPLUS_env.logger.print_warn("Ignoring effect conditions.");
    std::getline(*ifs, line);   // n_act
    MYASSERT(my::isint(line, 0));
    this -> n_act_ = stoi(line);
    this -> actions_ = std::vector<HPLUS_action>(this -> n_act_);
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        // process each action
        std::getline(*ifs, line);   // begin_operator
        MYASSERT(line == "begin_operator");
        std::getline(*ifs, line);   // symbolic action name
        std::string name = line;
        my::BitField act_pre(this -> bf_size_);
        std::getline(*ifs, line);   // number of prevail conditions
        MYASSERT(my::isint(line, 0, this -> n_var_));
        unsigned int n_pre = stoi(line);
        for (unsigned int pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(*ifs, line);   // pair 'variable value'
            my::split(line, &tokens, ' ');
            MYASSERT(tokens.size() == 2);
            MYASSERT(my::isint(tokens[0], 0, this -> n_var_ - 1)); // variable index
            unsigned int var = stoi(tokens[0]);
            MYASSERT(my::isint(tokens[1], 0, this -> variables_[var].get_range() - 1)); // variable value
            unsigned int value = stoi(tokens[1]);
            unsigned int c = 0;
            for (unsigned int i = 0; i < var; c += this -> variables_[i].get_range(), i++) {}
            act_pre.set(c + value);
        }
        std::getline(*ifs, line);   // number of effects
        MYASSERT(my::isint(line, 0));
        unsigned int n_eff = stoi(line);
        my::BitField act_eff(this -> bf_size_);
        for (unsigned int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(*ifs, line);   // effect line
            std::vector<std::string> tokens;
            my::split(line, &tokens, ' ');
            MYASSERT(tokens.size() == 4); // not expecting effect conditions
            MYASSERT(my::isint(tokens[0], 0, 0)); // number of effect conditions (ignored and check to be 0)
            MYASSERT(my::isint(tokens[1], 0, this -> n_var_ - 1));   // variable affected by the action
            unsigned int var = stoi(tokens[1]);
            MYASSERT(my::isint(tokens[2], -1, this -> variables_[var].get_range() - 1));    // precondition of the variable
            int pre_val = stoi(tokens[2]);
            MYASSERT(my::isint(tokens[3], 0, this -> variables_[var].get_range() - 1)); // precondition of the variable
            unsigned int eff_val = stoi(tokens[3]);
            unsigned int c = 0;
            for (unsigned int i = 0; i < var; c += this -> variables_[i].get_range(), i++){}
            if (pre_val >= 0) act_pre.set(c + pre_val);
            act_eff.set(c + eff_val);
        }
        std::getline(*ifs, line);   // action cost
        MYASSERT(my::isint(line));
        unsigned int cost = 1;
        if (this -> use_costs_) cost = stoi(line);
        std::getline(*ifs, line);   // end_operator
        MYASSERT(line == "end_operator");
        this -> actions_[act_i] = HPLUS_action(act_pre, act_eff, cost, name);
    }

    HPLUS_env.logger.print_warn("Ignoring axiom section.");

    HPLUS_stats.parsing_time = HPLUS_env.get_time() - start_time;
    
}