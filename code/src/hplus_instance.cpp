#include "../include/hplus_instance.hpp"

// ##################################################################### //
// ########################### HPLUS_VARIABLE ########################## //
// ##################################################################### //

HPLUS_variable::HPLUS_variable(const unsigned int range, const std::string name, const std::vector<std::string> val_names) {

    #if INTCHECKS
    ASSERT(val_names.size() == range);
    #endif

    this -> range = range;
    this -> name = std::string(name);
    this -> val_names = new std::string[this -> range];
    for (int i = 0; i < this -> range; i++) this -> val_names[i] = val_names[i];

}

HPLUS_variable::~HPLUS_variable() { MYDELL(this->val_names); }

unsigned int HPLUS_variable::get_range() const { return this -> range; }

const std::string* HPLUS_variable::get_name() const { return &this -> name; }

const std::string* HPLUS_variable::get_val_names() const { return this -> val_names; }

// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

HPLUS_action::HPLUS_action(my::BitField* pre_bf, my::BitField* eff_bf, const unsigned int cost, const std::string name) {
    
    #if INTCHECKS
    ASSERT(pre_bf != nullptr);
    ASSERT(eff_bf != nullptr);
    #endif

    this -> pre = pre_bf; pre_bf = nullptr;
    this -> eff = eff_bf; eff_bf = nullptr;
    this -> cost = cost;
    this -> name = name.c_str();

}

HPLUS_action::~HPLUS_action() { MYDEL(this->pre); MYDEL(this->eff); }

const my::BitField* HPLUS_action::get_pre() const { return this -> pre; }

const my::BitField* HPLUS_action::get_eff() const { return this -> eff; }

const std::string* HPLUS_action::get_name() const { return &this -> name; }

unsigned int HPLUS_action::get_cost() const { return this -> cost; }

// ##################################################################### //
// ########################### HPLUS_INSTANCE ########################## //
// ##################################################################### //

HPLUS_instance::HPLUS_instance(const std::string file_path) {

    std::ifstream file(file_path.c_str(), std::ifstream::in);
    MYASSERT(file.good());
    parse_inst_file(&file);
    file.close();

    HPLUS_env.logger.print_info("Created HPLUS_instance.");

}

HPLUS_instance::~HPLUS_instance() {

    for (int i = 0; i < this -> n_var; i++) delete this -> variables[i];
    MYDELL(this -> variables);
    for (int i = 0; i < this -> n_act; i++) delete this -> actions[i];
    MYDELL(this->actions);
    MYDEL(this->initial_state);
    MYDEL(this->goal_state);
    MYDELL(this->best_solution);

}

int HPLUS_instance::get_version() const { return this -> version; }

bool HPLUS_instance::unitary_cost() const { return !this -> use_costs; }

unsigned int HPLUS_instance::get_nvar() const { return this -> n_var; }

unsigned int HPLUS_instance::get_nact() const { return this -> n_act; }

unsigned int HPLUS_instance::get_bfsize() const { return this -> bf_size; }

const HPLUS_variable** HPLUS_instance::get_variables() const { return this -> variables; }

const HPLUS_action** HPLUS_instance::get_actions() const { return this -> actions; }

const my::BitField* HPLUS_instance::get_istate() const { return this -> initial_state; }

const my::BitField* HPLUS_instance::get_gstate() const { return this -> goal_state; }

// TODO: Maybe make thread safe?
void HPLUS_instance::update_best_solution(const std::vector<unsigned int> solution, const unsigned int cost) {

    unsigned int nact = solution.size();
    #if INTCHECKS
    bool* dbcheck = new bool[this -> n_act]();
    unsigned int costcheck = 0;
    ASSERT(nact <= this -> n_act);
    for (int i = 0; i < nact; i++) {
        ASSERT(solution[i] < this -> n_act);
        ASSERT(!dbcheck[solution[i]]);
        dbcheck[solution[i]] = 1;
        costcheck += this -> actions[solution[i]] -> get_cost();
    }
    ASSERT(costcheck == cost);
    DELL(dbcheck);
    #endif

    if (cost >= this -> best_cost) return;

    if (nact > this -> best_nact) {
        if (this -> best_solution != nullptr) delete[] this -> best_solution;
        this -> best_solution = new unsigned int[nact];
    }
    for (int i = 0; i < nact; i++) this -> best_solution[i] = solution[i];
    this -> best_nact = nact;
    this -> best_cost = cost;

    HPLUS_env.logger.print_info("Updated best solution - Cost: %4d.", this -> best_cost);

}

void HPLUS_instance::get_best_solution(std::vector<unsigned int>* solution, unsigned int* cost) const {

    #if INTCHECKS
    ASSERT(solution != nullptr);
    ASSERT(cost != nullptr);
    #endif

    solution -> clear();
    (*solution) = std::vector<unsigned int>(this -> best_nact);
    for (int i = 0; i < this -> best_nact; i++) (*solution)[i] = this -> best_solution[i];
    *cost = this -> best_cost;

}

unsigned int HPLUS_instance::get_best_cost() const { return this -> best_cost; }

void HPLUS_instance::parse_inst_file(std::ifstream* ifs) {

    HPLUS_env.logger.print_info("Parsing SAS file.");

    double start_time = HPLUS_env.get_time();
    std::string line;

    // * version section
    std::getline(*ifs, line);   // begin_version
    MYASSERT(line == "begin_version");
    std::getline(*ifs, line);   // version_number
    MYASSERT(my::isint(line));
    this -> version = std::stoi(line);
    std::getline(*ifs, line);   // end_version
    MYASSERT(line == "end_version");

    // * metric section
    std::getline(*ifs, line);   // begin_metric
    MYASSERT(line == "begin_metric");
    std::getline(*ifs, line);   // metric
    MYASSERT(my::isint(line, 0, 1));
    this -> use_costs = stoi(line) == 1;
    std::getline(*ifs, line);   // end_metric
    MYASSERT(line == "end_metric");

    // * variables section
    HPLUS_env.logger.print_warn("Ignoring axiom layers.");
    std::getline(*ifs, line);   // n_var
    MYASSERT(my::isint(line, 0));
    this -> n_var = (unsigned int) std::stoi(line);
    this -> variables = new const HPLUS_variable*[this -> n_var];
    this -> bf_size = 0;
    for (int var_i = 0; var_i < this -> n_var; var_i++) {
        // process each variable
        std::getline(*ifs, line);   // begin_variable
        MYASSERT(line == "begin_variable");
        std::string name;
        std::getline(*ifs, name);   // variable name
        std::getline(*ifs, line);   // axiom layer (ignored)
        if (line != "-1") HPLUS_env.logger.print_warn("Axiom layer is %s.", line.c_str());
        std::getline(*ifs, line);   // range of variable
        MYASSERT(my::isint(line, 0));
        unsigned int range = (unsigned int) stoi(line);
        this -> bf_size += range;
        std::vector<std::string> val_names(range);
        for (int j = 0; j < range; j++) {
            std::getline(*ifs, line);   // name for variable value
            val_names[j] = line;
        }
        this -> variables[var_i] = new HPLUS_variable(range, name, val_names);
        std::getline(*ifs, line);   // end_variable
        MYASSERT(line == "end_variable");
    }
    
    // * mutex section (ignored)
    HPLUS_env.logger.print_warn("Ignoring mutex section.");
    std::getline(*ifs, line);   // number of mutex groups
    MYASSERT(my::isint(line, 0));
    int nmgroups = stoi(line);
    for (int i = 0; i < nmgroups; i++) {
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
    this -> initial_state = new my::BitField(this -> bf_size);
    for (int var_i = 0, c = 0; var_i < this -> n_var; c += this -> variables[var_i] -> get_range(), var_i++) {
        std::getline(*ifs, line);   // initial value of var_i
        MYASSERT(my::isint(line, 0, this -> variables[var_i] -> get_range() - 1));
        unsigned int val = (unsigned int) stoi(line);
        this -> initial_state -> set(c + val);
    }
    std::getline(*ifs, line);   // end_state
    MYASSERT(line == "end_state");

    // * goal state section
    std::getline(*ifs, line);   // begin_goal
    MYASSERT(line == "begin_goal");
    this -> goal_state = new my::BitField(this -> bf_size);
    std::getline(*ifs, line);   // number of goals
    MYASSERT(my::isint(line, 0, this -> n_var));
    int ngoals = stoi(line);
    for (int i = 0; i < ngoals; i++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(*ifs, line);   // pair 'variable goal'
        my::split(line, &tokens, ' ');
        MYASSERT(tokens.size() == 2);
        MYASSERT(my::isint(tokens[0], 1, this -> n_var - 1)); // variable index
        unsigned int var = (unsigned int) stoi(tokens[0]);
        MYASSERT(my::isint(tokens[1], 0, this -> variables[var] -> get_range() - 1)); // variable goal
        unsigned int value = (unsigned int) stoi(tokens[1]);
        int c = 0;
        for (int i = 0; i < var; c += this -> variables[i] -> get_range(), i++);
        this -> goal_state -> set(c + value);
    }
    std::getline(*ifs, line);   // end_goal
    MYASSERT(line == "end_goal");

    // * operator (actions) section
    //TODO: Ask if it's ok if I put preconditions together with prevail conditions
    HPLUS_env.logger.print_warn("Ignoring effect conditions.");
    std::getline(*ifs, line);   // n_act
    MYASSERT(my::isint(line, 0));
    this -> n_act = (unsigned int) stoi(line);
    this -> actions = new const HPLUS_action*[this -> n_act];
    for (int act_i = 0; act_i < this -> n_act; act_i++) {
        // process each action
        std::getline(*ifs, line);   // begin_operator
        MYASSERT(line == "begin_operator");
        std::getline(*ifs, line);   // symbolic action name
        std::string name = line;
        my::BitField* act_pre = new my::BitField(this -> bf_size);
        std::getline(*ifs, line);   // number of prevail conditions
        MYASSERT(my::isint(line, 0, this -> n_var));
        unsigned int n_pre = (unsigned int) stoi(line);
        for (int pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(*ifs, line);   // pair 'variable value'
            my::split(line, &tokens, ' ');
            MYASSERT(tokens.size() == 2);
            MYASSERT(my::isint(tokens[0], 0, this -> n_var - 1)); // variable index
            unsigned int var = (unsigned int) stoi(tokens[0]);
            MYASSERT(my::isint(tokens[1], 0, this -> variables[var] -> get_range() - 1)); // variable value
            unsigned int value = (unsigned int) stoi(tokens[1]);
            int c = 0;
            for (int i = 0; i < var; c += this -> variables[i] -> get_range(), i++);
            act_pre -> set(c + value);
        }
        std::getline(*ifs, line);   // number of effects
        MYASSERT(my::isint(line, 0));
        unsigned int n_eff = (unsigned int) stoi(line);
        my::BitField* act_eff = new my::BitField(this -> bf_size);
        for (int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(*ifs, line);   // effect line
            std::vector<std::string> tokens;
            my::split(line, &tokens, ' ');
            MYASSERT(tokens.size() == 4); // not expecting effect conditions
            MYASSERT(my::isint(tokens[0], 0, 0)); // number of effect conditions (ignored and check to be 0)
            MYASSERT(my::isint(tokens[1], 0, this -> n_var - 1));   // variable affected by the action
            unsigned int var = (unsigned int) stoi(tokens[1]);
            MYASSERT(my::isint(tokens[2], -1, this -> variables[var] -> get_range() - 1));    // precondition of the variable
            int pre_val = stoi(tokens[2]);
            MYASSERT(my::isint(tokens[3], 0, this -> variables[var] -> get_range() - 1)); // precondition of the variable
            unsigned int eff_val = (unsigned int) stoi(tokens[3]);
            int c = 0;
            for (int i = 0; i < var; c += this -> variables[i] -> get_range(), i++);
            if (pre_val >= 0) act_pre -> set(c + pre_val);
            act_eff -> set(c + eff_val);
        }
        std::getline(*ifs, line);   // action cost
        MYASSERT(my::isint(line));
        unsigned int cost = 1;
        if (this -> use_costs) cost = (unsigned int) stoi(line);
        std::getline(*ifs, line);   // end_operator
        MYASSERT(line == "end_operator");
        this -> actions[act_i] = new HPLUS_action(act_pre, act_eff, cost, name);
    }

    HPLUS_env.logger.print_warn("Ignoring axiom section.");

    HPLUS_stats.parsing_time = HPLUS_env.get_time() - start_time;

    this -> best_solution = new unsigned int[0];
    this -> best_nact = 0;
    this -> best_cost = UINT_MAX;
    
}