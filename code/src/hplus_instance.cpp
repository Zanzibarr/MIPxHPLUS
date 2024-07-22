#include "../include/hplus_instance.hpp"

HPLUS_variable::HPLUS_variable(const unsigned int range, const std::string name, const std::vector<std::string> val_names) {

    this -> range = range;
    this -> name = name;
    this -> val_names = std::vector<std::string>(val_names);
}

HPLUS_variable::~HPLUS_variable() {

}

unsigned int HPLUS_variable::get_range() const { return this -> range; }

std::string HPLUS_variable::get_name() const { return name; }

std::vector<std::string> HPLUS_variable::get_val_names() const { return this -> val_names; }

HPLUS_action::HPLUS_action(const BitField* pre_bf, const BitField* eff_bf, const unsigned int cost, const std::string name) {

    unsigned int size = pre_bf -> size();
    
    this -> pre = new BitField(size);
    this -> eff = new BitField(size);
    for (int i = 0; i < size; i++) {
        if ((*pre_bf)[i]) this -> pre -> set(i);
        if ((*eff_bf)[i]) this -> eff -> set(i);
    };
    this -> cost = cost;
    this -> name = name;

}

HPLUS_action::~HPLUS_action() {
    
}

BitField HPLUS_action::get_pre() const { return *this -> pre; }

BitField HPLUS_action::get_eff() const { return *this -> eff; }

std::string HPLUS_action::get_name() const { return this -> name; }

unsigned int HPLUS_action::get_cost() const { return this -> cost; }

HPLUS_domain::HPLUS_domain(const std::string file_path, HPLUS_problem* problem, Logger* logger) {
    
    this -> logger = logger;

    std::ifstream file(file_path.c_str(), std::ifstream::in);
    if (!file.good()) this -> logger -> raise_error("File %s not found.", file_path.c_str());
    parse_inst_file(&file, problem);
    file.close();

    this -> logger -> print_info("Created HPLUS_domain.");

}

bool HPLUS_domain::unitary_cost() const { return !this -> use_costs; }

unsigned int HPLUS_domain::get_nvar() const { return this -> n_var; }

unsigned int HPLUS_domain::get_nact() const { return this -> n_act; }

unsigned int HPLUS_domain::get_bfsize() const { return this -> bf_size; }

const HPLUS_variable* HPLUS_domain::get_variables() const { return this -> variables; }

const HPLUS_action* HPLUS_domain::get_actions() const { return this -> actions; }

const Logger* HPLUS_domain::get_logger() const { return this -> logger; }

void HPLUS_domain::parse_inst_file(std::ifstream* ifs, HPLUS_problem* problem) {
    
    std::string line;

    // version section
    std::getline(*ifs, line);   // begin_version
    my::asserteq(line, "begin_version", this -> logger);
    std::getline(*ifs, line);   // version_number
    my::assertisint(line, this -> logger);
    this -> version = std::stoi(line);
    std::getline(*ifs, line);   // end_version
    my::asserteq(line, "end_version", this -> logger);

    // metric section
    std::getline(*ifs, line);   // begin_metric
    my::asserteq(line, "begin_metric", this -> logger);
    std::getline(*ifs, line);   // metric
    my::assertisint(line, this -> logger, 0, 1);
    std::getline(*ifs, line);   // end_metric
    my::asserteq(line, "end_metric", this -> logger);

    // variables section
    this -> logger -> print_warn("Ignoring axiom layers.");
    std::getline(*ifs, line);   // n_var
    my::assertisint(line, this -> logger, 0);
    this -> n_var = (unsigned int) std::stoi(line);
    this -> variables = new HPLUS_variable[this -> n_var];
    this -> bf_size = 0;
    for (int var_i = 0; var_i < this -> n_var; var_i++) {
        // process each variable
        std::getline(*ifs, line);   // begin_variable
        my::asserteq(line, "begin_variable", this -> logger);
        std::string name;
        std::getline(*ifs, name);   // variable name
        std::getline(*ifs, line);   // axiom layer (ignored)
        if (line != "-1") this -> logger -> print_warn("Axiom layer is %s.", line.c_str());
        std::getline(*ifs, line);   // range of variable
        my::assertisint(line, this -> logger, 0);
        unsigned int range = (unsigned int) stoi(line);
        this -> bf_size += range;
        std::vector<std::string> val_names(range);
        for (int j = 0; j < range; j++) {
            std::getline(*ifs, line);   // name for variable value
            val_names[j] = line;
        }
        this -> variables[var_i] = HPLUS_variable(range, name, val_names);
        std::getline(*ifs, line);   // end_variable
        my::asserteq(line, "end_variable", this -> logger);
    }
    
    // mutex section (ignored)
    this -> logger -> print_warn("Ignoring mutex section.");
    std::getline(*ifs, line);   // number of mutex groups
    my::assertisint(line, this -> logger, 0);
    int nmgroups = stoi(line);
    for (int i = 0; i < nmgroups; i++) {
        std::getline(*ifs, line);   // begin_mutex_group
        my::asserteq(line, "begin_mutex_group", this -> logger);
        while (line != "end_mutex_group") {
            std::getline(*ifs, line); // reach end_mutex_group (ignore all content)
            if (line == "begin_state") this -> logger -> raise_error("Mutex group skipped in SAS file without reaching 'end_mutex_group'.");
        }
    }

    // initial state section
    std::getline(*ifs, line);   // begin_state
    my::asserteq(line, "begin_state", this -> logger);
    std::vector<unsigned int> istate = std::vector<unsigned int>(this -> n_var);
    for (int var_i = 0; var_i < this -> n_var; var_i++) {
        std::getline(*ifs, line);   // initial value of var_i
        my::assertisint(line, this -> logger, 0, this -> variables[var_i].get_range() - 1);
        unsigned int val = (unsigned int) stoi(line);
        istate[var_i] = val;
    }
    std::getline(*ifs, line);   // end_state
    my::asserteq(line, "end_state", this -> logger);

    // goal state section
    std::getline(*ifs, line);   // begin_goal
    my::asserteq(line, "begin_goal", this -> logger);
    std::vector<int> gstate = std::vector<int>(this -> n_var);
    for (int i = 0; i < this -> n_var; i++) gstate[i] = -1;
    std::getline(*ifs, line);   // number of goals
    my::assertisint(line, this -> logger, 0, this -> n_var);
    int ngoals = stoi(line);
    for (int i = 0; i < ngoals; i++) {
        // parsing each goal
        int var = -1, value = -1;
        std::vector<std::string> tokens;
        std::getline(*ifs, line);   // pair 'variable goal'
        my::split(line, &tokens, ' ');
        my::asserteq(tokens.size(), 2, this -> logger);
        my::assertisint(tokens[0], this -> logger, 1, this -> n_var - 1); // variable index
        var = stoi(tokens[0]);
        my::assertisint(tokens[1], this -> logger, 0, this -> variables[var].get_range() - 1); // variable goal
        value = stoi(tokens[1]);
        gstate[var] = value;
    }
    std::getline(*ifs, line);   // end_goal
    my::asserteq(line, "end_goal", this -> logger);

    if (problem != NULL) delete problem;
    problem = new HPLUS_problem(this, istate, gstate);

    // operator (actions) section
    //TODO: Ask if it's ok if I put preconditions together with prevail conditions
    this -> logger -> print_warn("Ignoring effect conditions.");
    std::getline(*ifs, line);   // n_act
    my::assertisint(line, this -> logger, 0);
    this -> n_act = (unsigned int) stoi(line);
    this -> actions = new HPLUS_action[this -> n_act];
    for (int act_i = 0; act_i < this -> n_act; act_i++) {
        // process each action
        std::getline(*ifs, line);   // begin_operator
        my::asserteq(line, "begin_operator", this -> logger);
        std::getline(*ifs, line);   // symbolic action name
        std::string name = line;
        std::vector<int> pre_state = std::vector<int>(this -> n_var);
        for (int i = 0; i < this -> n_var; i++) pre_state[i] = -1;
        std::getline(*ifs, line);   // number of prevail conditions
        my::assertisint(line, this -> logger, 0, this -> n_var);
        unsigned int n_pre = (unsigned int) stoi(line);
        for (int pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            int var = -1, value = -1;
            std::vector<std::string> tokens;
            std::getline(*ifs, line);   // pair 'variable value'
            my::split(line, &tokens, ' ');
            my::asserteq(tokens.size(), 2, this -> logger);
            my::assertisint(tokens[0], this -> logger, 0, this -> n_var - 1); // variable index
            var = stoi(tokens[0]);
            my::assertisint(tokens[1], this -> logger, 0, this -> variables[var].get_range() - 1); // variable value
            value = stoi(tokens[1]);
            pre_state[var] = value;
        }
        std::getline(*ifs, line);   // number of effects
        my::assertisint(line, this -> logger, 0);
        unsigned int n_eff = (unsigned int) stoi(line);
        std::vector<int> eff_state = std::vector<int>(this -> n_var);
        for (int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(*ifs, line);   // effect line
            std::vector<std::string> tokens;
            my::split(line, &tokens, ' ');
            my::asserteq(tokens.size(), 4, this -> logger); // not expecting effect conditions
            my::assertisint(tokens[0], this -> logger, 0, 0);   // number of effect conditions (ignored and check to be 0)
            my::assertisint(tokens[1], this -> logger, 0, this -> n_var - 1);   // variable affected by the action
            unsigned int var = (unsigned int) stoi(tokens[1]);
            my::assertisint(tokens[2], this -> logger, -1, this -> variables[var].get_range() - 1);    // precondition of the variable
            int pre_val = stoi(tokens[2]);
            my::assertisint(tokens[3], this -> logger, 0, this -> variables[var].get_range() - 1);     // effect on the variable
            unsigned int eff_val = (unsigned int) stoi(tokens[3]);
            if (pre_val >= 0) pre_state[var] = pre_val;
            eff_state[var] = eff_val;
        }
        std::getline(*ifs, line);   // action cost
        my::assertisint(line, this -> logger);
        unsigned int cost = 1;
        if (this -> use_costs) cost = (unsigned int) stoi(line);
        std::getline(*ifs, line);   // end_operator
        my::asserteq(line, "end_operator", this -> logger);
        BitField pre = BitField(this -> bf_size);
        BitField eff = BitField(this -> bf_size);
        for (int i = 0, c = 0; i < this -> n_var; i++, c += this -> variables[i].get_range()) {
            if (pre_state[i] >= 0) pre.set(c + pre_state[i]);
            if (eff_state[i] >= 0) eff.set(c + eff_state[i]);
        }
        this -> actions[act_i] = HPLUS_action(&pre, &eff, cost, name);
    }

    #if HPLUS_VERBOSE >= 100

    this -> logger -> print_info("Version: %d.", this -> version);
    this -> logger -> print_info("Metric: %d.", this -> use_costs);
    this -> logger -> print_info("N_var: %d.", this -> n_var);
    for (int i = 0; i < this -> n_var; i++) this -> logger -> print_info("name(var_%d) = '%s'.", i, this -> variables[i].get_name().c_str());
    for (int i = 0; i < this -> n_var; i++) this -> logger -> print_info("range(var_%d) = '%d'.", i, this -> variables[i].get_range());
    for (int i = 0; i < this -> n_var; i++){
        std::vector<std::string> val_names = this -> variables[i].get_val_names();
        for (int j = 0; j < this -> variables[i].get_range(); j++) {
            this -> logger -> print_info("name(var_%d[%d]) = '%s'.", i, j, val_names[j].c_str());
        }
    }
    this -> logger -> print_info("Bitfield size: %d", this -> bf_size);
    this -> logger -> print_info("Initial state: %s.", problem -> get_istate().view().c_str());
    this -> logger -> print_info("Goal state: %s.", problem -> get_gstate().view().c_str());
    this -> logger -> print_info("N_act: %d.", this -> n_act);
    for (int act_i = 0; act_i < this -> n_act; act_i++) {
        this -> logger -> print_info("pre(act_%d) = '%s'.", act_i, this -> actions[act_i].get_pre().view().c_str());
        this -> logger -> print_info("eff(act_%d) = '%s'.", act_i, this -> actions[act_i].get_eff().view().c_str());
        this -> logger -> print_info("name(act_%d) = '%s'.", act_i, this -> actions[act_i].get_name().c_str());
        this -> logger -> print_info("cost(act_%d) = '%d'.", act_i, this -> actions[act_i].get_cost());
    }

    #endif

    this -> logger -> print_info("Parsed SAS file.");
    
}

HPLUS_problem::HPLUS_problem(HPLUS_domain* domain, const std::vector<unsigned int> istate, const std::vector<int> gstate) {

    unsigned int n_var = domain -> get_nvar();
    const HPLUS_variable* variables = domain -> get_variables();

    #if INTCHECKS
    for (int i = 0; i < n_var; i++) {
        if (istate[i] >= variables[i].get_range()) domain -> get_logger() -> raise_error("Variable %d in the initial state has value %d but range %d.", i, istate[i], variables[i].get_range());
        if (gstate[i] >= (int) variables[i].get_range()) domain -> get_logger() -> raise_error("Variable %d in the goal state has value %d but range %d.", i, gstate[i], variables[i].get_range());
    }
    #endif

    this -> domain = domain;
    unsigned int bf_size = this -> domain -> get_bfsize();
    this -> initial_state = new BitField(bf_size);
    this -> goal_state = new BitField(bf_size);

    for (int i = 0, c=0; i < n_var; i++, c += variables[i].get_range()) {
        this -> initial_state -> set(c + istate[i]);
        if (gstate[i] >= 0) this -> goal_state -> set(c + gstate[i]);
    }

    this -> best_nact = INT_MAX;
    this -> best_cost = UINT_MAX;

    this -> domain -> get_logger() -> print_info("Created HPLUS_problem for this instance.");

}

BitField HPLUS_problem::get_istate() const { return *this -> initial_state; }

BitField HPLUS_problem::get_gstate() const { return *this -> goal_state; }

//TODO: If needed make threadsafe
void HPLUS_problem::update_best_solution(const std::vector<unsigned int> solution, const unsigned int nact, const unsigned int cost) {
    
    /*#if INTCHECKS
    int n_act = this -> domain -> get_nact();
    int* dbcheck = new int[n_act]();
    const std::vector<HPLUS_action>* actions = this -> domain -> get_actions();
    unsigned int costcheck = 0;
    if (nact > n_act) this -> domain -> get_logger() -> raise_error("Proposed solution considers %d actions, while only %d actions exist.", nact, n_act);
    for (int i = 0; i < nact; i++) {
        if (solution[i] >= n_act) this -> domain -> get_logger() -> raise_error("Proposed solution has in %d position action %d, while only %d actions exist.", i, solution[i], n_act);
        if (dbcheck[solution[i]]) this -> domain -> get_logger() -> raise_error("Proposed solution has multiple occurrences of action %d.", solution[i]);
        dbcheck[solution[i]] = 1;
        costcheck += (*actions)[solution[i]].get_cost();
    }
    if (costcheck != cost) this -> domain -> get_logger() -> raise_error("Proposed solution with cost %d but suggested cost is %d.", cost, costcheck);
    delete[] dbcheck; dbcheck = NULL;
    #endif

    if (cost >= this -> best_cost) return;

    if (nact > this -> best_nact) this -> best_solution = std::vector<unsigned int>(solution);
    this -> best_nact = nact;
    this -> best_cost = cost;

    this -> domain -> get_logger() -> print_info("Updated best solution - Cost: %4d.", this -> best_cost);*/

}

void HPLUS_problem::get_best_solution(std::vector<unsigned int>* solution, unsigned int* nact, unsigned int* cost) const {

    *solution = std::vector<unsigned int>(this -> best_solution);
    *nact = this -> best_nact;
    *cost = this -> best_cost;

}

unsigned int HPLUS_problem::get_best_cost() const { return this -> best_cost; }