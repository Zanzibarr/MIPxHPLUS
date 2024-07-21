#include "../include/hplus_instance.hpp"

HPLUS_action::HPLUS_action(const BitField* pre_bf, const BitField* eff_bf, const unsigned int cost, const std::string name) {

    unsigned int size = pre_bf -> size();
    
    this -> pre = new BitField(size);
    this -> eff = new BitField(size);
    for (int i = 0; i < size; i++) {
        if ((*pre_bf)[i]) this -> pre -> set(i);
        if ((*eff_bf)[i]) this -> eff -> set(i);
    }
    this -> cost = cost;
    this -> name = name;

}

HPLUS_action::HPLUS_action(const HPLUS_action* from_action) {

    int size = from_action -> pre -> size();

    this -> pre = new BitField(size);
    this -> eff = new BitField(size);

    for (int i = 0; i < size; i++) { 
        if ((*(from_action -> pre))[i]) this -> pre -> set(i);
        if ((*(from_action -> eff))[i]) this -> eff -> set(i);
    }

    this -> cost = from_action -> cost;
    this -> name = from_action -> name;
    
}

HPLUS_action::~HPLUS_action() {
    
    delete this -> pre; this -> pre = NULL;
    delete this -> eff; this -> eff = NULL;

}

const BitField* HPLUS_action::get_pre() const { return this -> pre; }

const BitField* HPLUS_action::get_eff() const { return this -> eff; }

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

HPLUS_domain::HPLUS_domain(const HPLUS_domain* from_dom) {
    
    this -> logger = from_dom -> logger;
    this -> version = from_dom -> version;
    this -> use_costs = from_dom -> use_costs;
    this -> n_var = from_dom -> n_var;
    this -> var_names = new std::string[this -> n_var];
    for (int i = 0; i < this -> n_var; i++) this -> var_names[i] = from_dom -> var_names[i];
    this -> var_ranges = new unsigned int[this -> n_var];
    for (int i = 0; i < this -> n_var; i++) this -> var_ranges[i] = from_dom -> var_ranges[i];
    this -> var_values_names = new std::string*[this -> n_var];
    for (int i = 0; i < this -> n_var; i++) {
        this -> var_values_names[i] = new std::string[this -> var_ranges[i]];
        for (int j = 0; j < this -> var_ranges[i]; j++) this -> var_values_names[i][j] = from_dom -> var_values_names[i][j];
    }
    this -> bf_size = from_dom -> bf_size;
    this -> n_act = from_dom -> n_act;
    this -> actions = new HPLUS_action[n_act];
    for (int i = 0; i < this -> n_act; i++) this -> actions[i] = HPLUS_action(from_dom -> actions[i]);

    this -> logger -> print_info("Created copy HPLUS_domain.");

}

HPLUS_domain::~HPLUS_domain() {
    
    delete[] this -> var_names; this -> var_names = NULL;
    delete[] this -> var_ranges; this -> var_ranges = NULL;
    for (int i = 0; i < this -> n_var; i++) { delete[] this -> var_values_names[i]; }
    delete[] this -> var_values_names; this -> var_values_names = NULL;
    delete[] this -> actions; this -> actions = NULL;

}

bool HPLUS_domain::unitary_cost() const { return !this -> use_costs; }

unsigned int HPLUS_domain::get_nvar() const { return this -> n_var; }

const unsigned int* HPLUS_domain::get_var_ranges() const { return this -> var_ranges; }

unsigned int HPLUS_domain::get_bfsize() const { return this -> bf_size; }

unsigned int HPLUS_domain::get_nact() const { return this -> n_act; }

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
    this -> var_names = new std::string[this -> n_var];
    this -> var_ranges = new unsigned int[this -> n_var];
    this -> var_values_names = new std::string*[this -> n_var];
    this -> bf_size = 0;
    for (int var_i = 0; var_i < this -> n_var; var_i++) {
        // process each variable
        std::getline(*ifs, line);   // begin_variable
        my::asserteq(line, "begin_variable", this -> logger);
        std::getline(*ifs, this -> var_names[var_i]);   // variable name
        std::getline(*ifs, line);   // axiom layer (ignored)
        if (line != "-1") this -> logger -> print_warn("Axiom layer is %s.", line.c_str());
        std::getline(*ifs, line);   // range of variable
        my::assertisint(line, this -> logger, 0);
        this -> var_ranges[var_i] = (unsigned int) stoi(line);
        this -> bf_size += this -> var_ranges[var_i];
        this -> var_values_names[var_i] = new std::string[this -> var_ranges[var_i]];
        for (int j = 0; j < this -> var_ranges[var_i]; j++) {
            std::getline(*ifs, line);   // name for variable value
            this -> var_values_names[var_i][j] = line;
        }
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
        while (line != "end_mutex_group") std::getline(*ifs, line); // reach end_mutex_group (ignore all content)
    }

    // initial state section
    std::getline(*ifs, line);   // begin_state
    my::asserteq(line, "begin_state", this -> logger);
    unsigned int* istate = new unsigned int[this -> n_var];
    for (int var_i = 0; var_i < this -> n_var; var_i++) {
        std::getline(*ifs, line);   // initial value of var_i
        my::assertisint(line, this -> logger, 0, this -> var_ranges[var_i] - 1);
        istate[var_i] = stoi(line);
    }
    std::getline(*ifs, line);   // end_state
    my::asserteq(line, "end_state", this -> logger);

    // goal state section
    std::getline(*ifs, line);   // begin_goal
    my::asserteq(line, "begin_goal", this -> logger);
    int* gstate = new int[this -> n_var];
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
        my::assertisint(tokens[1], this -> logger, 0, this -> var_ranges[var] - 1); // variable goal
        value = stoi(tokens[1]);
        gstate[var] = value;
    }
    std::getline(*ifs, line);   // end_goal
    my::asserteq(line, "end_goal", this -> logger);
    if (problem != NULL) delete problem;
    problem = new HPLUS_problem(this, istate, gstate);
    delete[] istate; istate = NULL;
    delete[] gstate; gstate = NULL;

    // operator (actions) section
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
        int* pre_state = new int[this -> n_var];
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
            my::assertisint(tokens[1], this -> logger, 0, this -> var_ranges[var] - 1); // variable value
            value = stoi(tokens[1]);
            pre_state[var] = value;
        }
        std::getline(*ifs, line);   // number of effects
        my::assertisint(line, this -> logger, 0);
        unsigned int n_eff = (unsigned int) stoi(line);
        int* eff_state = new int[this -> n_var];
        for (int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(*ifs, line);   // effect line
            std::vector<std::string> tokens;
            my::split(line, &tokens, ' ');
            my::asserteq(tokens.size(), 4, this -> logger); // not expecting effect conditions
            my::assertisint(tokens[0], this -> logger, 0, 0);   // number of effect conditions (ignored and check to be 0)
            my::assertisint(tokens[1], this -> logger, 0, this -> n_var - 1);   // variable affected by the action
            unsigned int var = (unsigned int) stoi(tokens[1]);
            my::assertisint(tokens[2], this -> logger, -1, this -> var_ranges[var] - 1);    // pre_val of the variable
            int pre_val = stoi(tokens[2]);
            my::assertisint(tokens[3], this -> logger, 0, this -> var_ranges[var] - 1);     // eff_val of the variable
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
        BitField* pre = new BitField(this -> bf_size);
        BitField* eff = new BitField(this -> bf_size);
        for (int i = 0, c = 0; i < this -> n_var; i++, c += this -> var_ranges[i]) {
            if (pre_state[i] >= 0) pre -> set(c + pre_state[i]);
            if (eff_state[i] >= 0) eff -> set(c + eff_state[i]);
        }
        delete[] pre_state; pre_state = NULL;
        delete[] eff_state; eff_state = NULL;
        //TODO: This works fine
        this -> logger -> print_info("PRE='%s'.", pre -> view().c_str());
        this -> logger -> print_info("EFF='%s'.", eff -> view().c_str());
        this -> actions[act_i] = HPLUS_action(pre, eff, cost, name);
        //TODO: This segfaults...
        this -> logger -> print_info("PRE='%s'.", this -> actions[act_i].get_pre() -> view().c_str());
        this -> logger -> print_info("EFF='%s'.", this -> actions[act_i].get_eff() -> view().c_str());
        delete pre; pre = NULL;
        delete eff; eff = NULL;
    }

    #if VERBOSE >= 100

    this -> logger -> print_info("Version: %d.", this -> version);
    this -> logger -> print_info("Metric: %d.", this -> use_costs);
    this -> logger -> print_info("N_var: %d.", this -> n_var);
    for (int i = 0; i < this -> n_var; i++) this -> logger -> print_info("name(var_%d) = '%s'.", i, this -> var_names[i].c_str());
    for (int i = 0; i < this -> n_var; i++) this -> logger -> print_info("range(var_%d) = '%d'.", i, this -> var_ranges[i]);
    for (int i = 0; i < this -> n_var; i++) for (int j = 0; j < this -> var_ranges[i]; j++)
        this -> logger -> print_info("name(var_%d[%d]) = '%s'.", i, j, this -> var_values_names[i][j].c_str());
    this -> logger -> print_info("Bitfield size: %d", this -> bf_size);
    this -> logger -> print_info("Initial state: %s.", problem -> get_istate() -> view().c_str());
    this -> logger -> print_info("Goal state: %s.", problem -> get_gstate() -> view().c_str());
    this -> logger -> print_info("N_act: %d.", this -> n_act);
    for (int act_i = 0; act_i < this -> n_act; act_i++) {
        //TODO: This segfaults too
        this -> logger -> print_info("pre(act_%d) = '%s'.", act_i, this -> actions[act_i].get_pre() -> view().c_str());
        this -> logger -> print_info("eff(act_%d) = '%s'.", act_i, this -> actions[act_i].get_eff() -> view().c_str());
        this -> logger -> print_info("name(act_%d) = '%s'.", act_i, this -> actions[act_i].get_name().c_str());
        this -> logger -> print_info("cost(act_%d) = '%d'.", act_i, this -> actions[act_i].get_cost());
    }

    #endif

    this -> logger -> print_info("Parsed SAS file.");
    
}

HPLUS_problem::HPLUS_problem(HPLUS_domain* domain, const unsigned int* istate, const int* gstate) {

    const unsigned int* var_ranges = domain -> get_var_ranges();
    #if INTCHECKS
    for (int i = 0; i < domain -> get_nvar(); i++) {
        if (istate[i] >= var_ranges[i]) domain -> get_logger() -> raise_error("Variable %d in the initial state has value %d but range %d.", i, istate[i], var_ranges[i]);
        if (gstate[i] >= (int) var_ranges[i]) domain -> get_logger() -> raise_error("Variable %d in the goal state has value %d but range %d.", i, gstate[i], var_ranges[i]);
    }
    #endif

    this -> domain = domain;
    int bf_size = this -> domain -> get_bfsize();
    this -> initial_state = new BitField(bf_size);
    this -> goal_state = new BitField(bf_size);
    int n_var = this -> domain -> get_nvar();

    for (int i = 0, c=0; i < n_var; i++, c += var_ranges[i]) {
        this -> initial_state -> set(c + istate[i]);
        if (gstate[i] >= 0) this -> goal_state -> set(c + gstate[i]);
    }

    this -> best_solution = NULL;
    this -> best_nact = INT_MAX;
    this -> best_cost = UINT_MAX;

    this -> domain -> get_logger() -> print_info("Created HPLUS_problem for this instance.");

}

HPLUS_problem::~HPLUS_problem() {
    
    delete this -> initial_state; this -> initial_state = NULL;
    delete this -> goal_state; this -> goal_state = NULL;
    delete[] this -> best_solution; this -> best_solution = NULL;

}

const BitField* HPLUS_problem::get_istate() const { return this -> initial_state; }

const BitField* HPLUS_problem::get_gstate() const { return this -> goal_state; }

//TODO: If needed make threadsafe
void HPLUS_problem::update_best_solution(const unsigned int* solution, const unsigned int nact, const unsigned int cost) {
    
    #if INTCHECKS
    int n_act = this -> domain -> get_nact();
    int* dbcheck = new int[n_act]();
    const HPLUS_action* actions = this -> domain -> get_actions();
    unsigned int costcheck = 0;
    if (nact > n_act) this -> domain -> get_logger() -> raise_error("Proposed solution considers %d actions, while only %d actions exist.", nact, n_act);
    for (int i = 0; i < nact; i++) {
        if (solution[i] >= n_act) this -> domain -> get_logger() -> raise_error("Proposed solution has in %d position action %d, while only %d actions exist.", i, solution[i], n_act);
        if (dbcheck[solution[i]]) this -> domain -> get_logger() -> raise_error("Proposed solution has multiple occurrences of action %d.", solution[i]);
        dbcheck[solution[i]] = 1;
        costcheck += actions[solution[i]].get_cost();
    }
    if (costcheck != cost) this -> domain -> get_logger() -> raise_error("Proposed solution with cost %d but suggested cost is %d.", cost, costcheck);
    delete[] actions; actions = NULL;
    delete[] dbcheck; dbcheck = NULL;
    #endif

    if (cost >= this -> best_cost) return;

    if (nact > this -> best_nact) {
        delete[] this -> best_solution;
        this -> best_solution = new unsigned int[nact];
    }
    for (int i = 0; i < nact; i++) this -> best_solution[i] = solution[i];
    this -> best_nact = nact;
    this -> best_cost = cost;

    this -> domain -> get_logger() -> print_info("Updated best solution - Cost: %4d.", this -> best_cost);

}

void HPLUS_problem::get_best_solution(unsigned int* solution, unsigned int* nact, unsigned int* cost) const {

    for (int i = 0; i < this -> best_nact; i++) solution[i] = this -> best_solution[i];
    *nact = this -> best_nact;
    *cost = this -> best_cost;

}

unsigned int HPLUS_problem::get_best_cost() const { return this -> best_cost; }