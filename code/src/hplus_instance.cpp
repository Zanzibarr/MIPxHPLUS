#include "../headers/hplus_instance.hpp"

HPLUS_action::HPLUS_action(unsigned int size_bf) {
    
    this -> pre = new BitField(size_bf);
    this -> eff = new BitField(size_bf);
    this -> cost = INFINITY;

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
    
}

HPLUS_action::~HPLUS_action() {
    
    delete this -> pre; this -> pre = NULL;
    delete this -> eff; this -> eff = NULL;

}

HPLUS_domain::HPLUS_domain(const std::string file_path, Logger* logger) {
    
    this -> logger = logger;

    FILE* file = fopen(file_path.c_str(), "r");
    parse_inst_file(file);
    fclose(file);

    this -> logger -> print_info("Created HPLUS_domain.");

}

HPLUS_domain::HPLUS_domain(const HPLUS_domain* from_dom) {
    
    this -> logger = from_dom -> logger;
    this -> use_costs = from_dom -> use_costs;
    this -> n_var = from_dom -> n_var;
    this -> var_ranges = new int[this -> n_var];
    for (int i = 0; i < this -> n_var; i++) (this -> var_ranges)[i] = (from_dom -> var_ranges)[i];
    this -> bf_size = from_dom -> bf_size;
    this -> n_act = from_dom -> n_act;
    this -> actions = new HPLUS_action[n_act];
    for (int i = 0; i < this -> n_act; i++) (this -> actions)[i] = HPLUS_action((from_dom -> actions)[i]);

    this -> logger -> print_info("Created copy HPLUS_domain.");

}

HPLUS_domain::~HPLUS_domain() {
    
    delete[] this -> var_ranges; this -> var_ranges = NULL;
    delete[] this -> actions; this -> actions = NULL;

}

bool HPLUS_domain::unitary_cost() const { return !(this -> use_costs); }

int HPLUS_domain::get_nvar() const { return this -> n_var; }

const int* HPLUS_domain::get_var_ranges() const { return this -> var_ranges; }

int HPLUS_domain::get_bfsize() const { return this -> bf_size; }

int HPLUS_domain::get_nact() const { return this -> n_act; }

const HPLUS_action* HPLUS_domain::get_actions() const { return this -> actions; }

const Logger* HPLUS_domain::get_logger() const { return this -> logger; }

void HPLUS_domain::parse_inst_file(const FILE* file) {

    //TODO

    this -> logger -> print_info("Parsed SAS file.");
    
}

HPLUS_problem::HPLUS_problem(HPLUS_domain* domain, const unsigned int* istate, const unsigned int* gstate) {

    const int* var_ranges = domain -> get_var_ranges();
    #if INTCHECKS
    for (int i = 0; i < domain -> get_nvar(); i++) {
        if (istate[i] >= var_ranges[i]) domain -> get_logger() -> raise_error("Variable %d in the initial state has value %d but range %d.", i, istate[i], var_ranges[i]);
        if (gstate[i] >= var_ranges[i]) domain -> get_logger() -> raise_error("Variable %d in the goal state has value %d but range %d.", i, gstate[i], var_ranges[i]);
    }
    #endif

    this -> domain = domain;
    int bf_size = this -> domain -> get_bfsize();
    this -> initial_state = new BitField(bf_size);
    this -> goal_state = new BitField(bf_size);
    int n_var = this -> domain -> get_nvar();

    for (int i = 0, c=0; i < n_var; i++) {
        this -> initial_state -> set(c + istate[i]);
        this -> goal_state -> set(c + gstate[i]);
        c += var_ranges[i];
    }

    this -> best_solution = NULL;
    this -> best_nact = INFINITY;
    this -> best_cost = INFINITY;

    this -> domain -> get_logger() -> print_info("Created HPLUS_problem for this instance.");

}

HPLUS_problem::~HPLUS_problem() {
    
    delete this -> initial_state; this -> initial_state = NULL;
    delete this -> goal_state; this -> goal_state = NULL;
    delete this -> best_solution; this -> best_solution = NULL;

}

const BitField* HPLUS_problem::get_istate() const { return this -> initial_state; }

const BitField* HPLUS_problem::get_gstate() const { return this -> goal_state; }

void HPLUS_problem::update_best_solution(const int* solution, const int* nact, const int* cost) {

    //TODO

    this -> domain -> get_logger() -> print_info("Updated best solution - Cost: %4d.", this -> best_cost);

}

void HPLUS_problem::get_best_solution(int* solution, int* nact, int* cost) const {

    for (int i = 0; i < this -> best_nact; i++) solution[i] = (this -> best_solution)[i];
    *nact = this -> best_nact;
    *cost = this -> best_cost;

}

int HPLUS_problem::get_best_cost() const { return this -> best_cost; }