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

}

HPLUS_domain::HPLUS_domain(const HPLUS_domain* from_dom) {
    
    this -> logger = from_dom -> logger;
    this -> use_costs = from_dom -> use_costs;
    this -> n_var = from_dom -> n_var;
    this -> var_ranges = new int[this -> n_var];
    for (int i = 0; i < this -> n_var; i++) (this -> var_ranges)[i] = (from_dom -> var_ranges)[i];
    this -> n_act = from_dom -> n_act;
    this -> actions = new HPLUS_action*[n_act];
    for (int i = 0; i < this -> n_act; i++) (this -> actions)[i] = new HPLUS_action((from_dom -> actions)[i]);

}

HPLUS_domain::~HPLUS_domain() {
    
    delete[] this -> var_ranges; this -> var_ranges = NULL;
    for (int i = 0; i < this -> n_act; i++) delete (this -> actions)[i];
    delete[] this -> actions;

}

void HPLUS_domain::parse_inst_file(const FILE* file) {

    //TODO
    
}

HPLUS_problem::HPLUS_problem(const HPLUS_domain* domain, const int* istate, const int* gstate) {
    
    //TODO
    this -> initial_state = new BitField(10);
    this -> goal_state = new BitField(10);

}

HPLUS_problem::~HPLUS_problem() {
    
    delete this -> initial_state; this -> initial_state = NULL;
    delete this -> goal_state; this -> goal_state = NULL;
    delete this -> best_solution; this -> best_solution = NULL;

}