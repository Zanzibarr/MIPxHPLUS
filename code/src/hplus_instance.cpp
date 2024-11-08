#include "../include/hplus_instance.hpp"

HPLUS_instance HPLUS_inst;

// ##################################################################### //
// ########################### HPLUS_INSTANCE ########################## //
// ##################################################################### //

HPLUS_instance::HPLUS_instance(const std::string& file_path) {

    this -> version_ = 0;
    this -> use_costs_ = false;
    this -> n_act_ = 0;
    this -> n_var_ = 0;

    std::ifstream file(file_path.c_str(), std::ifstream::in);
    my::assert(file.good(), "Opening instance file failed.");
    parse_inst_file_(&file);
    file.close();

    this -> best_nact_ = 0;
    this -> best_cost_ = UINT_MAX;

    this -> model_opt_eliminated_var = my::BitField(this -> n_var_);
    this -> model_opt_fixed_var = my::BitField(this -> n_var_);
    this -> model_opt_eliminated_act = my::BitField(this -> n_act_);
    this -> model_opt_fixed_act = my::BitField(this -> n_act_);
    this -> model_opt_eliminated_fa = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> n_var_));
    this -> model_opt_fixed_fa = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> n_var_));
    this -> model_opt_timestamps_var = std::vector<int>(this -> n_var_, -1);
    this -> model_opt_timestamps_act = std::vector<int>(this -> n_act_, -1);

    this -> model_opt_cpxidx_to_actidx = std::vector<unsigned int>(this -> n_act_);
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) this -> model_opt_cpxidx_to_actidx[act_i] = act_i;

    pthread_mutex_init(&(this -> solution_read_write), nullptr);

    lprint_info("Created HPLUS_instance.");

}

int HPLUS_instance::get_version() const { return this -> version_; }

bool HPLUS_instance::unitary_cost() const { return !this -> use_costs_; }

unsigned int HPLUS_instance::get_nact() const { return this -> n_act_; }

unsigned int HPLUS_instance::get_nvar() const { return this -> n_var_; }

const std::vector<HPLUS_action>& HPLUS_instance::get_actions() const { return this -> actions_; }

const my::BitField& HPLUS_instance::get_gstate() const { return this -> goal_state_; }

void HPLUS_instance::update_best_solution(const std::vector<unsigned int>& solution, const unsigned int cost) {

    #if HPLUS_INTCHECK
    my::BitField dbcheck = my::BitField(this -> n_act_);
    unsigned int costcheck = 0;
    my::assert(solution.size() <= this -> n_act_, "Solution has more actions that there actually exists.");             // check that there aren't more actions that there exists
    my::BitField feas_checker(this -> n_var_);
    for (auto act_i : solution) {
        my::assert(act_i < this -> n_act_, "Solution contains unexisting action.");                                     // check that the solution only contains existing actions
        my::assert(!dbcheck[act_i], "Solution contains duplicate action.");                                             // check that there are no duplicates
        dbcheck.set(act_i);
        my::assert(feas_checker.contains(this -> actions_[act_i].get_pre()), "Preconditions are not respected.");       // check if the preconditions are respected at each step
        feas_checker |= this -> actions_[act_i].get_eff();
        costcheck += this -> actions_[act_i].get_cost();
    }
    my::assert(feas_checker.contains(this -> goal_state_), "The solution doesn't lead to the final state.");            // check if the solution leads to the goal state
    my::assert(costcheck == cost, "Declared cost is different from calculated one.");                                   // check if the cost is the declared one
    #endif

    pthread_mutex_lock(&(this -> solution_read_write));

    if (cost >= this -> best_cost_) return;

    this -> best_solution_ = std::vector<unsigned int>(solution);
    this -> best_nact_ = solution.size();
    this -> best_cost_ = cost;

    HPLUS_env.logger.print_info("Updated best solution - Cost: %4d.", this -> best_cost_);

    pthread_mutex_unlock(&(this -> solution_read_write));

}

void HPLUS_instance::get_best_solution(std::vector<unsigned int>& solution, unsigned int& cost) {

    pthread_mutex_lock(&(this -> solution_read_write));

    solution = std::vector<unsigned int>(this -> best_solution_);
    cost = this -> best_cost_;

    pthread_mutex_unlock(&(this -> solution_read_write));

}

void HPLUS_instance::print_best_sol() {

    pthread_mutex_lock(&(this -> solution_read_write));
    
    HPLUS_env.logger.print("Solution cost: %d", this -> best_cost_);
    for(auto act_i : this -> best_solution_) HPLUS_env.logger.print("(%s)", this -> actions_[act_i].get_name().c_str());

    pthread_mutex_unlock(&(this -> solution_read_write));

}

void HPLUS_instance::initial_heuristic() {

    HPLUS_stats.wstart_time = HPLUS_env.time_limit - HPLUS_env.get_time();
    double start_time = HPLUS_env.get_time();

    //[ ]: TODO
    // HPLUS_env.status = my::status::FEAS;

    HPLUS_stats.wstart_time = HPLUS_env.get_time() - start_time;
    
}

void HPLUS_instance::model_optimization() {

    HPLUS_stats.opt_time = HPLUS_env.time_limit - HPLUS_env.get_time();
    double start_time = HPLUS_env.get_time();

    HPLUS_inst.imai_model_enhancements();

    HPLUS_stats.opt_time = HPLUS_env.get_time();
    
}

void HPLUS_instance::optimized_heuristic() {

    double backup_swtart_time = HPLUS_stats.wstart_time;
    HPLUS_stats.wstart_time += HPLUS_env.time_limit - HPLUS_env.get_time();
    double start_time = HPLUS_env.get_time();

    //[ ]: TODO

    HPLUS_stats.wstart_time = backup_swtart_time + HPLUS_env.get_time() - start_time;

}

unsigned int HPLUS_instance::get_nvar_opt() const { return this -> model_opt_nvar; }

unsigned int HPLUS_instance::get_nact_opt() const { return this -> model_opt_nact; }

const my::BitField& HPLUS_instance::get_model_opt_eliminated_var() const { return this -> model_opt_eliminated_var; }

const my::BitField& HPLUS_instance::get_model_opt_eliminated_act() const { return this -> model_opt_eliminated_act; }

unsigned int HPLUS_instance::cpxidx_to_actidx(unsigned int cpxidx) const { return this -> model_opt_cpxidx_to_actidx[cpxidx]; }

//[ ]: Optimize
void HPLUS_instance::landmarks_extraction(std::vector<my::BitField>& landmarks_set, my::BitField& fact_landmarks, my::BitField& act_landmarks) const {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Extracting landmarks.");
    #endif

    landmarks_set = std::vector<my::BitField>(this -> n_var_, my::BitField(this -> n_var_ + 1));
    fact_landmarks = my::BitField(this -> n_var_);
    act_landmarks = my::BitField(this -> n_act_);

    for (unsigned int var_i = 0; var_i < this -> n_var_; var_i++) landmarks_set[var_i].set(this -> n_var_);

    my::BitField s_set(this -> n_var_);

    std::deque<int> actions_queue;
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) if(s_set.contains(this -> actions_[act_i].get_pre())) actions_queue.push_back(act_i);

    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(this -> n_var_);
    for (unsigned int var_i = 0; var_i < this -> n_var_; var_i++) for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) if (this -> actions_[act_i].get_pre()[var_i]) act_with_pre[var_i].push_back(act_i);

    while(!actions_queue.empty()) {

        const HPLUS_action a = this -> actions_[actions_queue.front()];
        actions_queue.pop_front();

        const auto& pre_sparse = a.get_pre_sparse();
        const auto& add_sparse = a.get_eff_sparse();

        for (auto var_i : add_sparse) {

            s_set.set(var_i);

            my::BitField x = my::BitField(this -> n_var_ + 1);
            for (auto var_j : add_sparse) x.set(var_j);

            for (auto var_j : pre_sparse) {
                // if variable var_i' has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set the flag
                if (landmarks_set[var_j][this -> n_var_]) {
                    x.set(this -> n_var_);
                    // if x is now full we can exit, since all further unions won't change x
                    break;
                } else x |= landmarks_set[var_j];
            }

            // we then check if L[var_i] != X, and if they are the same we skip,
            // if X = P, then (X intersection L[P]) = L[P], hence we can already skip
            if (!x[this -> n_var_]) {

                // if the set for variable var_i is the full set of variables,
                // the intersection generates back x -> we can skip the intersection
                if (!landmarks_set[var_i][this -> n_var_]) x &= landmarks_set[var_i];

                // we already know that x is not the full set now, so if
                // the set for variable var_i is the full set, we know that x is not
                // equal to the set for variable var_i -> we can skip the check
                if (landmarks_set[var_i][this -> n_var_] || x != landmarks_set[var_i]) {

                    landmarks_set[var_i] = x;
                    for (auto act_i : act_with_pre[var_i])
                        if (s_set.contains(this -> actions_[act_i].get_pre()) && std::find(actions_queue.begin(), actions_queue.end(), act_i) == actions_queue.end())
                            actions_queue.push_back(act_i);

                }

            }

        }

    }

    // list of actions that have as effect variable p
    std::vector<std::vector<unsigned int>> act_with_eff(this -> n_var_);
    for (auto var_i : !fact_landmarks) for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) if (this -> actions_[act_i].get_eff()[var_i]) act_with_eff[var_i].push_back(act_i);

    // popolate fact_landmarks and act_landmarks sets
    for (auto var_i : this -> goal_state_) {
        for (auto var_j : !fact_landmarks) {
            if (landmarks_set[var_i][var_j] || landmarks_set[var_i][this -> n_var_]) {
                fact_landmarks.set(var_j);
                unsigned int count = 0, cand_act;
                for (auto act_i : act_with_eff[var_j]) {
                    cand_act = act_i;
                    count++;
                    if (count > 1) break;
                }
                if (count == 1) act_landmarks.set(cand_act);
            }
        }
    }

}

void HPLUS_instance::first_adders_extraction(const std::vector<my::BitField>& landmarks_set, std::vector<my::BitField>& fadd) const {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Extracting first adders.");
    #endif

    fadd = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> n_var_));

    // list of fact landmarks for each variable
    std::vector<std::vector<unsigned int>> var_flm_sparse(this -> n_var_);
    for (unsigned int var_i = 0; var_i < this -> n_var_; var_i++) for (unsigned int var_j = 0; var_j < this -> n_var_; var_j++) if (landmarks_set[var_i][var_j] || landmarks_set[var_i][this -> n_var_]) var_flm_sparse[var_i].push_back(var_j);

    // compute the set of first adders
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        // f_lm_a is the set of fact landmarks of action act_i
        my::BitField f_lm_a(this -> n_var_);
        for (auto var_i : this -> actions_[act_i].get_pre()) for (auto i : var_flm_sparse[var_i]) f_lm_a.set(i);
        // fadd[a] := { p in add(a) s.t. p is not a fact landmark for a }
        fadd[act_i] |= (this -> actions_[act_i].get_eff() & !f_lm_a);
    }

}

void HPLUS_instance::relevance_analysis(const my::BitField& fact_landmarks, const std::vector<my::BitField>& fadd) {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Relevance analysis.");
    #endif

    my::BitField relevant_variables = my::BitField(this -> n_var_);
    my::BitField relevant_actions = my::BitField(this -> n_act_);

    // compute first round of relevand variables and actions
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        if (fadd[act_i].intersects(this -> goal_state_)) {
            relevant_variables |= this -> actions_[act_i].get_pre();
            relevant_actions.set(act_i);
        }
    }

    // list of actions yet to check
    auto cand_actions_sparse = (!relevant_actions).sparse();

    // keep looking for other relevant actions/variables until no more can be found
    bool new_act = true;
    while (new_act) {
        new_act = false;
        std::vector<unsigned int> new_relevant_actions;
        for (auto act_i : cand_actions_sparse) {
            if (this -> actions_[act_i].get_eff().intersects(relevant_variables)) {
                relevant_actions.set(act_i);
                relevant_variables |= this -> actions_[act_i].get_pre();
                new_relevant_actions.push_back(act_i);
                new_act = true;
            }
        }
        auto it = std::set_difference(cand_actions_sparse.begin(), cand_actions_sparse.end(), new_relevant_actions.begin(), new_relevant_actions.end(), cand_actions_sparse.begin());
        cand_actions_sparse.resize(it - cand_actions_sparse.begin());
    }
    relevant_variables |= this -> goal_state_;

    // eliminate actions and variables that are not relevant (or landmarks)
    this -> model_opt_eliminated_var |= (!relevant_variables - fact_landmarks);
    this -> model_opt_eliminated_act |= !relevant_actions;

}

void HPLUS_instance::dominated_actions_elimination(const std::vector<my::BitField>& landmarks_set, const std::vector<my::BitField>& fadd) {
    
    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Extracting dominated actions.");
    #endif

    auto remaining_var_sparse = (!this -> model_opt_eliminated_var).sparse();
    std::vector<my::BitField> act_flm = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> n_var_));
    std::vector<std::vector<unsigned int>> var_flm_sparse(this -> n_var_);

    // compute the landmarks for each variable remaining
    for (auto var_i : remaining_var_sparse)
        for (auto var_j : remaining_var_sparse)
            if (landmarks_set[var_i][var_j] || landmarks_set[var_i][this -> n_var_])
                var_flm_sparse[var_i].push_back(var_j);

    // compute the landmarks for each action remaining
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++)
        for (auto var_i : this -> actions_[act_i].get_pre_sparse()) if (!this -> model_opt_eliminated_var[var_i])
            for (auto i : var_flm_sparse[var_i])
                act_flm[act_i].set(i);

    // find efficently all actions that satisfy point 1) of Proposition 4 of in Imai's Paper
    my::SSBT subset_finder = my::SSBT();
    for (auto act_i : !this -> model_opt_eliminated_act) subset_finder.add(act_i, fadd[act_i]);

    my::BitField dominated_actions(this -> n_act_);

    // find all dominated actions and eliminate them
    for (auto dominant_act : !this -> model_opt_eliminated_act) if (!dominated_actions[dominant_act]) {
        for (auto dominated_act : subset_finder.find_subsets(fadd[dominant_act])) if (!this -> model_opt_eliminated_act[dominated_act]) {
            
            if (this -> model_opt_fixed_act[dominated_act] || dominant_act == dominated_act || this -> actions_[dominant_act].get_cost() > this -> actions_[dominated_act].get_cost() || !act_flm[dominated_act].contains(this -> actions_[dominant_act].get_pre())) continue;

            dominated_actions.set(dominated_act);
            this -> model_opt_eliminated_act.set(dominated_act);

        }

    }

}

void HPLUS_instance::immediate_action_application(const my::BitField& act_landmarks) {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Immediate action application.");
    #endif

    my::BitField current_state(this -> n_var_);
    my::BitField actions_left = !this -> model_opt_eliminated_act;

    // keep looking until no more actions can be applied
    int counter = 0;
    bool found_next_action = true;
    while (found_next_action) {

        found_next_action = false;

        for (auto act_i : actions_left) {

            const auto& pre = this -> actions_[act_i].get_pre() & !this -> model_opt_eliminated_var;
            const auto& eff = this -> actions_[act_i].get_eff() & !this -> model_opt_eliminated_var;

            if (current_state.contains(pre) && (act_landmarks[act_i] || this -> actions_[act_i].get_cost() == 0)) {

                actions_left.unset(act_i);
                this -> model_opt_fixed_act.set(act_i);
                if (HPLUS_env.alg == HPLUS_CLI_IMAI) this -> model_opt_timestamps_act[act_i] = counter;
                this -> model_opt_fixed_var |= pre;
                for (auto var_i : eff) if (!current_state[var_i]) {
                    this -> model_opt_fixed_var.set(var_i);
                    if (HPLUS_env.alg == HPLUS_CLI_IMAI) this -> model_opt_timestamps_var[var_i] = counter+1;
                    this -> model_opt_fixed_fa[act_i].set(var_i);
                    for (auto act_j : actions_left) this -> model_opt_eliminated_fa[act_j].set(var_i);
                }
                current_state |= eff;
                counter++;
                found_next_action = true;

            }

        }

    }

}

// void HPLUS_instance::inverse_actions_extraction(const my::BitField& eliminated_actions, const my::BitField& fixed_actions, std::map<unsigned int, std::vector<unsigned int>>& inverse_actions) const {

//     #if HPLUS_VERBOSE >= 20
//     lprint_info("(debug) Extracting inverse actions.");
//     #endif

//     my::SSBT subset_finder = my::SSBT();

//     // find efficiently all actions that satisfy point 2) of the Definition 1 in section 4.6 of Imai's paper
//     auto remaining_actions = (!eliminated_actions).sparse();
//     for (auto act_i : remaining_actions) subset_finder.add(act_i, this -> actions_[act_i].get_eff());

//     for (auto act_i : !eliminated_actions) {
//         const auto& pre = this -> actions_[act_i].get_pre();
//         const auto& eff = this -> actions_[act_i].get_eff();
//         for (auto act_j : subset_finder.find_subsets(pre)) {
//             if (this -> actions_[act_j].get_pre().contains(eff)) {
//                 if (!fixed_actions[act_i]) {
//                     inverse_actions.try_emplace(act_i, std::vector<unsigned int>());
//                     inverse_actions[act_i].push_back(act_j);
//                 }
//                 if (!fixed_actions[act_j]) {
//                     inverse_actions.try_emplace(act_j, std::vector<unsigned int>());
//                     inverse_actions[act_j].push_back(act_i);
//                 }
//             }
//         }
//     }

// }

void HPLUS_instance::imai_model_enhancements() {

    lprint_info("Model enhancement from Imai's paper.");

    std::vector<my::BitField> landmarks_set;
    my::BitField fact_landmarks;
    my::BitField act_landmarks;
    std::vector<my::BitField> fadd;

    this -> landmarks_extraction(landmarks_set, fact_landmarks, act_landmarks);

    this -> model_opt_fixed_var |= fact_landmarks;
    this -> model_opt_fixed_act |= act_landmarks;

    this -> first_adders_extraction(landmarks_set, fadd);

    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) this -> model_opt_eliminated_fa[act_i] |= (this -> actions_[act_i].get_eff() & !fadd[act_i]);

    this -> relevance_analysis(fact_landmarks, fadd);

    this -> dominated_actions_elimination(landmarks_set, fadd);

    this -> immediate_action_application(act_landmarks);

    //[ ]: This might not be worth doing (and might be wrong)
    // if (inverse_actions != nullptr) this -> inverse_actions_extraction(eliminated_actions, this -> fixed_actions, *inverse_actions);

    int count = 0;
    for (auto _ : this -> model_opt_eliminated_var) count++;
    this -> model_opt_nvar = this -> n_var_ - count;
    count = 0;
    for (auto _ : this -> model_opt_eliminated_act) count++;
    this -> model_opt_cpxidx_to_actidx = std::vector<unsigned int>();
    for (auto act_i : !this -> model_opt_eliminated_act) this -> model_opt_cpxidx_to_actidx.push_back(act_i);
    this -> model_opt_nact = this -> n_act_ - count;

    #if HPLUS_INTCHECK
    my::assert(!this -> model_opt_fixed_var.intersects(this -> model_opt_eliminated_var), "Eliminated and fixed variables intersect.");
    my::assert(!this -> model_opt_fixed_act.intersects(this -> model_opt_eliminated_act), "Eliminated and fixed actions intersect.");
    #endif

    #if HPLUS_VERBOSE >= 20
    count = 0;
    for (auto _ : HPLUS_inst.model_opt_fixed_var) count++;
    HPLUS_env.logger.print_info("Optimized number of variables:         %10d --> %10d.", HPLUS_inst.get_nvar(), HPLUS_inst.get_nvar_opt() - count);
    count = 0;
    for (auto _ : HPLUS_inst.model_opt_fixed_act) count++;
    HPLUS_env.logger.print_info("Optimized number of actions:           %10d --> %10d.", HPLUS_inst.get_nact(), HPLUS_inst.get_nact_opt() - count);
    count = 0;
    for (auto _ : HPLUS_inst.model_opt_eliminated_fa) count++;
    for (auto _ : HPLUS_inst.model_opt_fixed_fa) count++;
    HPLUS_env.logger.print_info("Optimized number of first archievers:  %10d --> %10d.", HPLUS_inst.get_nvar() * HPLUS_inst.get_nact(), HPLUS_inst.get_nact_opt() * HPLUS_inst.get_nvar_opt() - count);
    #endif

}

void HPLUS_instance::parse_inst_file_(std::ifstream* ifs) {

    lprint_info("Parsing SAS file.");

    HPLUS_stats.parsing_time = HPLUS_env.time_limit - HPLUS_env.get_time();
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
    lprint_warn("Ignoring axiom layers.");
    std::getline(*ifs, line);   // nvar_pre_exp
    my::assert(my::isint(line, 0), "Corrupted file.");
    unsigned int nvar_pre_exp = std::stoi(line);
    std::vector<unsigned int> var_ranges(nvar_pre_exp);
    this -> n_var_ = 0;
    for (unsigned int var_i = 0; var_i < nvar_pre_exp; var_i++) {
        // process each variable
        std::getline(*ifs, line);   // begin_variable
        my::assert(line == "begin_variable", "Corrupted file.");
        std::getline(*ifs, line);   // variable name (ignored)
        std::getline(*ifs, line);   // axiom layer (ignored)
        my::assert(line == "-1", "Axiom layer is not -1, this software is not made for this instance.");
        std::getline(*ifs, line);   // range of variable
        my::assert(my::isint(line, 0), "Corrupted file.");
        unsigned int range = stoi(line);
        var_ranges[var_i] = range;
        this -> n_var_ += range;
        for (unsigned int j = 0; j < range; j++) std::getline(*ifs, line);   // name for variable value (ignored)
        std::getline(*ifs, line);   // end_variable
        my::assert(line == "end_variable", "Corrupted file.");
    }
    
    // * mutex section (ignored)
    lprint_warn("Ignoring mutex section.");
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
    my::BitField istate = my::BitField(this -> n_var_);
    for (unsigned int var_i = 0, c = 0; var_i < nvar_pre_exp; c += var_ranges[var_i], var_i++) {
        std::getline(*ifs, line);   // initial value of var_i
        my::assert(my::isint(line, 0, var_ranges[var_i] - 1), "Corrupted file.");
        unsigned int val = stoi(line);
        istate.set(c + val);
    }
    std::getline(*ifs, line);   // end_state
    my::assert(line == "end_state", "Corrupted file.");

    // removing initial state variables
    std::vector<unsigned int> post_istate_removal_offset(this -> n_var_);
    int counter = 0;
    for (unsigned int var_i = 0; var_i < this -> n_var_; var_i++) {
        if (istate[var_i]) counter++;
        post_istate_removal_offset[var_i] = counter;
    }
    this -> n_var_ -= nvar_pre_exp;
    this -> model_opt_nvar = this -> n_var_;

    // * goal state section
    std::getline(*ifs, line);   // begin_goal
    my::assert(line == "begin_goal", "Corrupted file.");
    this -> goal_state_ = my::BitField(this -> n_var_);
    std::getline(*ifs, line);   // number of goals
    my::assert(my::isint(line, 0, nvar_pre_exp), "Corrupted file.");
    unsigned int ngoals = stoi(line);
    for (unsigned int _ = 0; _ < ngoals; _++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(*ifs, line);   // pair 'variable goal'
        tokens = my::split_string(line, ' ');
        my::assert(tokens.size() == 2, "Corrupted file.");
        my::assert(my::isint(tokens[0], 0, nvar_pre_exp - 1), "Corrupted file."); // variable index
        unsigned int var = stoi(tokens[0]);
        my::assert(my::isint(tokens[1], 0, var_ranges[var] - 1), "Corrupted file."); // variable goal
        unsigned int value = stoi(tokens[1]);
        unsigned int var_strips = value;
        for (unsigned int j = 0; j < var; var_strips +=var_ranges[j], j++) {}
        if (!istate[var_strips]) this -> goal_state_.set(var_strips - post_istate_removal_offset[var_strips]);
    }
    std::getline(*ifs, line);   // end_goal
    my::assert(line == "end_goal", "Corrupted file.");

    // * operator (actions) section
    lprint_warn("Ignoring effect conditions.");
    std::getline(*ifs, line);   // n_act
    my::assert(my::isint(line, 0), "Corrupted file.");
    this -> n_act_ = stoi(line);
    this -> model_opt_nact = this -> n_act_;
    this -> actions_ = std::vector<HPLUS_action>(this -> n_act_);
    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        // process each action
        std::getline(*ifs, line);   // begin_operator
        my::assert(line == "begin_operator", "Corrupted file.");
        std::getline(*ifs, line);   // symbolic action name
        std::string name = line;
        my::BitField act_pre(this -> n_var_);
        std::getline(*ifs, line);   // number of prevail conditions
        my::assert(my::isint(line, 0, nvar_pre_exp), "Corrupted file.");
        unsigned int n_pre = stoi(line);
        for (unsigned int pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(*ifs, line);   // pair 'variable value'
            tokens = my::split_string(line, ' ');
            my::assert(tokens.size() == 2, "Corrupted file.");
            my::assert(my::isint(tokens[0], 0, nvar_pre_exp - 1), "Corrupted file."); // variable index
            unsigned int var = stoi(tokens[0]);
            my::assert(my::isint(tokens[1], 0,var_ranges[var] - 1), "Corrupted file."); // variable value
            unsigned int value = stoi(tokens[1]);
            unsigned int var_strips = value;
            for (unsigned int i = 0; i < var; var_strips += var_ranges[i], i++) {}
            if (!istate[var_strips]) act_pre.set(var_strips - post_istate_removal_offset[var_strips]);
        }
        std::getline(*ifs, line);   // number of effects
        my::assert(my::isint(line, 0), "Corrupted file.");
        unsigned int n_eff = stoi(line);
        my::BitField act_eff(this -> n_var_);
        for (unsigned int eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(*ifs, line);   // effect line
            std::vector<std::string> tokens;
            tokens = my::split_string(line, ' ');
            my::assert(tokens.size() == 4, "This program won't handle effect conditions."); // not expecting effect conditions
            my::assert(my::isint(tokens[0], 0, 0), "This program won't handle effect conditions."); // number of effect conditions (ignored and check to be 0)
            my::assert(my::isint(tokens[1], 0, nvar_pre_exp - 1), "Corrupted file.");   // variable affected by the action
            unsigned int var = stoi(tokens[1]);
            my::assert(my::isint(tokens[2], -1, var_ranges[var] - 1), "Corrupted file.");    // precondition of the variable
            int pre_val = stoi(tokens[2]);
            my::assert(my::isint(tokens[3], 0, var_ranges[var] - 1), "Corrupted file."); // effect of the variable
            unsigned int eff_val = stoi(tokens[3]);
            unsigned int c = 0;
            for (unsigned int i = 0; i < var; c += var_ranges[i], i++){}
            if (pre_val >= 0 && !istate[c + pre_val]) act_pre.set(c + pre_val - post_istate_removal_offset[c + pre_val]);
            if (!istate[c + eff_val]) act_eff.set(c + eff_val - post_istate_removal_offset[c + eff_val]);
        }
        std::getline(*ifs, line);   // action cost
        my::assert(my::isint(line), "Corrupted file.");
        unsigned int cost = 1;
        if (this -> use_costs_) cost = stoi(line);
        std::getline(*ifs, line);   // end_operator
        my::assert(line == "end_operator", "Corrupted file.");
        this -> actions_[act_i] = HPLUS_action(act_pre, act_eff, cost, name);
    }

    lprint_warn("Ignoring axiom section.");

    HPLUS_stats.parsing_time = HPLUS_env.get_time() - start_time;
    
}

// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

HPLUS_action::HPLUS_action(my::BitField& pre_bf, my::BitField& eff_bf, const unsigned int cost, const std::string& name) {

    this -> pre_ = my::BitField(pre_bf);
    this -> eff_ = my::BitField(eff_bf);
    for (unsigned int var_i = 0; var_i < this -> pre_.size(); var_i++) {
        if (this -> pre_[var_i]) this -> sparse_pre_.push_back(var_i);
        if (this -> eff_[var_i]) this -> sparse_eff_.push_back(var_i);
    }
    this -> cost_ = cost;
    this -> name_ = name;

}

const my::BitField& HPLUS_action::get_pre() const { return this -> pre_; }

const std::vector<unsigned int>& HPLUS_action::get_pre_sparse() const { return this -> sparse_pre_; }

const my::BitField& HPLUS_action::get_eff() const { return this -> eff_; }

const std::vector<unsigned int>& HPLUS_action::get_eff_sparse() const { return this -> sparse_eff_; }

const std::string& HPLUS_action::get_name() const { return this -> name_; }

unsigned int HPLUS_action::get_cost() const { return this -> cost_; }