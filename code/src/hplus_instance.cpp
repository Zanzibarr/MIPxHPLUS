#include "../include/hplus_instance.hpp"

// ##################################################################### //
// ############################## GLOBALS ############################## //
// ##################################################################### //

HPLUS_instance HPLUS_inst;
HPLUS_environment HPLUS_env;
HPLUS_statistics HPLUS_stats;

void HPLUS_statistics::print() const {

    #if HPLUS_VERBOSE < 5
    return;
    #endif

    mylog.print("\n\n--------------------------------------------------");
    mylog.print("-----------------   Statistics   -----------------");
    mylog.print("--------------------------------------------------\n");
    mylog.print(" >>  Parsing time                  %10.3fs  <<", this -> parsing_time);
    mylog.print(" >>  Problem simplification time   %10.3fs  <<", this -> simplification_time);
    mylog.print(" >>  Heuristic time                %10.3fs  <<", this -> heuristic_time);
    mylog.print(" >>  Model building time           %10.3fs  <<", this -> build_time);
    mylog.print(" >>  CPLEX execution time          %10.3fs  <<", this -> execution_time);
    if (HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_SMALL || HPLUS_env.alg == HPLUS_CLI_ALG_DYNAMIC_LARGE) mylog.print(" >>  CPLEX callback time           %10.3fs  <<", this -> callback_time);
    mylog.print(" >>  Total time                    %10.3fs  <<", this -> total_time);
    mylog.print("\n\n");

}

// ##################################################################### //
// ########################### HPLUS_INSTANCE ########################## //
// ##################################################################### //

HPLUS_instance::HPLUS_instance(const std::string& instance_file) {

    parse_inst_file_(instance_file);

    this -> n_var_post_simpl_ = this -> n_var_;
    this -> n_act_post_simpl_ = this -> n_act_;

    this -> best_cost_ = UINT_MAX;

    this -> eliminated_var_ = my::binary_set(this -> n_var_);
    this -> fixed_var_ = my::binary_set(this -> n_var_);
    this -> eliminated_act_ = my::binary_set(this -> n_act_);
    this -> fixed_act_ = my::binary_set(this -> n_act_);
    this -> eliminated_fa_ = std::vector<my::binary_set>(this -> n_act_, my::binary_set(this -> n_var_));
    this -> fixed_fa_ = std::vector<my::binary_set>(this -> n_act_, my::binary_set(this -> n_var_));
    this -> timestamps_var_ = std::vector<int>(this -> n_var_, -1);
    this -> timestamps_act_ = std::vector<int>(this -> n_act_, -1);

    this -> inverse_act_ = std::vector<std::vector<size_t>>(n_act_, std::vector<size_t>());

    this -> var_idx_post_simplification_ = std::vector<size_t>(this -> n_var_);
    for (size_t var_i = 0; var_i < this -> n_var_; var_i++) this -> var_idx_post_simplification_[var_i] = var_i;
    this -> act_idx_post_simplification_ = std::vector<size_t>(this -> n_act_);
    for (size_t act_i = 0; act_i < this -> n_act_; act_i++) this -> act_idx_post_simplification_[act_i] = act_i;
    this -> cpx_idx_to_act_idx_ = std::vector<size_t>(this -> n_act_);
    for (size_t act_i = 0; act_i < this -> n_act_; act_i++) this -> cpx_idx_to_act_idx_[act_i] = act_i;

    pthread_mutex_init(&(this -> solution_read_write_), nullptr);

    mylog.print_info("Created HPLUS_instance.");

}

int HPLUS_instance::get_version() const { return this -> version_; }

bool HPLUS_instance::unitary_cost() const { return this -> unitary_costs_; }

size_t HPLUS_instance::get_n_var(bool simplified) const { return simplified ? this -> n_var_post_simpl_ : this -> n_var_; }

size_t HPLUS_instance::get_n_act(bool simplified) const { return simplified ? this -> n_act_post_simpl_ : this -> n_act_; }

const my::binary_set HPLUS_instance::get_remaining_variables() const { return !this -> eliminated_var_; }

const my::binary_set HPLUS_instance::get_remaining_actions() const { return !this -> eliminated_act_; }

const std::vector<HPLUS_action>& HPLUS_instance::get_actions() const { return this -> actions_; }

const my::binary_set& HPLUS_instance::get_goal_state() const { return this -> goal_state_; }

void HPLUS_instance::update_best_sol(const std::vector<size_t>& solution, const unsigned int cost) {

    #if HPLUS_INTCHECK
    my::binary_set dbcheck = my::binary_set(this -> n_act_);
    unsigned int costcheck = 0;
    my::assert(solution.size() <= this -> n_act_, "Solution has more actions that there actually exists.");             // check that there aren't more actions that there exists
    my::binary_set feas_checker(this -> n_var_);
    for (auto act_i : solution) {
        my::assert(act_i < this -> n_act_, "Solution contains unexisting action.");                                     // check that the solution only contains existing actions
        my::assert(!dbcheck[act_i], "Solution contains duplicate action.");                                             // check that there are no duplicates
        dbcheck.add(act_i);
        my::assert(feas_checker.contains(this -> actions_[act_i].get_pre()), "Preconditions are not respected.");       // check if the preconditions are respected at each step
        feas_checker |= this -> actions_[act_i].get_eff();
        costcheck += this -> actions_[act_i].get_cost();
    }
    my::assert(feas_checker.contains(this -> goal_state_), "The solution doesn't lead to the final state.");            // check if the solution leads to the goal state
    my::assert(costcheck == cost, "Declared cost is different from calculated one.");                                   // check if the cost is the declared one
    #endif

    pthread_mutex_lock(&(this -> solution_read_write_));

    if (cost >= this -> best_cost_) {
        pthread_mutex_unlock(&(this -> solution_read_write_));
        return;
    }

    this -> best_solution_ = std::vector<size_t>(solution);
    this -> best_cost_ = cost;

    mylog.print_info("Updated best solution - Cost: %4d.", this -> best_cost_);

    pthread_mutex_unlock(&(this -> solution_read_write_));

}

void HPLUS_instance::get_best_sol(std::vector<size_t>& solution, unsigned int& cost) {

    pthread_mutex_lock(&(this -> solution_read_write_));

    solution = std::vector<size_t>(this -> best_solution_);
    cost = this -> best_cost_;

    pthread_mutex_unlock(&(this -> solution_read_write_));

}

unsigned int HPLUS_instance::get_best_sol_cost() {

    pthread_mutex_lock(&(this -> solution_read_write_));

    unsigned int ret = this -> best_cost_;

    pthread_mutex_unlock(&(this -> solution_read_write_));

    return ret;

}


void HPLUS_instance::print_best_sol() {

    pthread_mutex_lock(&(this -> solution_read_write_));
    
    mylog.print("Solution cost: %d", this -> best_cost_);
    for(auto act_i : this -> best_solution_) mylog.print("(%s)", this -> actions_[act_i].get_name().c_str());

    pthread_mutex_unlock(&(this -> solution_read_write_));

}

void HPLUS_instance::problem_simplification() {

    this -> imai_model_enhancements();

    size_t count = 0;
    for (auto var_i : !this -> eliminated_var_) this -> var_idx_post_simplification_[var_i] = count++;
    this -> n_var_post_simpl_ = count;
    count = 0;
    for (auto act_i : !this -> eliminated_act_) {
        this -> cpx_idx_to_act_idx_[count] = act_i;
        this -> act_idx_post_simplification_[act_i] = count++;
    }
    this -> n_act_post_simpl_ = count;

    this -> fa_individual_start_ = std::vector<size_t>(this -> n_act_post_simpl_);
    for (size_t act_i = 0; act_i < this -> n_act_post_simpl_; act_i++) this -> fa_individual_start_[act_i] = act_i * this -> n_var_post_simpl_;

    #if HPLUS_INTCHECK
    my::assert(!this -> fixed_var_.intersects(this -> eliminated_var_), "Eliminated and fixed variables intersect.");
    my::assert(!this -> fixed_act_.intersects(this -> eliminated_act_), "Eliminated and fixed actions intersect.");
    #endif

    #if HPLUS_VERBOSE >= 20
    count = 0;
    for (auto _ : this -> fixed_var_) count++;
    mylog.print_info("Optimized number of variables:         %10d --> %10d.", this -> n_var_, this -> n_var_post_simpl_ - count);
    count = 0;
    for (auto _ : this -> fixed_act_) count++;
    mylog.print_info("Optimized number of actions:           %10d --> %10d.", this -> n_act_, this -> n_act_post_simpl_ - count);
    count = 0;
    for (auto _ : this -> eliminated_fa_) count++;
    for (auto _ : this -> fixed_fa_) count++;
    mylog.print_info("Optimized number of first archievers:  %10d --> %10d.", this -> n_var_ * this -> n_act_, this -> n_var_post_simpl_ * this -> n_act_post_simpl_ - count);
    #endif
    
}

const my::binary_set& HPLUS_instance::get_fixed_variables() const { return this -> fixed_var_; }

const my::binary_set& HPLUS_instance::get_fixed_actions() const { return this -> fixed_act_; }

const std::vector<my::binary_set>& HPLUS_instance::get_eliminated_fa() const { return this -> eliminated_fa_; }

const std::vector<my::binary_set>& HPLUS_instance::get_fixed_fa() const { return this -> fixed_fa_; }

const std::vector<int> HPLUS_instance::get_timestamps_var() const { return this -> timestamps_var_; }

const std::vector<int> HPLUS_instance::get_timestamps_act() const { return this -> timestamps_act_; }

const std::vector<size_t>& HPLUS_instance::get_inverse_actions(const size_t act_i) const { return this -> inverse_act_[act_i]; }

void HPLUS_instance::store_cycle(std::vector<size_t>& cycle) { this -> dynamic_model_cycles_.push_back(std::vector<size_t>(cycle)); }

const std::vector<std::vector<size_t>>& HPLUS_instance::get_cycles() const { return this -> dynamic_model_cycles_; }

size_t HPLUS_instance::var_idx_post_simplification(size_t var_idx) const { return this -> var_idx_post_simplification_[var_idx]; }

size_t HPLUS_instance::act_idx_post_simplification(size_t act_idx) const { return this -> act_idx_post_simplification_[act_idx]; }

size_t HPLUS_instance::fa_idx_post_simplification(size_t act_idx, size_t var_idx) const { return this -> fa_individual_start_[this -> act_idx_post_simplification_[act_idx]] + this -> var_idx_post_simplification_[var_idx]; }

size_t HPLUS_instance::cpx_idx_to_act_idx(size_t cpx_idx) const { return this -> cpx_idx_to_act_idx_[cpx_idx]; }

void HPLUS_instance::landmarks_extraction(std::vector<my::binary_set>& landmarks_set, my::binary_set& fact_landmarks, my::binary_set& act_landmarks) const {

    #if HPLUS_VERBOSE >= 20
    mylog.print_info("(debug) Extracting landmarks.");
    #endif

    landmarks_set = std::vector<my::binary_set>(this -> n_var_, my::binary_set(this -> n_var_ + 1));
    fact_landmarks = my::binary_set(this -> n_var_);
    act_landmarks = my::binary_set(this -> n_act_);

    for (size_t var_i = 0; var_i < this -> n_var_; var_i++) landmarks_set[var_i].add(this -> n_var_);

    my::binary_set s_set(this -> n_var_);

    std::deque<size_t> actions_queue;
    for (size_t act_i = 0; act_i < this -> n_act_; act_i++) if(s_set.contains(this -> actions_[act_i].get_pre())) actions_queue.push_back(act_i);

    // list of actions that have as precondition variable p
    std::vector<std::vector<size_t>> act_with_pre(this -> n_var_);
    for (size_t var_i = 0; var_i < this -> n_var_; var_i++) for (size_t act_i = 0; act_i < this -> n_act_; act_i++) if (this -> actions_[act_i].get_pre()[var_i]) act_with_pre[var_i].push_back(act_i);

    while(!actions_queue.empty()) {

        const HPLUS_action a = this -> actions_[actions_queue.front()];
        actions_queue.pop_front();

        const auto& pre_sparse = a.get_pre_sparse();
        const auto& add_sparse = a.get_eff_sparse();

        for (auto var_i : add_sparse) {

            s_set.add(var_i);

            my::binary_set x = my::binary_set(this -> n_var_ + 1);
            for (auto var_j : add_sparse) x.add(var_j);

            for (auto var_j : pre_sparse) {
                // if variable var_i' has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set the flag
                if (landmarks_set[var_j][this -> n_var_]) {
                    x.add(this -> n_var_);
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
    std::vector<std::vector<size_t>> act_with_eff(this -> n_var_);
    for (auto var_i : !fact_landmarks) for (size_t act_i = 0; act_i < this -> n_act_; act_i++) if (this -> actions_[act_i].get_eff()[var_i]) act_with_eff[var_i].push_back(act_i);

    // popolate fact_landmarks and act_landmarks sets
    for (auto var_i : this -> goal_state_) {
        for (auto var_j : !fact_landmarks) {
            if (landmarks_set[var_i][var_j] || landmarks_set[var_i][this -> n_var_]) {
                fact_landmarks.add(var_j);
                size_t count = 0, cand_act;
                for (auto act_i : act_with_eff[var_j]) {
                    cand_act = act_i;
                    count++;
                    if (count > 1) break;
                }
                if (count == 1) act_landmarks.add(cand_act);
            }
        }
    }

}

void HPLUS_instance::first_adders_extraction(const std::vector<my::binary_set>& landmarks_set, std::vector<my::binary_set>& fadd) const {

    #if HPLUS_VERBOSE >= 20
    mylog.print_info("(debug) Extracting first adders.");
    #endif

    fadd = std::vector<my::binary_set>(this -> n_act_, my::binary_set(this -> n_var_));

    // list of fact landmarks for each variable
    std::vector<std::vector<size_t>> var_flm_sparse(this -> n_var_);
    for (size_t var_i = 0; var_i < this -> n_var_; var_i++) for (size_t var_j = 0; var_j < this -> n_var_; var_j++) if (landmarks_set[var_i][var_j] || landmarks_set[var_i][this -> n_var_]) var_flm_sparse[var_i].push_back(var_j);

    // compute the set of first adders
    for (size_t act_i = 0; act_i < this -> n_act_; act_i++) {
        // f_lm_a is the set of fact landmarks of action act_i
        my::binary_set f_lm_a(this -> n_var_);
        for (auto var_i : this -> actions_[act_i].get_pre()) for (auto i : var_flm_sparse[var_i]) f_lm_a.add(i);
        // fadd[a] := { p in add(a) s.t. p is not a fact landmark for a }
        fadd[act_i] |= (this -> actions_[act_i].get_eff() & !f_lm_a);
    }

}

void HPLUS_instance::relevance_analysis(const my::binary_set& fact_landmarks, const std::vector<my::binary_set>& fadd) {

    #if HPLUS_VERBOSE >= 20
    mylog.print_info("(debug) Relevance analysis.");
    #endif

    my::binary_set relevant_variables = my::binary_set(this -> n_var_);
    my::binary_set relevant_actions = my::binary_set(this -> n_act_);

    // compute first round of relevand variables and actions
    for (size_t act_i = 0; act_i < this -> n_act_; act_i++) {
        if (fadd[act_i].intersects(this -> goal_state_)) {
            relevant_variables |= this -> actions_[act_i].get_pre();
            relevant_actions.add(act_i);
        }
    }

    // list of actions yet to check
    auto cand_actions_sparse = (!relevant_actions).sparse();

    // keep looking for other relevant actions/variables until no more can be found
    bool new_act = true;
    while (new_act) {
        new_act = false;
        std::vector<size_t> new_relevant_actions;
        for (auto act_i : cand_actions_sparse) {
            if (this -> actions_[act_i].get_eff().intersects(relevant_variables)) {
                relevant_actions.add(act_i);
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
    this -> eliminated_var_ |= (!relevant_variables - fact_landmarks);
    this -> eliminated_act_ |= !relevant_actions;

}

void HPLUS_instance::dominated_actions_elimination(const std::vector<my::binary_set>& landmarks_set, const std::vector<my::binary_set>& fadd) {
    
    #if HPLUS_VERBOSE >= 20
    mylog.print_info("(debug) Extracting dominated actions.");
    #endif

    auto remaining_var_sparse = (!this -> eliminated_var_).sparse();
    std::vector<my::binary_set> act_flm = std::vector<my::binary_set>(this -> n_act_, my::binary_set(this -> n_var_));
    std::vector<std::vector<size_t>> var_flm_sparse(this -> n_var_);

    // compute the landmarks for each variable remaining
    for (auto var_i : remaining_var_sparse)
        for (auto var_j : remaining_var_sparse)
            if (landmarks_set[var_i][var_j] || landmarks_set[var_i][this -> n_var_])
                var_flm_sparse[var_i].push_back(var_j);
                
    // compute the landmarks for each action remaining
    for (size_t act_i = 0; act_i < this -> n_act_; act_i++)
        for (auto var_i : this -> actions_[act_i].get_pre_sparse()) if (!this -> eliminated_var_[var_i])
            for (auto i : var_flm_sparse[var_i])
                act_flm[act_i].add(i);

    // find efficently all actions that satisfy point 1) of Proposition 4 of in Imai's Paper
    my::subset_searcher subset_finder = my::subset_searcher();
    for (auto act_i : !this -> eliminated_act_) subset_finder.add(act_i, fadd[act_i]);

    my::binary_set dominated_actions(this -> n_act_);

    // find all dominated actions and eliminate them
    for (auto dominant_act : !this -> eliminated_act_) if (!dominated_actions[dominant_act]) {
        for (auto dominated_act : subset_finder.find_subsets(fadd[dominant_act])) if (!this -> eliminated_act_[dominated_act]) {
            
            if (this -> fixed_act_[dominated_act] || dominant_act == dominated_act || this -> actions_[dominant_act].get_cost() > this -> actions_[dominated_act].get_cost() || !act_flm[dominated_act].contains(this -> actions_[dominant_act].get_pre())) continue;

            dominated_actions.add(dominated_act);
            this -> eliminated_act_.add(dominated_act);

        }

    }

}

void HPLUS_instance::immediate_action_application(const my::binary_set& act_landmarks) {

    #if HPLUS_VERBOSE >= 20
    mylog.print_info("(debug) Immediate action application.");
    #endif

    my::binary_set current_state(this -> n_var_);
    my::binary_set actions_left = !this -> eliminated_act_;

    // keep looking until no more actions can be applied
    int counter = 0;
    bool found_next_action = true;
    while (found_next_action) {

        found_next_action = false;

        for (auto act_i : actions_left) {

            const auto& pre = this -> actions_[act_i].get_pre() & !this -> eliminated_var_;
            const auto& eff = this -> actions_[act_i].get_eff() & !this -> eliminated_var_;

            if (current_state.contains(pre) && (act_landmarks[act_i] || this -> actions_[act_i].get_cost() == 0)) {

                actions_left.remove(act_i);
                this -> fixed_act_.add(act_i);
                if (HPLUS_env.alg == HPLUS_CLI_ALG_IMAI) this -> timestamps_act_[act_i] = counter;
                this -> fixed_var_ |= pre;
                for (auto var_i : eff) if (!current_state[var_i]) {
                    this -> fixed_var_.add(var_i);
                    if (HPLUS_env.alg == HPLUS_CLI_ALG_IMAI) this -> timestamps_var_[var_i] = counter+1;
                    this -> fixed_fa_[act_i].add(var_i);
                    for (auto act_j : actions_left) this -> eliminated_fa_[act_j].add(var_i);
                }
                current_state |= eff;
                counter++;
                found_next_action = true;

            }

        }

    }

}

void HPLUS_instance::inverse_actions_extraction() {

    #if HPLUS_VERBOSE >= 20
    mylog.print_info("(debug) Extracting inverse actions.");
    #endif

    my::subset_searcher subset_finder = my::subset_searcher();

    // find efficiently all actions that satisfy point 2) of the Definition 1 in section 4.6 of Imai's paper
    const auto remaining_actions = this -> get_remaining_actions().sparse();
    for (auto act_i : remaining_actions) subset_finder.add(act_i, this -> actions_[act_i].get_eff());

    for (auto act_i : remaining_actions) {
        const auto& pre = this -> actions_[act_i].get_pre();
        const auto& eff = this -> actions_[act_i].get_eff();
        for (auto act_j : subset_finder.find_subsets(pre)) {
            if (this -> actions_[act_j].get_pre().contains(eff)) {
                if (!this -> get_fixed_actions()[act_i]) this -> inverse_act_[act_i].push_back(act_j);
                if (!this -> get_fixed_actions()[act_j]) this -> inverse_act_[act_j].push_back(act_i);
            }
        }
    }

}

void HPLUS_instance::imai_model_enhancements() {

    // mylog.print_info("Model enhancement from Imai's paper.");

    std::vector<my::binary_set> landmarks_set;
    my::binary_set fact_landmarks;
    my::binary_set act_landmarks;
    std::vector<my::binary_set> fadd;

    this -> landmarks_extraction(landmarks_set, fact_landmarks, act_landmarks);

    this -> fixed_var_ |= fact_landmarks;
    this -> fixed_act_ |= act_landmarks;

    this -> first_adders_extraction(landmarks_set, fadd);

    for (size_t act_i = 0; act_i < this -> n_act_; act_i++) this -> eliminated_fa_[act_i] |= (this -> actions_[act_i].get_eff() & !fadd[act_i]);

    this -> relevance_analysis(fact_landmarks, fadd);

    this -> dominated_actions_elimination(landmarks_set, fadd);

    this -> immediate_action_application(act_landmarks);

    if (HPLUS_env.alg != HPLUS_CLI_ALG_RANKOOH && HPLUS_env.alg != HPLUS_CLI_ALG_GREEDY) this -> inverse_actions_extraction();

}

void HPLUS_instance::parse_inst_file_(const std::string& instance_path_) {

    mylog.print_info("Parsing SAS file.");

    std::ifstream ifs(instance_path_.c_str(), std::ifstream::in);
    my::assert(ifs.good(), "Opening instance file failed.");

    HPLUS_stats.parsing_time = HPLUS_env.time_limit - timer.get_time();
    std::string line;

    // * version section
    std::getline(ifs, line);   // begin_version
    my::assert(line == "begin_version", "Corrupted file.");
    std::getline(ifs, line);   // version_number
    my::assert(my::isint(line), "Corrupted file.");
    this -> version_ = std::stoi(line);
    std::getline(ifs, line);   // end_version
    my::assert(line == "end_version", "Corrupted file.");

    // * metric section
    std::getline(ifs, line);   // begin_metric
    my::assert(line == "begin_metric", "Corrupted file.");
    std::getline(ifs, line);   // metric
    my::assert(my::isint(line, 0, 1), "Corrupted file.");
    this -> unitary_costs_ = stoi(line) != 1;
    std::getline(ifs, line);   // end_metric
    my::assert(line == "end_metric", "Corrupted file.");

    // * variables section
    mylog.print_warn("Ignoring axiom layers.");
    std::getline(ifs, line);   // nvar_pre_exp
    my::assert(my::isint(line, 0), "Corrupted file.");
    size_t nvar_pre_exp = std::stoi(line);
    std::vector<size_t> var_ranges(nvar_pre_exp);
    this -> n_var_ = 0;
    for (size_t var_i = 0; var_i < nvar_pre_exp; var_i++) {
        // process each variable
        std::getline(ifs, line);   // begin_variable
        my::assert(line == "begin_variable", "Corrupted file.");
        std::getline(ifs, line);   // variable name (ignored)
        std::getline(ifs, line);   // axiom layer (ignored)
        my::assert(line == "-1", "Axiom layer is not -1, this software is not made for this instance.");
        std::getline(ifs, line);   // range of variable
        my::assert(my::isint(line, 0), "Corrupted file.");
        size_t range = stoi(line);
        var_ranges[var_i] = range;
        this -> n_var_ += range;
        for (size_t j = 0; j < range; j++) std::getline(ifs, line);   // name for variable value (ignored)
        std::getline(ifs, line);   // end_variable
        my::assert(line == "end_variable", "Corrupted file.");
    }
    
    // * mutex section (ignored)
    mylog.print_warn("Ignoring mutex section.");
    std::getline(ifs, line);   // number of mutex groups
    my::assert(my::isint(line, 0), "Corrupted file.");
    size_t nmgroups = stoi(line);
    for (size_t i = 0; i < nmgroups; i++) {
        std::getline(ifs, line);   // begin_mutex_group
        my::assert(line == "begin_mutex_group", "Corrupted file.");
        while (line != "end_mutex_group") {
            std::getline(ifs, line); // reach end_mutex_group (ignore all content)
            my::assert(line != "begin_state", "Corrupted file.");
        }
    }

    // * initial state section
    std::getline(ifs, line);   // begin_state
    my::assert(line == "begin_state", "Corrupted file.");
    my::binary_set istate = my::binary_set(this -> n_var_);
    for (size_t var_i = 0, c = 0; var_i < nvar_pre_exp; c += var_ranges[var_i], var_i++) {
        std::getline(ifs, line);   // initial value of var_i
        my::assert(my::isint(line, 0, var_ranges[var_i] - 1), "Corrupted file.");
        size_t val = stoi(line);
        istate.add(c + val);
    }
    std::getline(ifs, line);   // end_state
    my::assert(line == "end_state", "Corrupted file.");

    // removing initial state variables
    std::vector<size_t> post_istate_removal_offset(this -> n_var_);
    size_t counter = 0;
    for (size_t var_i = 0; var_i < this -> n_var_; var_i++) {
        if (istate[var_i]) counter++;
        post_istate_removal_offset[var_i] = counter;
    }
    this -> n_var_ -= nvar_pre_exp;

    // * goal state section
    std::getline(ifs, line);   // begin_goal
    my::assert(line == "begin_goal", "Corrupted file.");
    this -> goal_state_ = my::binary_set(this -> n_var_);
    std::getline(ifs, line);   // number of goals
    my::assert(my::isint(line, 0, nvar_pre_exp), "Corrupted file.");
    size_t ngoals = stoi(line);
    for (size_t _ = 0; _ < ngoals; _++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(ifs, line);   // pair 'variable goal'
        tokens = my::split_string(line, ' ');
        my::assert(tokens.size() == 2, "Corrupted file.");
        my::assert(my::isint(tokens[0], 0, nvar_pre_exp - 1), "Corrupted file."); // variable index
        size_t var = stoi(tokens[0]);
        my::assert(my::isint(tokens[1], 0, var_ranges[var] - 1), "Corrupted file."); // variable goal
        size_t value = stoi(tokens[1]);
        size_t var_strips = value;
        for (size_t j = 0; j < var; var_strips +=var_ranges[j], j++) {}
        if (!istate[var_strips]) this -> goal_state_.add(var_strips - post_istate_removal_offset[var_strips]);
    }
    std::getline(ifs, line);   // end_goal
    my::assert(line == "end_goal", "Corrupted file.");

    // * operator (actions) section
    mylog.print_warn("Ignoring effect conditions.");
    std::getline(ifs, line);   // n_act
    my::assert(my::isint(line, 0), "Corrupted file.");
    this -> n_act_ = stoi(line);
    this -> actions_ = std::vector<HPLUS_action>(this -> n_act_);
    for (size_t act_i = 0; act_i < this -> n_act_; act_i++) {
        // process each action
        std::getline(ifs, line);   // begin_operator
        my::assert(line == "begin_operator", "Corrupted file.");
        std::getline(ifs, line);   // symbolic action name
        std::string name = line;
        my::binary_set act_pre(this -> n_var_);
        std::getline(ifs, line);   // number of prevail conditions
        my::assert(my::isint(line, 0, nvar_pre_exp), "Corrupted file.");
        size_t n_pre = stoi(line);
        for (size_t pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(ifs, line);   // pair 'variable value'
            tokens = my::split_string(line, ' ');
            my::assert(tokens.size() == 2, "Corrupted file.");
            my::assert(my::isint(tokens[0], 0, nvar_pre_exp - 1), "Corrupted file."); // variable index
            size_t var = stoi(tokens[0]);
            my::assert(my::isint(tokens[1], 0,var_ranges[var] - 1), "Corrupted file."); // variable value
            size_t value = stoi(tokens[1]);
            size_t var_strips = value;
            for (size_t i = 0; i < var; var_strips += var_ranges[i], i++) {}
            if (!istate[var_strips]) act_pre.add(var_strips - post_istate_removal_offset[var_strips]);
        }
        std::getline(ifs, line);   // number of effects
        my::assert(my::isint(line, 0), "Corrupted file.");
        size_t n_eff = stoi(line);
        my::binary_set act_eff(this -> n_var_);
        for (size_t eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(ifs, line);   // effect line
            std::vector<std::string> tokens;
            tokens = my::split_string(line, ' ');
            my::assert(tokens.size() == 4, "This program won't handle effect conditions."); // not expecting effect conditions
            my::assert(my::isint(tokens[0], 0, 0), "This program won't handle effect conditions."); // number of effect conditions (ignored and check to be 0)
            my::assert(my::isint(tokens[1], 0, nvar_pre_exp - 1), "Corrupted file.");   // variable affected by the action
            size_t var = stoi(tokens[1]);
            my::assert(my::isint(tokens[2], -1, var_ranges[var] - 1), "Corrupted file.");    // precondition of the variable
            int pre_val = stoi(tokens[2]);
            my::assert(my::isint(tokens[3], 0, var_ranges[var] - 1), "Corrupted file."); // effect of the variable
            size_t eff_val = stoi(tokens[3]);
            size_t c = 0;
            for (size_t i = 0; i < var; c += var_ranges[i], i++){}
            if (pre_val >= 0 && !istate[c + pre_val]) act_pre.add(c + pre_val - post_istate_removal_offset[c + pre_val]);
            if (!istate[c + eff_val]) act_eff.add(c + eff_val - post_istate_removal_offset[c + eff_val]);
        }
        std::getline(ifs, line);   // action cost
        my::assert(my::isint(line), "Corrupted file.");
        unsigned int cost = 1;
        if (!this -> unitary_costs_) cost = stoi(line);
        std::getline(ifs, line);   // end_operator
        my::assert(line == "end_operator", "Corrupted file.");
        this -> actions_[act_i] = HPLUS_action(act_pre, act_eff, cost, name);
    }

    mylog.print_warn("Ignoring axiom section.");

    ifs.close();

    HPLUS_stats.parsing_time = timer.get_time();
    
}

// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

HPLUS_action::HPLUS_action(const my::binary_set& preconditions, const my::binary_set& effects, unsigned int cost, const std::string& name) {

    this -> pre_ = my::binary_set(preconditions);
    this -> eff_ = my::binary_set(effects);
    for (size_t var_i = 0; var_i < this -> pre_.size(); var_i++) {
        if (this -> pre_[var_i]) this -> pre_sparse_.push_back(var_i);
        if (this -> eff_[var_i]) this -> eff_sparse_.push_back(var_i);
    }
    this -> cost_ = cost;
    this -> name_ = name;

}

const my::binary_set& HPLUS_action::get_pre() const { return this -> pre_; }

const std::vector<size_t>& HPLUS_action::get_pre_sparse() const { return this -> pre_sparse_; }

const my::binary_set& HPLUS_action::get_eff() const { return this -> eff_; }

const std::vector<size_t>& HPLUS_action::get_eff_sparse() const { return this -> eff_sparse_; }

const std::string& HPLUS_action::get_name() const { return this -> name_; }

unsigned int HPLUS_action::get_cost() const { return this -> cost_; }