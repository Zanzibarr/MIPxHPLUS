#include "../include/hplus_instance.hpp"

// ##################################################################### //
// ##################### HPLUS_INSTANCE ALGORITHMS ##################### //
// ##################################################################### //

void HPLUS_instance::landmarks_extraction(std::vector<my::BitField>& landmarks_set, my::BitField& fact_landmarks, my::BitField& act_landmarks) const {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Extracting landmarks.");
    #endif

    landmarks_set = std::vector<my::BitField>(this -> nvarstrips_, my::BitField(this -> nvarstrips_ + 1));
    fact_landmarks = my::BitField(this -> nvarstrips_);
    act_landmarks = my::BitField(this -> n_act_);

    for (unsigned int p = 0; p < this -> nvarstrips_; p++) landmarks_set[p].set(this -> nvarstrips_);

    my::BitField s_set(this -> nvarstrips_);

    std::deque<int> actions_queue;
    for (unsigned int i = 0; i < this -> n_act_; i++) if(s_set.contains(this -> actions_[i].get_pre())) actions_queue.push_back(i);

    // list of actions that have as precondition variable p
    std::vector<std::vector<unsigned int>> act_with_pre(this -> nvarstrips_);
    for (unsigned int p = 0; p < this -> nvarstrips_; p++) for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) if (this -> actions_[act_i].get_pre()[p]) act_with_pre[p].push_back(act_i);

    while(!actions_queue.empty()) {

        const HPLUS_action a = this -> actions_[actions_queue.front()];
        actions_queue.pop_front();

        const auto& pre_a = a.get_pre_sparse();
        const auto& add_a = a.get_eff_sparse();

        for (auto p : add_a) {

            s_set.set(p);

            my::BitField x = my::BitField(this -> nvarstrips_ + 1);
            for (auto p : add_a) x.set(p);

            for (auto pp : pre_a) {
                // if variable p' has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set the flag
                if (landmarks_set[pp][this -> nvarstrips_]) {
                    x.set(this -> nvarstrips_);
                    // if x is now full we can exit, since all further unions won't change x
                    break;
                } else x |= landmarks_set[pp];
            }

            // we then check if L[p] != X, and if they are the same we skip,
            // if X = P, then (X intersection L[P]) = L[P], hence we can already skip
            if (!x[this -> nvarstrips_]) {

                // if the set for variable p is the full set of variables,
                // the intersection generates back x -> we can skip the intersection
                if (!landmarks_set[p][this -> nvarstrips_]) x &= landmarks_set[p];

                // we already know that x is not the full set now, so if
                // the set for variable p is the full set, we know that x is not
                // equal to the set for variable p -> we can skip the check
                if (landmarks_set[p][this -> nvarstrips_] || x != landmarks_set[p]) {

                    landmarks_set[p] = x;
                    for (auto aa : act_with_pre[p])
                        if (s_set.contains(this -> actions_[aa].get_pre()) && std::find(actions_queue.begin(), actions_queue.end(), aa) == actions_queue.end())
                            actions_queue.push_back(aa);

                }

            }

        }

    }

    // list of actions that have as effect variable p
    std::vector<std::vector<unsigned int>> act_with_eff(this -> nvarstrips_);
    for (auto p : !fact_landmarks) for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) if (this -> actions_[act_i].get_eff()[p]) act_with_eff[p].push_back(act_i);

    for (auto p : this -> goal_state_) {
        for (auto j : !fact_landmarks) {
            if (landmarks_set[p][j] || landmarks_set[p][this -> nvarstrips_]) {
                fact_landmarks.set(j);
                unsigned int count = 0, cand_act;
                for (auto act_i : act_with_eff[j]) {
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

    fadd = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> nvarstrips_));

    // list of fact landmarks for each variable
    std::vector<std::vector<unsigned int>> var_flm(this -> nvarstrips_);
    for (unsigned int p = 0; p < this -> nvarstrips_; p++) for (unsigned int i = 0; i < this -> nvarstrips_; i++) if (landmarks_set[p][i] || landmarks_set[p][this -> nvarstrips_]) var_flm[p].push_back(i);

    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        const auto& pre = this -> actions_[act_i].get_pre_sparse();
        // f_lm_a is the set of fact landmarks of action act_i
        my::BitField f_lm_a(this -> nvarstrips_);
        for (auto p : pre) for (auto i : var_flm[p]) f_lm_a.set(i);
        // fadd[a] := { p in add(a) s.t. p is not a fact landmark for a }
        fadd[act_i] |= (this -> actions_[act_i].get_eff() & !f_lm_a);
    }

}

void HPLUS_instance::relevance_analysis(const my::BitField& fact_landmarks, const std::vector<my::BitField>& fadd, my::BitField& relevant_variables, my::BitField& relevant_actions) const {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Relevance analysis.");
    #endif

    relevant_variables = my::BitField(this -> nvarstrips_);
    relevant_actions = my::BitField(this -> n_act_);

    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        if (fadd[act_i].intersects(this -> goal_state_)) {
            relevant_variables |= this -> actions_[act_i].get_pre();
            relevant_actions.set(act_i);
        }
    }

    auto cand_actions = (!relevant_actions).sparse();

    bool new_act = true;
    while (new_act) {
        new_act = false;
        std::vector<unsigned int> removed_act;
        for (auto act_i : cand_actions) {
            if (this -> actions_[act_i].get_eff().intersects(relevant_variables)) {
                relevant_actions.set(act_i);
                relevant_variables |= this -> actions_[act_i].get_pre();
                removed_act.push_back(act_i);
                new_act = true;
            }
        }
        auto it = std::set_difference(cand_actions.begin(), cand_actions.end(), removed_act.begin(), removed_act.end(), cand_actions.begin());
        cand_actions.resize(it - cand_actions.begin());
    }
    relevant_variables |= this -> goal_state_;

}

void HPLUS_instance::dominated_actions_extraction(const std::vector<my::BitField>& landmarks_set, const std::vector<my::BitField>& fadd, const my::BitField& eliminated_actions, const my::BitField& fixed_actions, my::BitField& dominated_actions) const {
    
    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Extracting dominated actions.");
    #endif

    dominated_actions =  my::BitField(this -> n_act_);
    const auto actions = this -> actions_;

    std::vector<my::BitField> f_lm_a = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> nvarstrips_));

    std::vector<std::vector<unsigned int>> var_flm(this -> nvarstrips_);
    for (unsigned int p = 0; p < this -> nvarstrips_; p++) for (unsigned int i = 0; i < this -> nvarstrips_; i++) if (landmarks_set[p][i] || landmarks_set[p][this -> nvarstrips_]) var_flm[p].push_back(i);

    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) {
        const auto& pre = this -> actions_[act_i].get_pre_sparse();
        for (auto p : pre) for (auto i : var_flm[p]) f_lm_a[act_i].set(i);
    }

    // find efficently all actions that satisfy point 1) of Proposition 4 of in Imai's Paper
    my::SSBT subset_finder = my::SSBT();
    for (auto act_i : !eliminated_actions) subset_finder.add(act_i, fadd[act_i]);

    for (auto dominant_act : !eliminated_actions) if (!dominated_actions[dominant_act]) {
        for (auto dominated_act : subset_finder.find_subsets(fadd[dominant_act])) {
            
            if (fixed_actions[dominated_act] || dominant_act == dominated_act || actions[dominant_act].get_cost() > actions[dominated_act].get_cost() || !f_lm_a[dominated_act].contains(actions[dominant_act].get_pre())) continue;

            dominated_actions.set(dominated_act);

        }

    }

}

void HPLUS_instance::immediate_action_application(const my::BitField& act_landmarks, const my::BitField& eliminated_variables, my::BitField& fixed_variables, const my::BitField& eliminated_actions, my::BitField& fixed_actions, std::vector<my::BitField>& eliminated_first_archievers, std::vector<my::BitField>& fixed_first_archievers, std::vector<int>* fixed_var_timestamps, std::vector<int>* fixed_act_timestamps) const {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Immediate action application.");
    #endif

    my::BitField current_state(this -> nvarstrips_);
    auto actions_left = !eliminated_actions;

    int counter = 0;
    bool found_next_action = true;
    while (found_next_action) {

        found_next_action = false;

        for (auto act_i : actions_left) {

            const auto& pre = this -> actions_[act_i].get_pre();
            const auto& eff = this -> actions_[act_i].get_pre();
            const auto& eff_sparse = this -> actions_[act_i].get_pre_sparse();

            if (current_state.contains(pre) && (act_landmarks[act_i] || this -> actions_[act_i].get_cost() == 0)) {

                actions_left.unset(act_i);
                fixed_actions.set(act_i);
                if (this -> fixed_act_timestamps != nullptr) (*this -> fixed_act_timestamps)[act_i] = counter;
                fixed_variables |= pre;
                for (auto p : eff_sparse) if (!current_state[p] && !eliminated_variables[p]) {
                    fixed_variables.set(p);
                    if (this -> fixed_act_timestamps != nullptr) (*fixed_var_timestamps)[p] = counter+1;
                    fixed_first_archievers[act_i].set(p);
                    for (auto act_j : actions_left) eliminated_first_archievers[act_j].set(p);
                }
                current_state |= eff;
                counter++;
                found_next_action = true;

            }

        }

    }

}

void HPLUS_instance::inverse_actions_extraction(const my::BitField& eliminated_actions, const my::BitField& fixed_actions, std::map<unsigned int, std::vector<unsigned int>>& inverse_actions) const {

    #if HPLUS_VERBOSE >= 20
    lprint_info("(debug) Extracting inverse actions.");
    #endif

    my::SSBT subset_finder = my::SSBT();

    // find efficiently allactions that satisfy point 2) of the Definition 1 in section 4.6 of Imai's paper
    auto remaining_actions = (!eliminated_actions).sparse();
    for (auto act_i : remaining_actions) subset_finder.add(act_i, this -> actions_[act_i].get_eff());

    for (auto act_i : !eliminated_actions) {
        const auto& pre = this -> actions_[act_i].get_pre();
        const auto& eff = this -> actions_[act_i].get_eff();
        for (auto act_j : subset_finder.find_subsets(pre)) {
            if (this -> actions_[act_j].get_pre().contains(eff)) {
                if (!fixed_actions[act_i]) {
                    inverse_actions.try_emplace(act_i, std::vector<unsigned int>());
                    inverse_actions[act_i].push_back(act_j);
                }
                if (!fixed_actions[act_j]) {
                    inverse_actions.try_emplace(act_j, std::vector<unsigned int>());
                    inverse_actions[act_j].push_back(act_i);
                }
            }
        }
    }

}

void HPLUS_instance::extract_imai_enhancements(my::BitField& eliminated_variables, my::BitField& eliminated_actions) {

    lprint_info("Model enhancement from Imai's paper.");

    std::vector<my::BitField> landmarks_set;
    my::BitField fact_landmarks;
    my::BitField act_landmarks;
    std::vector<my::BitField> fadd;
    my::BitField relevant_variables;
    my::BitField relevant_actions;
    my::BitField dominated_actions;

    this -> landmarks_extraction(landmarks_set, fact_landmarks, act_landmarks);

    this -> fixed_variables |= fact_landmarks;
    this -> fixed_actions |= act_landmarks;

    this -> first_adders_extraction(landmarks_set, fadd);

    for (unsigned int act_i = 0; act_i < this -> n_act_; act_i++) this -> eliminated_first_archievers[act_i] |= (this -> actions_[act_i].get_eff() & !fadd[act_i]);

    this -> relevance_analysis(fact_landmarks, fadd, relevant_variables, relevant_actions);

    eliminated_variables = my::BitField(!relevant_variables - fact_landmarks);
    eliminated_actions = my::BitField(!relevant_actions);

    this -> immediate_action_application(act_landmarks, eliminated_variables, this -> fixed_variables, eliminated_actions, this -> fixed_actions, this -> eliminated_first_archievers, fixed_first_archievers, fixed_var_timestamps, this -> fixed_act_timestamps);

    this -> dominated_actions_extraction(landmarks_set, fadd, eliminated_actions, this -> fixed_actions, dominated_actions);

    eliminated_actions |= dominated_actions;

    //[ ]: This might not be worth doing (and might be wrong)
    // if (inverse_actions != nullptr) this -> inverse_actions_extraction(eliminated_actions, this -> fixed_actions, *inverse_actions);

}

void HPLUS_instance::problem_semplification(const my::BitField& eliminated_variables, const my::BitField& eliminated_actions) {

    // ====================================================== //
    // ======= REMOVE ELIMINATED ACTIONS AND VARIABLES ====== //
    // ====================================================== //

    lprint_info("Problem semplification.");

    // removing eliminated actions from bitfields
    std::vector<unsigned int> el_act_offset(this -> n_act_, 0);
    int removed_actions = 0;
    for (unsigned int i = 0; i < this -> n_act_; i++) {
        if (eliminated_actions[i]) removed_actions++;
        el_act_offset[i] = removed_actions;
    }
    int new_act_size = this -> n_act_ - removed_actions;
    this -> n_act_ = new_act_size;                                                                                                  // updating the number of actions
    auto rm_el_act = [&eliminated_actions, &el_act_offset, &new_act_size](const my::BitField& bf) {
        my::BitField new_bf(new_act_size);
        for (auto act_i : bf) if (!eliminated_actions[act_i]) new_bf.set(act_i - el_act_offset[act_i]);
        return new_bf;
    };
    this -> fixed_actions = rm_el_act(this -> fixed_actions);                                                                       // removing the eliminated actions from the fixed actions set

    // removing eliminated actions from lists
    std::vector<unsigned int> rmact = eliminated_actions.sparse();
    std::sort(rmact.rbegin(), rmact.rend());
    for (auto idx : rmact) {
        this -> actions_.erase(this -> actions_.begin() + idx);                                                                     // removing the eliminated actions from the list of actions
        this -> fixed_first_archievers.erase(this -> fixed_first_archievers.begin() + idx);                                         // removing the eliminated actions from the fixed first archievers set
        this -> eliminated_first_archievers.erase(this -> eliminated_first_archievers.begin() + idx);                               // removing the eliminated actions from the eliminated first archievers set
        if (this -> fixed_act_timestamps != nullptr) this -> fixed_act_timestamps -> erase(this -> fixed_act_timestamps -> begin() + idx);     // removing the eliminated actions from the fixed timestamps list
    }

    // removing eliminated variables from bitfields
    std::vector<unsigned int> el_var_offset(this -> nvarstrips_, 0);
    int removed_variables = 0;
    for (unsigned int i = 0; i < this -> nvarstrips_; i++) {
        if (eliminated_variables[i]) removed_variables++;
        el_var_offset[i] = removed_variables;
    }
    int new_var_size = this -> nvarstrips_ - removed_variables;
    this -> nvarstrips_ = new_var_size;                                                                                             // updating the number of variables (strips)
    auto rm_el_var = [&eliminated_variables, &el_var_offset, &new_var_size](const my::BitField& bf) {
        my::BitField new_bf(new_var_size);
        for (auto p : bf) if(!eliminated_variables[p]) new_bf.set(p - el_var_offset[p]);
        return new_bf;
    };
    for (auto& a : this -> actions_) {
        a.update_pre(rm_el_var(a.get_pre()));                                                                                       // removing the eliminated variables from the actions preconditions
        a.update_eff(rm_el_var(a.get_eff()));                                                                                       // removing the eliminated variables from the actions effects
    }
    this -> goal_state_ = rm_el_var(this -> goal_state_);                                                                           // removing the eliminated variables from the goal state
    this -> fixed_variables = rm_el_var(this -> fixed_variables);                                                                   // removing the eliminated variables from the fixed variables set
    for (auto& bf_a : this -> eliminated_first_archievers) bf_a = rm_el_var(bf_a);                                                  // removing the eliminated variables from the eliminated first archievers set
    for (auto& bf_a : this -> fixed_first_archievers) bf_a = rm_el_var(bf_a);                                                       // removing the eliminated variables from the fixed first archievers set

    // removing eliminated variables from lists
    std::vector<unsigned int> rmvar = eliminated_variables.sparse();
    std::sort(rmvar.rbegin(), rmvar.rend());

    if (fixed_var_timestamps != nullptr) for (auto idx : rmvar) fixed_var_timestamps -> erase(fixed_var_timestamps -> begin() + idx);   // removing the eliminated variables from the fixed timestamps list

}
// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

HPLUS_action::HPLUS_action(my::BitField& pre_bf, my::BitField& eff_bf, const unsigned int cost, const std::string& name) {

    this -> pre_ = my::BitField(pre_bf);
    this -> eff_ = my::BitField(eff_bf);
    for (unsigned int i = 0; i < this -> pre_.size(); i++) {
        if (this -> pre_[i]) this -> sparse_pre_.push_back(i);
        if (this -> eff_[i]) this -> sparse_eff_.push_back(i);
    }
    this -> cost_ = cost;
    this -> name_ = name;

}

void HPLUS_action::update_pre(const my::BitField& bf) {

    this -> pre_ = bf;
    this -> sparse_pre_.clear();
    for (unsigned int i = 0; i < this -> pre_.size(); i++) if (this -> pre_[i]) this -> sparse_pre_.push_back(i);

}

void HPLUS_action::update_eff(const my::BitField& bf) {

    this -> eff_ = bf;
    this -> sparse_eff_.clear();
    for (unsigned int i = 0; i < this -> eff_.size(); i++) if (this -> eff_[i]) this -> sparse_eff_.push_back(i);

}

const my::BitField& HPLUS_action::get_pre() const { return this -> pre_; }

const std::vector<unsigned int>& HPLUS_action::get_pre_sparse() const { return this -> sparse_pre_; }

const my::BitField& HPLUS_action::get_eff() const { return this -> eff_; }

const std::vector<unsigned int>& HPLUS_action::get_eff_sparse() const { return this -> sparse_eff_; }

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

    this -> fixed_variables = my::BitField(this -> nvarstrips_);
    this -> fixed_actions = my::BitField(this -> n_act_);
    this -> eliminated_first_archievers = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> nvarstrips_));
    this -> fixed_first_archievers = std::vector<my::BitField>(this -> n_act_, my::BitField(this -> nvarstrips_));
    this -> fixed_var_timestamps = nullptr;
    this -> fixed_act_timestamps = nullptr;

    lprint_info("Created HPLUS_instance.");

}

int HPLUS_instance::get_version() const { return this -> version_; }

bool HPLUS_instance::unitary_cost() const { return !this -> use_costs_; }

unsigned int HPLUS_instance::get_nvar() const { return this -> n_var_; }

unsigned int HPLUS_instance::get_nact() const { return this -> n_act_; }

unsigned int HPLUS_instance::get_nvar_strips() const { return this -> nvarstrips_; }

const std::vector<HPLUS_action>& HPLUS_instance::get_actions() const { return this -> actions_; }

const my::BitField& HPLUS_instance::get_gstate() const { return this -> goal_state_; }

void HPLUS_instance::update_best_solution(const std::vector<unsigned int>& solution, const unsigned int cost) {

    #if HPLUS_INTCHECK
    my::BitField dbcheck = my::BitField(this -> n_act_);
    unsigned int costcheck = 0;
    my::assert(solution.size() <= this -> n_act_, "Solution has more actions that there actually exists.");             // check that there aren't more actions that there exists
    my::BitField feas_checker(this -> nvarstrips_);
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

    HPLUS_env.logger.print("Solution cost: %d", this -> best_cost_);
    for(auto act_i : this -> best_solution_) HPLUS_env.logger.print("(%s)", this -> actions_[act_i].get_name().c_str());

}

unsigned int HPLUS_instance::get_best_cost() const { return this -> best_cost_; }

void HPLUS_instance::parse_inst_file_(std::ifstream* ifs) {

    lprint_info("Parsing SAS file.");

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
    std::getline(*ifs, line);   // n_var
    my::assert(my::isint(line, 0), "Corrupted file.");
    this -> n_var_ = std::stoi(line);
    std::vector<unsigned int> var_ranges(this -> n_var_);
    this -> nvarstrips_ = 0;
    for (unsigned int var_i = 0; var_i < this -> n_var_; var_i++) {
        // process each variable
        std::getline(*ifs, line);   // begin_variable
        my::assert(line == "begin_variable", "Corrupted file.");
        std::string name;
        std::getline(*ifs, name);   // variable name
        std::getline(*ifs, line);   // axiom layer (ignored)
        my::assert(line == "-1", "Axiom layer is not -1, this software is not made for this instance.");
        std::getline(*ifs, line);   // range of variable
        my::assert(my::isint(line, 0), "Corrupted file.");
        unsigned int range = stoi(line);
        this -> nvarstrips_ += range;
        std::vector<std::string> val_names(range);
        for (unsigned int j = 0; j < range; j++) {
            std::getline(*ifs, line);   // name for variable value
            val_names[j] = line;
        }
        var_ranges[var_i] = range;
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
    my::BitField istate = my::BitField(this -> nvarstrips_);
    for (unsigned int var_i = 0, c = 0; var_i < this -> n_var_; c += var_ranges[var_i], var_i++) {
        std::getline(*ifs, line);   // initial value of var_i
        my::assert(my::isint(line, 0, var_ranges[var_i] - 1), "Corrupted file.");
        unsigned int val = stoi(line);
        istate.set(c + val);
    }
    std::getline(*ifs, line);   // end_state
    my::assert(line == "end_state", "Corrupted file.");

    // removing initial state variables
    std::vector<unsigned int> post_istate_removal_offset(this -> nvarstrips_);
    int counter = 0;
    for (unsigned int i = 0; i < this -> nvarstrips_; i++) {
        if (istate[i]) counter++;
        post_istate_removal_offset[i] = counter;
    }
    this -> nvarstrips_ -= this -> n_var_;

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
            my::assert(my::isint(tokens[1], 0,var_ranges[var] - 1), "Corrupted file."); // variable value
            unsigned int value = stoi(tokens[1]);
            unsigned int var_strips = value;
            for (unsigned int i = 0; i < var; var_strips += var_ranges[i], i++) {}
            if (!istate[var_strips]) act_pre.set(var_strips - post_istate_removal_offset[var_strips]);
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