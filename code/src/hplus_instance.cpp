/**
 * @file hplus_instance.cpp
 * @brief Methods implementation of the hplus_instance.hpp interface
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include "hplus_instance.hpp"
#include "utils.hpp"
#include "bs.hxx"
#include "log.hxx"
#include <algorithm>
#include <fstream>

void hplus::print_stats(const statistics& _s, const logger& _l) {
    #if HPLUS_VERBOSE < 5
    return
    #endif
    _l.print("\n\n--------------------------------------------------");
    _l.print("-----------------   Statistics   -----------------");
    _l.print("--------------------------------------------------\n");
    _l.print(" >>  Parsing time                  %10.3fs  <<", _s.parsing);
    _l.print(" >>  Problem simplification time   %10.3fs  <<", _s.optimization);
    _l.print(" >>  Heuristic time                %10.3fs  <<", _s.heuristic);
    _l.print(" >>  Model building time           %10.3fs  <<", _s.build);
    _l.print(" >>  CPLEX execution time          %10.3fs  <<", _s.execution);
    _l.print(" >>  CPLEX callback time           %10.3fs  <<", _s.callback);
    _l.print(" >>  Total time                    %10.3fs  <<", _s.total);
    _l.print("\n\n");
}

static inline void init(hplus::instance& _i) {
    _i = (hplus::instance){
        .unary_costs = false,
        .n = 0,
        .m = 0,
        .n_opt = 0,
        .m_opt = 0,
        .var_ranges = std::vector<size_t>(0),
        .actions = std::vector<hplus::action>(0),
        .goal = binary_set(),
        .best_solution = std::vector<size_t>(0),
        .best_cost = UINT_MAX,
        .var_e = binary_set(),
        .var_f = binary_set(),
        .act_e = binary_set(),
        .act_f = binary_set(),
        .fadd_e = std::vector<binary_set>(0),
        .fadd_f = std::vector<binary_set>(0),
        .var_t = std::vector<int>(0),
        .act_t = std::vector<int>(0),
        .act_inv = std::vector<std::vector<size_t>>(0),
        .var_opt_conv = std::vector<size_t>(0),
        .act_opt_conv = std::vector<size_t>(0),
        .fadd_checkpoint = std::vector<size_t>(0),
        .act_cpxtoidx = std::vector<size_t>(0),
        .act_with_eff = std::vector<std::vector<size_t>>(0),
        .act_with_pre = std::vector<std::vector<size_t>>(0)
    };
}
static inline bool is_deletefree(const hplus::instance& _i, std::vector<std::vector<std::pair<size_t, size_t>>>& _tmp_act_eff) {
    for (auto r : _i.var_ranges) if (r != 2) return false;
    std::vector<size_t> check(_i.n, -1);
    for (auto act_eff : _tmp_act_eff) {
        for (auto pair : act_eff) {
            if (check[pair.first] == -1) check[pair.first] = pair.second;
            else if (check[pair.first] != pair.second) return false;
        }
    }
    return true;
}
static inline void parse_inst_file(hplus::instance& _i, const hplus::environment& _e, hplus::statistics& _s, const logger& _l) {

    // ====================================================== //
    // ================== PARSING SAS FILE ================== //
    // ====================================================== //

    _PRINT_INFO("Parsing SAS file.");

    std::ifstream ifs(_e.input_file.c_str(), std::ifstream::in);
    assert(ifs.good());

    _s.parsing = _e.time_limit - _e.timer.get_time();
    std::string line;

    // * version section
    std::getline(ifs, line);   // begin_version
    if(line != "begin_version") _l.raise_error("Corrupted file");
    std::getline(ifs, line);   // version_number (ignored)
    if(!isint(line)) _l.raise_error("Corrupted file");
    std::getline(ifs, line);   // end_version
    if(line != "end_version") _l.raise_error("Corrupted file");

    // * metric section
    std::getline(ifs, line);   // begin_metric
    if(line != "begin_metric") _l.raise_error("Corrupted file");
    std::getline(ifs, line);   // metric
    if(!isint(line, 0, 1)) _l.raise_error("Corrupted file");
    _i.unary_costs = stoi(line) != 1;
    std::getline(ifs, line);   // end_metric
    if(line != "end_metric") _l.raise_error("Corrupted file");

    // * variables section
    _PRINT_WARN("Ignoring axiom layers.");
    std::getline(ifs, line);   // _i.n
    if(!isint(line, 0)) _l.raise_error("Corrupted file");
    size_t nvar = stoi(line);
    _i.n = nvar;
    _i.var_ranges = std::vector<size_t>(_i.n);
    for (size_t var_i = 0; var_i < _i.n; var_i++) {
        // process each variable
        std::getline(ifs, line);   // begin_variable
        if(line != "begin_variable") _l.raise_error("Corrupted file");
        std::getline(ifs, line);   // variable name (ignored)
        std::getline(ifs, line);   // axiom layer (ignored)
        if(line != "-1") _l.raise_error("Axiom layer is not -1, this software is not made for this instance.");
        std::getline(ifs, line);   // range of variable
        if(!isint(line, 0)) _l.raise_error("Corrupted file");
        size_t range = stoi(line);
        _i.var_ranges[var_i] = range;
        for (size_t j = 0; j < range; j++) std::getline(ifs, line);   // name for variable value (ignored)
        std::getline(ifs, line);   // end_variable
        if(line != "end_variable") _l.raise_error("Corrupted file");
    }
    
    // * mutex section (ignored)
    _PRINT_WARN("Ignoring mutex section.");
    std::getline(ifs, line);   // number of mutex groups
    if(!isint(line, 0)) _l.raise_error("Corrupted file");
    size_t nmgroups = stoi(line);
    for (size_t i = 0; i < nmgroups; i++) {
        std::getline(ifs, line);   // begin_mutex_group
        if(line != "begin_mutex_group") _l.raise_error("Corrupted file");
        while (line != "end_mutex_group") {
            std::getline(ifs, line); // reach end_mutex_group (ignore all content)
            if(line == "begin_state") _l.raise_error("Corrupted file");
        }
    }

    // * initial state section
    std::getline(ifs, line);   // begin_state
    if(line != "begin_state") _l.raise_error("Corrupted file");
    std::vector<int> tmp_istate(_i.n);
    for (size_t var_i = 0; var_i < _i.n; var_i++) {
        std::getline(ifs, line);   // initial value of var_i
        if(!isint(line, 0, _i.var_ranges[var_i] - 1)) _l.raise_error("Corrupted file");
        size_t val = stoi(line);
        tmp_istate[var_i] = val;
    }
    std::getline(ifs, line);   // end_state
    if(line != "end_state") _l.raise_error("Corrupted file");

    // * goal state section
    std::getline(ifs, line);   // begin_goal
    if(line != "begin_goal") _l.raise_error("Corrupted file");
    std::vector<int> tmp_goal(_i.n, -1);
    std::getline(ifs, line);   // number of goals
    if(!isint(line, 0, _i.n)) _l.raise_error("Corrupted file");
    size_t ngoals = stoi(line);
    for (size_t _ = 0; _ < ngoals; _++) {
        // parsing each goal
        std::vector<std::string> tokens;
        std::getline(ifs, line);   // pair 'variable goal'
        tokens = split_string(line, ' ');
        if(tokens.size() != 2) _l.raise_error("Corrupted file");
        if(!isint(tokens[0], 0, _i.n - 1)) _l.raise_error("Corrupted file"); // variable index
        size_t var = stoi(tokens[0]);
        if(!isint(tokens[1], 0, _i.var_ranges[var] - 1)) _l.raise_error("Corrupted file"); // variable goal
        size_t value = stoi(tokens[1]);
        tmp_goal[var] = value;
    }
    std::getline(ifs, line);   // end_goal
    if(line != "end_goal") _l.raise_error("Corrupted file");

    // * operator (actions) section
    _PRINT_WARN("Ignoring effect conditions.");
    std::getline(ifs, line);   // n_act
    if(!isint(line, 0)) _l.raise_error("Corrupted file");
    _i.m = stoi(line);
    _i.actions = std::vector<hplus::action>(_i.m);
    std::vector<std::vector<std::pair<size_t, size_t>>> tmp_act_pre(_i.m), tmp_act_eff(_i.m);
    for (size_t act_i = 0; act_i < _i.m; act_i++) {
        // process each action
        std::getline(ifs, line);   // begin_operator
        if(line != "begin_operator") _l.raise_error("Corrupted file");
        std::getline(ifs, line);   // symbolic action name
        std::string name = line;
        std::vector<int> act_pre(_i.n, -1);
        std::getline(ifs, line);   // number of prevail conditions
        if(!isint(line, 0, _i.n)) _l.raise_error("Corrupted file");
        size_t n_pre = stoi(line);
        for (size_t pre_i = 0; pre_i < n_pre; pre_i++) {
            // parsing each prevail condition
            std::vector<std::string> tokens;
            std::getline(ifs, line);   // pair 'variable value'
            tokens = split_string(line, ' ');
            if(tokens.size() != 2) _l.raise_error("Corrupted file");
            if(!isint(tokens[0], 0, _i.n - 1)) _l.raise_error("Corrupted file"); // variable index
            size_t var = stoi(tokens[0]);
            if(!isint(tokens[1], 0, _i.var_ranges[var] - 1)) _l.raise_error("Corrupted file"); // variable value
            size_t value = stoi(tokens[1]);
            act_pre[var] = value;
        }
        std::getline(ifs, line);   // number of effects
        if(!isint(line, 0)) _l.raise_error("Corrupted file");
        size_t n_eff = stoi(line);
        std::vector<int> act_eff(_i.n, -1);
        for (size_t eff_i = 0; eff_i < n_eff; eff_i++) {
            // parsing each effect
            std::getline(ifs, line);   // effect line
            std::vector<std::string> tokens;
            tokens = split_string(line, ' ');
            if(tokens.size() != 4) _l.raise_error("This program won't handle effect conditions."); // not expecting effect conditions
            if(!isint(tokens[0], 0, 0)) _l.raise_error("This program won't handle effect conditions."); // number of effect conditions (ignored and check to be 0)
            if(!isint(tokens[1], 0, _i.n - 1)) _l.raise_error("Corrupted file");   // variable affected by the action
            size_t var = stoi(tokens[1]);
            if(!isint(tokens[2], -1, _i.var_ranges[var] - 1)) _l.raise_error("Corrupted file");    // precondition of the variable
            int pre_val = stoi(tokens[2]);
            if(!isint(tokens[3], 0, _i.var_ranges[var] - 1)) _l.raise_error("Corrupted file"); // effect of the variable
            size_t eff_val = stoi(tokens[3]);
            if (pre_val >= 0) act_pre[var] = pre_val;
            act_eff[var] = eff_val;
        }
        std::getline(ifs, line);   // action cost
        if(!isint(line)) _l.raise_error("Corrupted file");
        unsigned int cost = 1;
        if (!_i.unary_costs) cost = stoi(line);
        std::getline(ifs, line);   // end_operator
        if(line != "end_operator") _l.raise_error("Corrupted file");
        _i.actions[act_i] = (hplus::action){.cost=cost, .name=name};

        for (size_t i = 0; i < _i.n; i++) {
            if (act_pre[i] >= 0) tmp_act_pre[act_i].emplace_back(i, act_pre[i]);
            if (act_eff[i] >= 0) tmp_act_eff[act_i].emplace_back(i, act_eff[i]);
        }
    }

    _PRINT_WARN("Ignoring axiom section.");

    ifs.close();

    // ====================================================== //
    // ================== BINARY EXPANSION ================== //
    // ====================================================== //

    binary_set istate;
    if (is_deletefree(_i, tmp_act_eff)) {
        #if HPLUS_VERBOSE >= 20
        _PRINT_INFO("Detected delete-free instance, skipping binary expansion.");
        #endif
        // TODO: Parsing delete-free instance
        todo(_l, "Parsing delete-free instance");
    } else {
        // binary expansion
        #if HPLUS_VERBOSE >= 20
        _PRINT_INFO("Performing binary expansion.");
        #endif
        size_t n_exp = 0;
        std::vector<size_t> offsets(_i.n);
        for (size_t i = 0; i < _i.n; i++) {
            offsets[i] = n_exp;
            n_exp += _i.var_ranges[i];
        }
        istate = binary_set(n_exp);
        _i.goal = binary_set(n_exp);
        for (size_t i = 0; i < _i.n; i++) {
            istate.add(offsets[i] + tmp_istate[i]);
            if (tmp_goal[i] >= 0) _i.goal.add(offsets[i] + tmp_goal[i]);
        }
        _i.n = n_exp;
        for (size_t i = 0; i < _i.m; i++) {
            binary_set act_pre(_i.n), act_eff(_i.n);
            for (auto pair : tmp_act_pre[i]) act_pre.add(offsets[pair.first] + pair.second);
            for (auto pair : tmp_act_eff[i]) act_eff.add(offsets[pair.first] + pair.second);
            _i.actions[i].pre = act_pre;
            _i.actions[i].pre_sparse = act_pre.sparse();
            _i.actions[i].eff = act_eff;
            _i.actions[i].eff_sparse = act_eff.sparse();
        }
    }

    // ====================================================== //
    // ================ INITIAL STATE REMOVAL =============== //
    // ====================================================== //

    #if HPLUS_VERBOSE >= 20
    _PRINT_INFO("Removing initial state variables.");
    #endif
    std::vector<size_t> istate_offsets(_i.n);
    for (size_t i = 0, c = 0; i < _i.n; i++) {
        if (istate[i]) c++;
        istate_offsets[i] = c;
    }
    _i.n -= nvar;
    binary_set goal_opt(_i.n);
    for (auto var : _i.goal) if (!istate[var]) goal_opt.add(var - istate_offsets[var]);
    _i.goal = goal_opt;
    for (size_t i = 0; i < _i.m; i++) {
        binary_set act_pre(_i.n), act_eff(_i.n);
        for (auto var : _i.actions[i].pre_sparse) if (!istate[var]) act_pre.add(var - istate_offsets[var]);
        for (auto var : _i.actions[i].eff_sparse) if (!istate[var]) act_eff.add(var - istate_offsets[var]);
        _i.actions[i].pre = act_pre;
        _i.actions[i].pre_sparse = act_pre.sparse();
        _i.actions[i].eff = act_eff;
        _i.actions[i].eff_sparse = act_eff.sparse();
    }
    
    _s.parsing = _e.timer.get_time();
}
static inline void init_instance_opt(hplus::instance& _i) {
    _i.n_opt = _i.n;
    _i.m_opt = _i.m;
    _i.var_e = binary_set(_i.n);
    _i.var_f = binary_set(_i.n);
    _i.act_e = binary_set(_i.m);
    _i.act_f = binary_set(_i.m);
    _i.fadd_e = std::vector<binary_set>(_i.m, binary_set(_i.n));
    _i.fadd_f = std::vector<binary_set>(_i.m, binary_set(_i.n));
    _i.var_t = std::vector<int>(_i.n, -1);
    _i.act_t = std::vector<int>(_i.m, -1);
    _i.act_inv = std::vector<std::vector<size_t>>(_i.m, std::vector<size_t>());
    _i.var_opt_conv = std::vector<size_t>(_i.n);
    for (size_t idx = 0; idx < _i.n; idx++) _i.var_opt_conv[idx] = idx;
    _i.act_opt_conv = std::vector<size_t>(_i.m);
    for (size_t idx = 0; idx < _i.m; idx++) _i.act_opt_conv[idx] = idx;
    _i.fadd_checkpoint = std::vector<size_t>(_i.m);
    for (size_t idx = 0; idx < _i.m; idx++) _i.fadd_checkpoint[idx] = idx * _i.n;
    _i.act_cpxtoidx = std::vector<size_t>(_i.m);
    for (size_t idx = 0; idx < _i.m; idx++) _i.act_cpxtoidx[idx] = idx;
}
void hplus::create_instance(instance& _i, const environment& _e, statistics& _s, const logger& _l) {
    init(_i);
    parse_inst_file(_i, _e, _s, _l);
    init_instance_opt(_i);

    _PRINT_INFO("Created HPLUS_instance.");
}

binary_set hplus::var_remaining(const instance& _i) { return !_i.var_e; }
binary_set hplus::act_remaining(const instance& _i) { return !_i.act_e; }

void hplus::update_sol(instance& _i, const std::vector<size_t> _s, const unsigned int _c, const logger& _l) {
    binary_set dbcheck = binary_set(_i.m);
    unsigned int costcheck = 0;
    _ASSERT(_s.size() <= _i.m);      // check that there aren't more actions that there exists
    binary_set feas_checker(_i.n);
    for (auto act_i : _s) {
        _ASSERT(act_i < _i.m);       // check that the solution only contains existing actions
        _ASSERT(!dbcheck[act_i]);        // check that there are no duplicates
        dbcheck.add(act_i);
        _ASSERT(feas_checker.contains(_i.actions[act_i].pre));       // check if the preconditions are respected at each step
        feas_checker |= _i.actions[act_i].eff;
        costcheck += _i.actions[act_i].cost;
    }
    _ASSERT(feas_checker.contains(_i.goal));     // check if the solution leads to the goal state
    _ASSERT(costcheck == _c);    // check if the cost is the declared one

    if (_c >= _i.best_cost) return;

    _i.best_solution = std::vector<size_t>(_s);
    _i.best_cost = _c;

    _l.print_info("Updated best solution - Cost: %10u.", _i.best_cost);
}
void hplus::print_sol(const instance& _i, const logger& _l) {
    _l.print("Solution cost: %10u", _i.best_cost);
    for(auto act_i : _i.best_solution) _l.print("(%s)", _i.actions[act_i].name.c_str());
}

static inline void landmark_extraction(hplus::instance& _i, std::vector<binary_set>& _lm, binary_set& _fl, binary_set& _al, const logger& _l) {
    #if HPLUS_VERBOSE >= 20
    _PRINT_INFO("(debug) Extracting landmarks.");
    #endif

    for (size_t var_i = 0; var_i < _i.n; var_i++) _lm[var_i].add(_i.n);

    binary_set s_set(_i.n);

    std::deque<size_t> actions_queue;
    for (size_t act_i = 0; act_i < _i.m; act_i++) if(s_set.contains(_i.actions[act_i].pre)) actions_queue.push_back(act_i);

    // list of actions that have as precondition variable p
    std::vector<std::vector<size_t>> act_with_pre(_i.n);
    for (size_t var_i = 0; var_i < _i.n; var_i++) for (size_t act_i = 0; act_i < _i.m; act_i++) if (_i.actions[act_i].pre[var_i]) act_with_pre[var_i].push_back(act_i);

    while(!actions_queue.empty()) {

        const hplus::action a = _i.actions[actions_queue.front()];
        actions_queue.pop_front();

        const auto& pre_sparse = a.pre_sparse;
        const auto& add_sparse = a.eff_sparse;

        for (auto var_i : add_sparse) {

            s_set.add(var_i);

            binary_set x = binary_set(_i.n + 1);
            for (auto var_j : add_sparse) x.add(var_j);

            for (auto var_j : pre_sparse) {
                // if variable var_i' has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set the flag
                if (_lm[var_j][_i.n]) {
                    x.add(_i.n);
                    // if x is now full we can exit, since all further unions won't change x
                    break;
                } else x |= _lm[var_j];
            }

            // we then check if L[var_i] != X, and if they are the same we skip,
            // if X = P, then (X intersection L[P]) = L[P], hence we can already skip
            if (!x[_i.n]) {

                // if the set for variable var_i is the full set of variables,
                // the intersection generates back x -> we can skip the intersection
                if (!_lm[var_i][_i.n]) x &= _lm[var_i];

                // we already know that x is not the full set now, so if
                // the set for variable var_i is the full set, we know that x is not
                // equal to the set for variable var_i -> we can skip the check
                if (_lm[var_i][_i.n] || x != _lm[var_i]) {

                    _lm[var_i] = x;
                    for (auto act_i : act_with_pre[var_i])
                        if (s_set.contains(_i.actions[act_i].pre) && std::find(actions_queue.begin(), actions_queue.end(), act_i) == actions_queue.end())
                            actions_queue.push_back(act_i);

                }

            }

        }

    }

    // list of actions that have as effect variable p
    std::vector<std::vector<size_t>> act_with_eff(_i.n);
    for (auto var_i : !_fl) for (size_t act_i = 0; act_i < _i.m; act_i++) if (_i.actions[act_i].eff[var_i]) act_with_eff[var_i].push_back(act_i);

    // popolate _fl and _al sets
    for (auto var_i : _i.goal) {
        for (auto var_j : !_fl) {
            if (_lm[var_i][var_j] || _lm[var_i][_i.n]) {
                _fl.add(var_j);
                size_t count = 0, cand_act;
                for (auto act_i : act_with_eff[var_j]) {
                    cand_act = act_i;
                    count++;
                    if (count > 1) break;
                }
                if (count == 1) _al.add(cand_act);
            }
        }
    }
    _i.var_f |= _fl;
    _i.act_f |= _al;
}
static inline void fadd_extraction(hplus::instance& _i, const std::vector<binary_set>& _lm, std::vector<binary_set>& _fadd, const logger& _l) {
    #if HPLUS_VERBOSE >= 20
    _PRINT_INFO("(debug) Extracting first adders.");
    #endif

    // list of fact landmarks for each variable
    std::vector<std::vector<size_t>> var_flm_sparse(_i.n);
    for (size_t var_i = 0; var_i < _i.n; var_i++) for (size_t var_j = 0; var_j < _i.n; var_j++) if (_lm[var_i][var_j] || _lm[var_i][_i.n]) var_flm_sparse[var_i].push_back(var_j);

    // compute the set of first adders
    for (size_t act_i = 0; act_i < _i.m; act_i++) {
        // f_lm_a is the set of fact landmarks of action act_i
        binary_set f_lm_a(_i.n);
        for (auto var_i : _i.actions[act_i].pre) for (auto i : var_flm_sparse[var_i]) f_lm_a.add(i);
        // _fadd[a] := { p in add(a) s.t. p is not a fact landmark for a }
        _fadd[act_i] |= (_i.actions[act_i].eff & !f_lm_a);
    }
    for (size_t act_i = 0; act_i < _i.m; act_i++) _i.fadd_e[act_i] |= (_i.actions[act_i].eff & !_fadd[act_i]);
}
static inline void relevance_analysis(hplus::instance& _i, binary_set& _fl, std::vector<binary_set>& _fadd, const logger& _l) {
    #if HPLUS_VERBOSE >= 20
    _PRINT_INFO("(debug) Relevance analysis.");
    #endif

    binary_set relevant_variables = binary_set(_i.n);
    binary_set relevant_actions = binary_set(_i.m);

    // compute first round of relevand variables and actions
    for (size_t act_i = 0; act_i < _i.m; act_i++) {
        if (_fadd[act_i].intersects(_i.goal)) {
            relevant_variables |= _i.actions[act_i].pre;
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
            if (_i.actions[act_i].eff.intersects(relevant_variables)) {
                relevant_actions.add(act_i);
                relevant_variables |= _i.actions[act_i].pre;
                new_relevant_actions.push_back(act_i);
                new_act = true;
            }
        }
        auto it = std::set_difference(cand_actions_sparse.begin(), cand_actions_sparse.end(), new_relevant_actions.begin(), new_relevant_actions.end(), cand_actions_sparse.begin());
        cand_actions_sparse.resize(it - cand_actions_sparse.begin());
    }
    relevant_variables |= _i.goal;

    // eliminate actions and variables that are not relevant (or landmarks)
    _i.var_e |= (!relevant_variables - _fl);
    _i.act_e |= !relevant_actions;
}
static inline void dominated_actions_elimination(hplus::instance& _i, std::vector<binary_set>& _lm, std::vector<binary_set>& _fadd, const logger& _l) {
    #if HPLUS_VERBOSE >= 20
    _PRINT_INFO("(debug) Extracting dominated actions.");
    #endif

    auto remaining_var_sparse = hplus::var_remaining(_i).sparse();
    std::vector<binary_set> act_flm = std::vector<binary_set>(_i.m, binary_set(_i.n));
    std::vector<std::vector<size_t>> var_flm_sparse(_i.n);

    // compute the landmarks for each variable remaining
    for (auto var_i : remaining_var_sparse)
        for (auto var_j : remaining_var_sparse)
            if (_lm[var_i][var_j] || _lm[var_i][_i.n])
                var_flm_sparse[var_i].push_back(var_j);
                
    // compute the landmarks for each action remaining
    for (size_t act_i = 0; act_i < _i.m; act_i++)
        for (auto var_i : _i.actions[act_i].pre_sparse) if (hplus::var_remaining(_i)[var_i])
            for (auto i : var_flm_sparse[var_i])
                act_flm[act_i].add(i);

    // find efficently all actions that satisfy point 1) of Proposition 4 of in Imai's Paper
    bs_searcher subset_finder = bs_searcher(_i.n);
    for (auto act_i : hplus::act_remaining(_i)) subset_finder.add(act_i, _fadd[act_i]);

    binary_set dominated_actions(_i.m);

    // find all dominated actions and eliminate them
    for (auto dominant_act : hplus::act_remaining(_i)) if (!dominated_actions[dominant_act]) {
        for (auto dominated_act : subset_finder.find_subsets(_fadd[dominant_act])) if (hplus::act_remaining(_i)[dominated_act]) {
            
            if (_i.act_f[dominated_act] || dominant_act == dominated_act || _i.actions[dominant_act].cost > _i.actions[dominated_act].cost || !act_flm[dominated_act].contains(_i.actions[dominant_act].pre)) continue;

            dominated_actions.add(dominated_act);
            _i.act_e.add(dominated_act);

        }

    }
}
static inline void immediate_action_application(hplus::instance& _i, const hplus::environment& _e, binary_set& _al, const logger& _l) {
    #if HPLUS_VERBOSE >= 20
    _PRINT_INFO("(debug) Immediate action application.");
    #endif

    binary_set current_state(_i.n);
    binary_set actions_left = hplus::act_remaining(_i);

    // keep looking until no more actions can be applied
    int counter = 0;
    bool found_next_action = true;
    while (found_next_action) {
        found_next_action = false;

        for (auto act_i : actions_left) {
            const auto& pre = _i.actions[act_i].pre & hplus::var_remaining(_i);
            const auto& eff = _i.actions[act_i].eff & hplus::var_remaining(_i);

            if (current_state.contains(pre) && (_al[act_i] || _i.actions[act_i].cost == 0)) {

                actions_left.remove(act_i);
                _i.act_f.add(act_i);
                if (_e.alg == HPLUS_CLI_ALG_IMAI) _i.act_t[act_i] = counter;
                _i.var_f |= pre;
                for (auto var_i : eff) if (!current_state[var_i]) {
                    _i.var_f.add(var_i);
                    if (_e.alg == HPLUS_CLI_ALG_IMAI) _i.var_t[var_i] = counter+1;
                    _i.fadd_f[act_i].add(var_i);
                    for (auto act_j : actions_left) _i.fadd_e[act_j].add(var_i);
                }
                current_state |= eff;
                counter++;
                found_next_action = true;
            }
        }
    }
}
static inline void inverse_actions_extraction(hplus::instance& _i, const logger& _l) {
    #if HPLUS_VERBOSE >= 20
    _PRINT_INFO("(debug) Extracting inverse actions.");
    #endif

    bs_searcher subset_finder = bs_searcher(_i.n);

    // find efficiently all actions that satisfy point 2) of the Definition 1 in section 4.6 of Imai's paper
    const auto remaining_actions = hplus::act_remaining(_i).sparse();
    for (auto act_i : remaining_actions) subset_finder.add(act_i, _i.actions[act_i].eff);

    for (auto act_i : remaining_actions) {
        const auto& pre = _i.actions[act_i].pre;
        const auto& eff = _i.actions[act_i].eff;
        for (auto act_j : subset_finder.find_subsets(pre)) {
            if (_i.actions[act_j].pre.contains(eff)) {
                if (!_i.act_f[act_i]) _i.act_inv[act_i].push_back(act_j);
                if (!_i.act_f[act_j]) _i.act_inv[act_j].push_back(act_i);
            }
        }
    }
}
static inline void finish_opt(hplus::instance& _i) {
    size_t count = 0;
    for (auto var_i : hplus::var_remaining(_i)) _i.var_opt_conv[var_i] = count++;
    _i.n_opt = count;
    count = 0;
    for (auto act_i : hplus::act_remaining(_i)) {
        _i.act_opt_conv[act_i] = count;
        _i.act_cpxtoidx[count++] = act_i;
    }
    _i.m_opt = count;
    _i.fadd_checkpoint = std::vector<size_t>(_i.m_opt);
    for (size_t act_i = 0; act_i < _i.m_opt; act_i++) _i.fadd_checkpoint[act_i] = act_i * _i.n_opt;
    _ASSERT(!_i.var_f.intersects(_i.var_e));
    _ASSERT(!_i.act_f.intersects(_i.act_e));
}
void hplus::instance_optimization(instance& _i, const environment& _e, const logger& _l) {
    std::vector<binary_set> landmarks = std::vector<binary_set>(_i.n, binary_set(_i.n+1));
    binary_set fact_landmarks = binary_set(_i.n);
    binary_set act_landmarks = binary_set(_i.m);
    std::vector<binary_set> fadd = std::vector<binary_set>(_i.m, binary_set(_i.n));
    landmark_extraction(_i, landmarks, fact_landmarks, act_landmarks, _l);
    fadd_extraction(_i, landmarks, fadd, _l);
    relevance_analysis(_i, fact_landmarks, fadd, _l);
    dominated_actions_elimination(_i, landmarks, fadd, _l);
    immediate_action_application(_i, _e, act_landmarks, _l);
    if (_e.inv_act) inverse_actions_extraction(_i, _l);
    finish_opt(_i);
}

void hplus::prepare_faster_actsearch(instance& _i) {
    _i.act_with_pre = std::vector<std::vector<size_t>>(_i.n);
    _i.act_with_eff = std::vector<std::vector<size_t>>(_i.n);
    std::vector<size_t> rem_var = var_remaining(_i).sparse(), rem_act = act_remaining(_i).sparse();
    for (auto var_i : rem_var) for (auto act_i : rem_act) {
        if (_i.actions[act_i].pre[var_i]) _i.act_with_pre[var_i].push_back(act_i);
        if (_i.actions[act_i].eff[var_i]) _i.act_with_eff[var_i].push_back(act_i);
    }
}