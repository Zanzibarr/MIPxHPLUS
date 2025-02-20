#include <cplex.h>
#include <set>
#include <algorithm>
#include "algorithms.hpp"
#include "utils.hpp"

// ##################################################################### //
// ############################ CPLEX UTILS ############################ //
// ##################################################################### //

void cpx_init(CPXENVptr& env, CPXLPptr& lp, const hplus::environment& _e) {
    int cpxerror;
    env = CPXopenCPLEX(&cpxerror); assert(!cpxerror);
    lp = CPXcreateprob(env, &cpxerror, "HPLUS"); assert(!cpxerror);
    // log file
    assert(!CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_OFF));
    assert(!CPXsetlogfilename(env, (HPLUS_CPLEX_OUTPUT_DIR"/log/"+_e.run_name+".log").c_str(), "w"));
    assert(!CPXsetintparam(env, CPX_PARAM_CLONELOG, -1));
    // tolerance
    assert(!CPXsetdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, 0));
    // memory/size limits
    assert(!CPXsetdblparam(env, CPXPARAM_MIP_Limits_TreeMemory, 12000));
    assert(!CPXsetdblparam(env, CPXPARAM_WorkMem, 4096));
    assert(!CPXsetintparam(env, CPXPARAM_MIP_Strategy_File, 3));
    // terminate condition
    assert(!CPXsetterminate(env, &global_terminate));
}

void cpx_close(CPXENVptr& env, CPXLPptr& lp) {
    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);
}

bool parse_cpx_status(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, hplus::environment& _e, const logger& _l) {
    switch ( int cpxstatus = CPXgetstat(env, lp) ) {
        case CPXMIP_TIME_LIM_FEAS:      // exceeded time limit, found intermediate solution
            _e.sol_s = solution_status::FEAS;
            return true;
        case CPXMIP_TIME_LIM_INFEAS:    // exceeded time limit, no intermediate solution found
            if (!_e.warm_start) _e.sol_s = solution_status::NOTFOUND;
            return false;
        case CPXMIP_INFEASIBLE:         // proven to be unfeasible
            _e.sol_s = solution_status::INFEAS;
            return false;
        case CPXMIP_ABORT_FEAS:         // terminated by user, found solution
            _e.sol_s = solution_status::FEAS;
            return true;
        case CPXMIP_ABORT_INFEAS:       // terminated by user, not found solution
            if (!_e.warm_start) _e.sol_s = solution_status::NOTFOUND;
            return false;
        case CPXMIP_OPTIMAL_TOL:        // found optimal within the tollerance
            _PRINT_WARN("Found optimal within the tolerance.");
            _e.sol_s = solution_status::OPT;
            return true;
        case CPXMIP_OPTIMAL:            // found optimal
            _e.sol_s = solution_status::OPT;
            return true;
        default:                        // unhandled status
            _l.raise_error("Error in tsp_cplex: unhandled cplex status: %d.", cpxstatus);
            return false;
    }
}

// ##################################################################### //
// ############################# HEURISTICS ############################ //
// ##################################################################### //

static inline bool greedycost(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    binary_set state(_i.n);
    _c = 0;
    // greedy choice
    auto find_best_act = [&_i](const std::vector<size_t>& _candidates) {
        int best_act = -1;
        double best_cost = INFINITY;
        for (auto act_i : _candidates) {
            if (_i.actions[act_i].cost < best_cost) {
                best_act = act_i;
                best_cost = _i.actions[act_i].cost;
            }
        }
        _ASSERT(best_act != -1);
        return best_act;
    };
    // binary_set searcher for faster actions lookup
    bs_searcher feasible_actions = bs_searcher(_i.n);
    for (auto act_i : hplus::act_remaining(_i)) feasible_actions.add(act_i, _i.actions[act_i].pre);
    while (!state.contains(_i.goal)) {
        std::vector<size_t> candidates = feasible_actions.find_subsets(state);
        if (candidates.empty()) return false;
        size_t choice = find_best_act(candidates);
        _s.push_back(choice);
        _c += _i.actions[choice].cost;
        state |= _i.actions[choice].eff;
        feasible_actions.remove(choice, _i.actions[choice].pre);
    }
    return true;
}

static inline bool greedycxe(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    binary_set state(_i.n);
    const auto var_rem = hplus::var_remaining(_i);
    _c = 0;
    // greedy choice
    auto find_best_act = [&_i, &state, &var_rem](const std::vector<size_t>& _candidates) {
        int best_act = -1;
        double best_cxe = INFINITY;
        for (auto act_i : _candidates) {
            int neff = 0;
            for (auto var_i : _i.actions[act_i].eff_sparse) if (!state[var_i] && var_rem[var_i]) neff++;
            if (neff == 0) continue;
            double cxe = ((double) _i.actions[act_i].cost) / neff; 
            if (cxe < best_cxe) {
                best_act = act_i;
                best_cxe = _i.actions[act_i].cost;
            }
        }
        _ASSERT(best_act != -1);
        return best_act;
    };
    // binary_set searcher for faster actions lookup
    bs_searcher feasible_actions = bs_searcher(_i.n);
    for (auto act_i : hplus::act_remaining(_i)) feasible_actions.add(act_i, _i.actions[act_i].pre);
    while (!state.contains(_i.goal)) {
        std::vector<size_t> candidates = feasible_actions.find_subsets(state);
        if (candidates.empty()) return false;
        size_t choice = find_best_act(candidates);
        _s.push_back(choice);
        _c += _i.actions[choice].cost;
        state |= _i.actions[choice].eff;
        feasible_actions.remove(choice, _i.actions[choice].pre);
    }
    return true;
}

static inline bool randheur(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    binary_set state(_i.n);
    _c = 0;
    // binary_set searcher for faster actions lookup
    bs_searcher feasible_actions = bs_searcher(_i.n);
    for (auto act_i : hplus::act_remaining(_i)) feasible_actions.add(act_i, _i.actions[act_i].pre);
    while (!state.contains(_i.goal)) {
        std::vector<size_t> candidates = feasible_actions.find_subsets(state);
        if (candidates.empty()) return false;
        size_t choice = candidates[rand() % candidates.size()];
        _s.push_back(choice);
        _c += _i.actions[choice].cost;
        state |= _i.actions[choice].eff;
        feasible_actions.remove(choice, _i.actions[choice].pre);
    }
    return true;
}

static inline bool randr(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    // TODO: Choose on CLI
    size_t repetitions = 100;
    std::vector<size_t> random_solution[repetitions];
    unsigned int random_costs[repetitions];
    // TODO: Threads
    for (size_t i = 0; i < repetitions; i++) if(!randheur(_i, random_solution[i], random_costs[i], _l)) return false;
    size_t best_sol = 0;
    unsigned int best_cost = random_costs[0];
    for (size_t i = 1; i < repetitions; i++) if (random_costs[i] < best_cost) {
        best_sol = i;
        best_cost = random_costs[i];
    }
    _s = random_solution[best_sol];
    _c = best_cost;
    return true;
}

static inline bool hmax(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    // TODO: hmax heuristic
    return false;
}

static inline bool hadd(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    // TODO: hadd heuristic
    return false;
}

static inline bool relax(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    // TODO: Solve linear relaxation and apply random walk
    return false;
}

static inline bool localsearch(const hplus::instance& _i, bool (*_h)(const hplus::instance& _i, std::vector<size_t>& _s, unsigned int& _c, const logger& _l), std::vector<size_t>& _s, unsigned int& _c, const logger& _l) {
    if (!_h(_i, _s, _c, _l)) return false;
    // TODO: Local search on top of the initial solution
    return false;
}

void find_heuristic(hplus::instance& _i, hplus::environment& _e, const logger& _l) {

    std::vector<size_t> heur_solution;
    unsigned int heur_cost;

    srand(time(0));
    bool found = false;

    if(_e.heur == "greedycost") found = greedycost(_i, heur_solution, heur_cost, _l);
    else if (_e.heur == "greedycxe") found = greedycxe(_i, heur_solution, heur_cost, _l);
    else if (_e.heur == "rand") found = randheur(_i, heur_solution, heur_cost, _l);
    else if (_e.heur == "randr") found = randr(_i, heur_solution, heur_cost, _l);
    else if (_e.heur == "hmax") found = hmax(_i, heur_solution, heur_cost, _l);
    else if (_e.heur == "hadd") found = hadd(_i, heur_solution, heur_cost, _l);
    else if (_e.heur == "relax") found = relax(_i, heur_solution, heur_cost, _l);
    else if (_e.heur == "local-greedycost") found = localsearch(_i, greedycost, heur_solution, heur_cost, _l);
    else if (_e.heur == "local-greedycxe") found = localsearch(_i, greedycxe, heur_solution, heur_cost, _l);
    else if (_e.heur == "local-rand") found = localsearch(_i, randheur, heur_solution, heur_cost, _l);
    else if (_e.heur == "local-randr") found = localsearch(_i, randr, heur_solution, heur_cost, _l);
    else if (_e.heur == "local-hmax") found = localsearch(_i, hmax, heur_solution, heur_cost, _l);
    else if (_e.heur == "local-hadd") found = localsearch(_i, hadd, heur_solution, heur_cost, _l);
    else if (_e.heur == "local-relax") found = localsearch(_i, relax, heur_solution, heur_cost, _l);
    else _l.raise_error("The heuristic specified (%s) is not on the list of possible heuristics... Please read the Readme.md for instructions.", _e.heur.c_str());
    
    if (!found) {
        _e.sol_s = solution_status::INFEAS;
        return;
    }

    hplus::update_sol(_i, heur_solution, heur_cost, _l);
    _e.sol_s = solution_status::FEAS;

}

// ##################################################################### //
// ################################ IMAI ############################### //
// ##################################################################### //

void cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {

    const auto& rem_var = hplus::var_remaining(_i).sparse();
    const auto& rem_act = hplus::act_remaining(_i).sparse();

    // ====================================================== //
    // ============== TIGHTER TIMESTAMPS BOUNDS ============= //
    // ====================================================== //

    int timestamps_ubound = _i.m_opt;
    if (_e.imai_tight_bounds) {

        // number of variables
        if (_i.n_opt < timestamps_ubound) timestamps_ubound = _i.n_opt;

        // max number of steps to reach heuristic
        if (_e.heur != "none") {
            unsigned int min_act_cost = _i.actions[0].cost + 1;      // +1 to avoid it being 0
            size_t n_act_zerocost = 0;
            for (auto act : _i.actions) {
                if (act.cost == 0) n_act_zerocost++;
                else if (act.cost < min_act_cost) min_act_cost = act.cost;
            }
            int nsteps = _i.best_cost / min_act_cost + n_act_zerocost;
            if (nsteps < timestamps_ubound) timestamps_ubound = nsteps;
        }

    }

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //
    // (section 3 of Imai's paper)

    size_t curr_col = 0;

    double* objs = new double[_i.m_opt];
    double* lbs = new double[_i.m_opt];
    double* ubs = new double[_i.m_opt];
    char* types = new char[_i.m_opt];

    auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {

        delete[] types; types = nullptr;
        delete[] ubs; ubs = nullptr;
        delete[] lbs; lbs = nullptr;
        delete[] objs; objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];

    };
    
    // -------- actions ------- //
    size_t act_start = curr_col;
    size_t count = 0;
    for (auto act_i : rem_act) {
        objs[count] = _i.actions[act_i].cost;
        lbs[count] = _i.act_f[act_i] ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    _ASSERT(count == _i.m_opt);
    curr_col += _i.m_opt;

    assert(!CPXnewcols(env, lp, _i.m_opt, objs, lbs, ubs, types, nullptr));

    // --- action timestamps -- //
    size_t tact_start = curr_col;
    count = 0;
    for (auto act_i : rem_act) {
        objs[count] = 0;
        lbs[count] = _i.act_t[act_i] >= 0 ? _i.act_t[act_i] : 0;
        ubs[count] = _i.act_t[act_i] >= 0 ? _i.act_t[act_i] : timestamps_ubound-1;
        types[count++] = 'I';
    }
    _ASSERT(count == _i.m_opt);
    curr_col += _i.m_opt;

    assert(!CPXnewcols(env, lp, _i.m_opt, objs, lbs, ubs, types, nullptr));

    resize_cpx_arrays(_i.n_opt);
    
    // ------- variables ------ //
    size_t var_start = curr_col;
    count = 0;
    for (auto var_i : rem_var) {
        objs[count] = 0;
        lbs[count] = (_i.var_f[var_i] || _i.goal[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    _ASSERT(count == _i.n_opt);
    curr_col += _i.n_opt;

    assert(!CPXnewcols(env, lp, _i.n_opt, objs, lbs, ubs, types, nullptr));

    // -- variable timestamps - //
    size_t tvar_start = curr_col;
    count = 0;
    for (auto i : rem_var) {
        objs[count] = 0;
        lbs[count] = _i.var_t[i] >= 0 ? _i.var_t[i] : 0;
        ubs[count] = _i.var_t[i] >= 0 ? _i.var_t[i] : timestamps_ubound;
        types[count++] = 'I';
    }
    _ASSERT(count == _i.n_opt);
    curr_col += _i.n_opt;

    assert(!CPXnewcols(env, lp, _i.n_opt, objs, lbs, ubs, types, nullptr));

    // --- first archievers --- //
    size_t fa_start = curr_col;
    count = 0;
    for (auto act_i : rem_act) {
        size_t count_var = 0;
        for (auto var_i : rem_var) {
            objs[count_var] = 0;
            lbs[count_var] = _i.fadd_f[act_i][var_i] ? 1 : 0;
            ubs[count_var] = (!_i.actions[act_i].eff[var_i] || _i.fadd_e[act_i][var_i]) ? 0 : 1;
            types[count_var++] = 'B';
        }
        _ASSERT(count_var == _i.n_opt);
        curr_col += _i.n_opt;
        assert(!CPXnewcols(env, lp, _i.n_opt, objs, lbs, ubs, types, nullptr));
        count++;
    }

    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //
    // (section 3 of Imai's paper)

    // accessing cplex variables
    auto get_act_idx = [&_i, &act_start](size_t idx) { return act_start + _i.act_opt_conv[idx]; };
    auto get_tact_idx = [&_i, &tact_start](size_t idx) { return tact_start + _i.act_opt_conv[idx]; };
    auto get_var_idx = [&_i, &var_start](size_t idx) { return var_start + _i.var_opt_conv[idx]; };
    auto get_tvar_idx = [&_i, &tvar_start](size_t idx) { return tvar_start + _i.var_opt_conv[idx]; };
    auto get_fa_idx = [&_i, &fa_start](size_t act_idx, size_t var_idx) { return fa_start + _i.fadd_checkpoint[_i.act_opt_conv[act_idx]] + _i.var_opt_conv[var_idx]; };
    
    int* ind_c1 = new int[_i.m_opt + 1];
    double* val_c1 = new double[_i.m_opt + 1];
    int ind_c2_4[2], ind_c5[3];
    double val_c2_4[2], val_c5[3];
    const int nnz_c2_4 = 2, nnz_c5 = 3;
    const char sensel = 'L', sensee = 'E';
    const double rhs_c1_2_4 = 0, rhs_c5 = timestamps_ubound;
    const int begin = 0;

    std::vector<int*> ind_c3(_i.n_opt);
    std::vector<double*> val_c3(_i.n_opt);
    std::vector<int> nnz_c3(_i.n_opt);
    std::vector<double> rhs_c3(_i.n_opt);

    for (auto i : rem_var) {
        ind_c3[_i.var_opt_conv[i]] = new int[_i.m_opt + 1];
        val_c3[_i.var_opt_conv[i]] = new double[_i.m_opt + 1];
        nnz_c3[_i.var_opt_conv[i]] = 0;
        rhs_c3[_i.var_opt_conv[i]] = 0;
        ind_c3[_i.var_opt_conv[i]][nnz_c3[_i.var_opt_conv[i]]] = get_var_idx(i);
        val_c3[_i.var_opt_conv[i]][nnz_c3[_i.var_opt_conv[i]]] = 1;
        nnz_c3[_i.var_opt_conv[i]]++;
    }
    
    const auto& rem_var_set = hplus::var_remaining(_i);

    for (auto act_i : rem_act) {
        const auto& inverse_actions = _i.act_inv[act_i];
        const auto& pre = _i.actions[act_i].pre & rem_var_set;
        for (auto var_i : pre) {
            // constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
            ind_c1[0] = get_act_idx(act_i);
            val_c1[0] = 1;
            ind_c1[1] = get_var_idx(var_i);
            val_c1[1] = -1;
            int nnz0 = 2;
            // (section 4.6 of Imai's paper)
            for (size_t i = 0; i < inverse_actions.size(); i++) if (_i.actions[inverse_actions[i]].eff[var_i]) {
                ind_c1[nnz0] = get_fa_idx(inverse_actions[i], var_i);
                val_c1[nnz0++] = 1;
            }
            assert(!CPXaddrows(env, lp, 0, 1, nnz0, &rhs_c1_2_4, &sensel, &begin, ind_c1, val_c1, nullptr, nullptr));
            // constraint 4: t_vj <= t_a, vj in pre(a)
            ind_c2_4[0] = get_tvar_idx(var_i);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_tact_idx(act_i);
            val_c2_4[1] = -1;
            assert(!CPXaddrows(env, lp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
        }
        const auto& eff = _i.actions[act_i].eff & rem_var_set;
        for (auto var_i : eff) {
            // constraint 2: z_avj <= x_a, vj in eff(a)
            ind_c2_4[0] = get_fa_idx(act_i, var_i);
            val_c2_4[0] = 1;
            ind_c2_4[1] = get_act_idx(act_i);
            val_c2_4[1] = -1;
            assert(!CPXaddrows(env, lp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
            // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
            ind_c5[0] = get_tact_idx(act_i);
            val_c5[0] = 1;
            ind_c5[1] = get_tvar_idx(var_i);
            val_c5[1] = -1;
            ind_c5[2] = get_fa_idx(act_i, var_i);
            val_c5[2] = timestamps_ubound + 1;
            assert(!CPXaddrows(env, lp, 0, 1, nnz_c5, &rhs_c5, &sensel, &begin, ind_c5, val_c5, nullptr, nullptr));
            // constraint 3: I(v_j) + sum(z_avj) = y_vj
            ind_c3[_i.var_opt_conv[var_i]][nnz_c3[_i.var_opt_conv[var_i]]] = get_fa_idx(act_i, var_i);
            val_c3[_i.var_opt_conv[var_i]][nnz_c3[_i.var_opt_conv[var_i]]] = -1;
            nnz_c3[_i.var_opt_conv[var_i]]++;
        }
    }

    for (size_t var_i = 0; var_i < _i.n_opt; var_i++) assert(!CPXaddrows(env, lp, 0, 1, nnz_c3[var_i], &rhs_c3[var_i], &sensee, &begin, ind_c3[var_i], val_c3[var_i], nullptr, nullptr));

    for (size_t var_i = 0; var_i < _i.n_opt; var_i++) {
        delete[] ind_c3[var_i]; ind_c3[var_i] = nullptr;
        delete[] val_c3[var_i]; val_c3[var_i] = nullptr;
    }
    delete[] val_c1; val_c1 = nullptr;
    delete[] ind_c1; ind_c1 = nullptr;

    // assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUTPUT_DIR"/lp/"+_e.run_name+".lp").c_str(), "LP"));

}

void cpx_post_warmstart_imai(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {

    _ASSERT(_e.sol_s != solution_status::INFEAS && _e.sol_s != solution_status::NOTFOUND);

    const auto& rem_var = hplus::var_remaining(_i);
    binary_set state = binary_set(_i.n);

    std::vector<size_t> warm_start = _i.best_solution;
    
    size_t ncols = CPXgetnumcols(env, lp);
    int* cpx_sol_ind = new int[ncols];
    double* cpx_sol_val = new double[ncols];

    int izero = 0;
    int effortlevel = CPX_MIPSTART_REPAIR;
    size_t nnz = 0, timestamp = 0;

    for (auto act_i : warm_start) {
        cpx_sol_ind[nnz] = _i.act_opt_conv[act_i];
        cpx_sol_val[nnz++] = 1;
        cpx_sol_ind[nnz] = _i.m_opt + _i.act_opt_conv[act_i];
        cpx_sol_val[nnz++] = timestamp;
        timestamp++;
        for (auto var_i : _i.actions[act_i].eff_sparse) if (rem_var[var_i] && !state[var_i]) {
            cpx_sol_ind[nnz] = 2 * _i.m_opt + _i.var_opt_conv[var_i];
            cpx_sol_val[nnz++] = 1;
            cpx_sol_ind[nnz] = 2 * _i.m_opt + _i.n_opt + _i.var_opt_conv[var_i];
            cpx_sol_val[nnz++] = timestamp;
            cpx_sol_ind[nnz] = 2 * _i.m_opt + 2 * _i.n_opt + _i.fadd_checkpoint[_i.act_opt_conv[act_i]] + _i.var_opt_conv[var_i];
            cpx_sol_val[nnz++] = 1;
            state.add(var_i);
        }
    }

    assert(!CPXaddmipstarts(env, lp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));

    delete[] cpx_sol_ind; cpx_sol_ind = nullptr;
    delete[] cpx_sol_val; cpx_sol_val = nullptr;

}

void store_imai_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    
    // get cplex result (interested only in the sequence of actions [0/_i.m_opt-1] used and its ordering [_i.m_opt/2nact-1])
    double* plan = new double[2 * _i.m_opt];
    assert(!CPXgetx(env, lp, plan, 0, 2 * _i.m_opt - 1));

    // convert to std collections for easier parsing
    std::vector<std::pair<double, size_t>> cpx_result;
    for (size_t i = 0; i < _i.m_opt; i++) if (plan[i] > HPLUS_CPX_INT_ROUNDING) cpx_result.emplace_back(plan[_i.m_opt+i], i);
    delete[] plan; plan = nullptr;

    // sort cpx_result based on actions timestamps
    std::sort(cpx_result.begin(), cpx_result.end(),
        [](const std::pair<double, size_t> &x, const std::pair<double, size_t> &y) {
            return x.first < y.first;
        }
    );

    // get solution from sorted cpx_result
    std::vector<size_t> solution;
    std::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution),
        [_i](const std::pair<double, size_t> &p) {
            return _i.act_cpxtoidx[p.second];
        }
    );

    // store solution
    hplus::update_sol(_i, solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [_i](const size_t acc, const size_t index) {
                return acc + _i.actions[index].cost;
            }
        ), _l
    );

}

// ##################################################################### //
// ############################## RANKOOH ############################## //
// ##################################################################### //

void cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {

    const auto& rem_var_set = hplus::var_remaining(_i);
    const auto& rem_var = rem_var_set.sparse();
    const auto& rem_act = hplus::act_remaining(_i).sparse();

    // ====================================================== //
    // ================= VERTEX ELIMINATION ================= //
    // ====================================================== //

    typedef struct {
        size_t first, second, third;
    } triangle;

    std::vector<std::set<size_t>> graph(_i.n);
    std::vector<binary_set> cumulative_graph(_i.n, binary_set(_i.n));
    std::vector<triangle> triangles_list;

    // TODO: (maybe) Implement using pq.hxx
    struct node {
        size_t deg, id;
    };
    struct compare_node {
        bool operator()(const node& n1, const node& n2) { return n1.deg > n2.deg; }
    };
    std::priority_queue<node, std::vector<node>, compare_node> nodes_queue;
    std::vector<size_t> degree_counter(_i.n, 0);

    // G_0
    for (auto act_i : rem_act) {
        auto eff_sparse = (_i.actions[act_i].eff & rem_var_set).sparse();
        for (auto var_i : _i.actions[act_i].pre & rem_var_set) {
            for (auto var_j : eff_sparse) if (var_i != var_j) {
                size_t pre_size = graph[var_i].size();
                graph[var_i].insert(var_j);
                if (pre_size != graph[var_i].size()) {
                    degree_counter[var_i] += 1;
                    degree_counter[var_j] += 1;
                }
            }
            cumulative_graph[var_i] |= _i.actions[act_i].eff;
        }
    }

    for (size_t node_i = 0; node_i < _i.n; node_i++)
        if (degree_counter[node_i] > 0)
            nodes_queue.emplace(node_i, degree_counter[node_i]);

    // finding minimum degree node
    auto find_min = [&_i, &degree_counter](std::priority_queue<node, std::vector<node>, compare_node>& nodes_queue) {
        int idx = -1;
        while (!nodes_queue.empty() && idx < 0) {
            node tmp = nodes_queue.top();
            nodes_queue.pop();
            if (degree_counter[tmp.id] == tmp.deg) idx = tmp.id;
        }
        return idx;
    };

    // G_i (min degree heuristics)
    for (auto _ : rem_var) {
        int idx = find_min(nodes_queue);
        if (idx == -1) break;

        // graph structure:
        // | \       > |
        // p -> idx -> q
        // | /       > |

        std::set<size_t> new_nodes;

        for (auto p : rem_var) if (graph[p].find(idx) != graph[p].end()) {

            for (auto q : graph[idx]) if (p != q) {

                // add edge p - q
                size_t pre_size = graph[p].size();
                graph[p].insert(q);
                if (pre_size != graph[p].size()) {
                    degree_counter[p] += 1;
                    degree_counter[q] += 1;
                }

                // update the overall graph
                cumulative_graph[p].add(q);
            
            }
            
            // remove the edge p - idx
            graph[p].erase(idx);
            degree_counter[p] -= 1;
            new_nodes.insert(p);

            // update triangles list
            for (auto q : graph[idx]) if (p != q) triangles_list.emplace_back(p, idx, q);

        }

        // remove the edge idx - q
        for (auto q : graph[idx]) {
            degree_counter[q] -= 1;
            new_nodes.insert(q);
        }
        graph[idx].clear();
        degree_counter[idx] = 0;
        
        // Update the priority queue
        for (auto node : new_nodes) if (degree_counter[node] > 0) nodes_queue.emplace(node, degree_counter[node]);

        #if HPLUS_INTCHECK              // care: this takes HUGE amount of time
        for (size_t node_i = 0; node_i < _i.n; node_i++) {
            size_t i_cnt = 0;
            i_cnt += graph[node_i].size();
            for (size_t tmp_j = 0; tmp_j < _i.n; tmp_j++) {
                for (auto tmp_k : graph[tmp_j]) {
                    if (tmp_k == node_i) i_cnt += 1;
                }
            }
            assert(i_cnt == degree_counter[node_i]);
        }
        #endif

    }

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    size_t curr_col = 0;
    double* objs = new double[_i.m_opt];
    double* lbs = new double[_i.m_opt];
    double* ubs = new double[_i.m_opt];
    char* types = new char[_i.m_opt];

    auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {

        delete[] types; types = nullptr;
        delete[] ubs; ubs = nullptr;
        delete[] lbs; lbs = nullptr;
        delete[] objs; objs = nullptr;

        objs = new double[new_size];
        lbs = new double[new_size];
        ubs = new double[new_size];
        types = new char[new_size];

    };

    // -------- actions ------- //
    size_t act_start = curr_col;
    size_t count = 0;
    for (auto act_i : rem_act) {
        objs[count] = _i.actions[act_i].cost;
        lbs[count] = _i.act_f[act_i] ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }
    _ASSERT(count == _i.m_opt);

    curr_col += _i.m_opt;

    assert(!CPXnewcols(env, lp, _i.m_opt, objs, lbs, ubs, types, nullptr));

    resize_cpx_arrays(_i.n_opt);

    // --- first archievers --- //
    size_t fa_start = curr_col;
    std::vector<size_t> fa_individual_start(_i.m_opt);
    count = 0;
    for (auto act_i : rem_act) {
        fa_individual_start[count] = count * _i.n_opt;
        size_t count_var = 0;
        for (auto var_i : rem_var) {
            objs[count_var] = 0;
            lbs[count_var] = _i.fadd_f[act_i][var_i] ? 1 : 0;
            ubs[count_var] = (!_i.actions[act_i].eff[var_i] || _i.fadd_e[act_i][var_i]) ? 0 : 1;
            types[count_var++] = 'B';
        }
        _ASSERT(count_var == _i.n_opt);
        curr_col += _i.n_opt;
        assert(!CPXnewcols(env, lp, _i.n_opt, objs, lbs, ubs, types, nullptr));
        count++;
    }

    // ------- variables ------ //
    size_t var_start = curr_col;
    count = 0;
    for (auto var_i : rem_var) {
        objs[count] = 0;
        lbs[count] = (_i.var_f[var_i] || _i.goal[var_i]) ? 1 : 0;
        ubs[count] = 1;
        types[count++] = 'B';
    }

    _ASSERT(count == _i.n_opt);
    curr_col += _i.n_opt;

    assert(!CPXnewcols(env, lp, _i.n_opt, objs, lbs, ubs, types, nullptr));

    // vertex elimination graph edges
    size_t veg_edges_start = curr_col;
    for (auto var_i : rem_var) {
        count = 0;
        for (auto var_j : rem_var) {
            objs[count] = 0;
            lbs[count] = 0;
            ubs[count] = cumulative_graph[var_i][var_j] ? 1 : 0;
            types[count++] = 'B';
        }
        _ASSERT(count == _i.n_opt);
        assert(!CPXnewcols(env, lp, _i.n_opt, objs, lbs, ubs, types, nullptr));
    }
    curr_col += _i.n_opt * _i.n_opt;

    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    auto get_act_idx = [&_i, &act_start](size_t idx) { return act_start + _i.act_opt_conv[idx]; };
    auto get_var_idx = [&_i, &var_start](size_t idx) { return var_start + _i.var_opt_conv[idx]; };
    auto get_fa_idx = [&_i, &fa_start](size_t act_idx, size_t var_idx) { return fa_start + _i.fadd_checkpoint[_i.act_opt_conv[act_idx]] + _i.var_opt_conv[var_idx]; };
    auto get_veg_idx = [&_i, &veg_edges_start](size_t idx_i, size_t idx_j) { return veg_edges_start + _i.var_opt_conv[idx_i] * _i.n_opt + _i.var_opt_conv[idx_j]; };

    int* ind = new int[_i.m_opt + 1];
    double* val = new double[_i.m_opt + 1];
    int nnz = 0;
    const char sense_e = 'E', sense_l = 'L';
    const double rhs_0 = 0, rhs_1 = 1;
    const int begin = 0;

    for (auto var_i : rem_var) {

        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        for (auto act_i : _i.act_with_eff[var_i]) {
            ind[nnz] = get_fa_idx(act_i, var_i);
            val[nnz++] = -1;
        }

        assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr));

    }
    
    for (auto var_i : rem_var) {
        for (auto var_j : rem_var) {
            nnz = 0;
            ind[nnz] = get_var_idx(var_j);
            val[nnz++] = -1;
            for (auto act_i : _i.act_with_eff[var_i]) {
                if (_i.actions[act_i].pre[var_j]) {
                    ind[nnz] = get_fa_idx(act_i, var_i);
                    val[nnz++] = 1;
                }
            }
            assert(!CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr));
        }
    }

    delete[] val; val = nullptr;
    delete[] ind; ind = nullptr;

    int ind_c5_c6_c7[2], ind_c8[3];
    double val_c5_c6_c7[2], val_c8[3];
    
    for (auto act_i : rem_act) {
        for (auto var_i : _i.actions[act_i].eff) if (rem_var_set[var_i]) {
            ind_c5_c6_c7[0] = get_act_idx(act_i);
            val_c5_c6_c7[0] = -1;
            ind_c5_c6_c7[1] = get_fa_idx(act_i, var_i);
            val_c5_c6_c7[1] = 1;
            assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
        }
    }
    
    for (auto act_i : rem_act) {
        const auto& pre = _i.actions[act_i].pre_sparse;
        const auto& eff = _i.actions[act_i].eff_sparse;
        for (auto var_i : pre) if (rem_var_set[var_i]) {
            for (auto var_j : eff) if (rem_var_set[var_j]) {
                ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
                val_c5_c6_c7[0] = -1;
                ind_c5_c6_c7[1] = get_fa_idx(act_i, var_j);
                val_c5_c6_c7[1] = 1;
                assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
            }
        }
    }
    
    for (auto var_i : rem_var) for (auto var_j : cumulative_graph[var_i]) {
        ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
        val_c5_c6_c7[0] = 1;
        ind_c5_c6_c7[1] = get_veg_idx(var_j, var_i);
        val_c5_c6_c7[1] = 1;
        assert(!CPXaddrows(env, lp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
    }

    for (size_t h = 0; h < triangles_list.size(); h++) {
        const size_t i = triangles_list[h].first, j = triangles_list[h].second, k = triangles_list[h].third;
        ind_c8[0] = get_veg_idx(i, j);
        val_c8[0] = 1;
        ind_c8[1] = get_veg_idx(j, k);
        val_c8[1] = 1;
        ind_c8[2] = get_veg_idx(i, k);
        val_c8[2] = -1;
        assert(!CPXaddrows(env, lp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr));
    }

    // assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUTPUT_DIR"/lp/"+_e.run_name+".lp").c_str(), "LP"));

}

void cpx_post_warmstart_rankooh(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    
    _ASSERT(_e.sol_s != solution_status::INFEAS && _e.sol_s != solution_status::NOTFOUND);

    const auto& remaining_variables = hplus::var_remaining(_i);
    binary_set state(_i.n);

    std::vector<size_t> warm_start = _i.best_solution;

    size_t ncols = CPXgetnumcols(env, lp);
    int* cpx_sol_ind = new int[ncols];
    double* cpx_sol_val = new double[ncols];

    int izero = 0;
    int effortlevel = CPX_MIPSTART_REPAIR;
    size_t nnz = 0;

    for (auto act_i : warm_start) {
        cpx_sol_ind[nnz] = _i.act_opt_conv[act_i];
        cpx_sol_val[nnz++] = 1;
        for (auto var_i : _i.actions[act_i].eff_sparse) if (remaining_variables[var_i] && !state[var_i]) {
            cpx_sol_ind[nnz] = _i.m_opt + _i.m_opt * _i.n_opt + _i.var_opt_conv[var_i];
            cpx_sol_val[nnz++] = 1;
            cpx_sol_ind[nnz] = _i.m_opt + _i.fadd_checkpoint[_i.act_opt_conv[act_i]] + _i.var_opt_conv[var_i];
            cpx_sol_val[nnz++] = 1;
            state.add(var_i);
        }
    }

    assert(!CPXaddmipstarts(env, lp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));
    delete[] cpx_sol_ind; cpx_sol_ind = nullptr;
    delete[] cpx_sol_val; cpx_sol_val = nullptr;

}

void store_rankooh_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    
    double* plan = new double[_i.m_opt + _i.m_opt * _i.n_opt];
    assert(!CPXgetx(env, lp, plan, 0, _i.m_opt + _i.m_opt * _i.n_opt - 1));
    
    // fixing the solution to read the plan (some actions are set to 1 even if they are not a first archiever of anything)
    for (size_t act_i_cpx = 0, fadd_i = _i.m_opt; act_i_cpx < _i.m_opt; act_i_cpx++, fadd_i += _i.n_opt) {
        bool set_zero = true;
        for (size_t var_i_cpx = 0; var_i_cpx < _i.n_opt; var_i_cpx++) {
            if (plan[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
                _ASSERT(plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
                set_zero = false;
                break;
            }
        }
        if (set_zero) plan[act_i_cpx] = 0;
    }

    // convert to std collections for easier parsing
    std::vector<size_t> cpx_result;
    for (size_t i = 0; i < _i.m_opt; i++) if (plan[i] > .5) cpx_result.push_back(_i.act_cpxtoidx[i]);
    delete[] plan; plan = nullptr;

    std::vector<size_t> solution;
    binary_set sorted(cpx_result.size());
    binary_set current_state(_i.n);
    
    while (sorted != binary_set(cpx_result.size(), true)) {
        #if HPLUS_INTCHECK
        bool intcheck = false;
        #endif
        for (auto i : !sorted) {
            if (current_state.contains(_i.actions[cpx_result[i]].pre)) {
                sorted.add(i);
                current_state |= _i.actions[cpx_result[i]].eff;
                solution.push_back(cpx_result[i]);
                #if HPLUS_INTCHECK
                intcheck = true;
                #endif
                break;
            }
        }
        _ASSERT(intcheck);      // FIXME: FAILED ON "woodworking-opt08-strips-p01.sas --no-op"
    }

    // store solution
    hplus::update_sol(_i, solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&_i](const unsigned int acc, const size_t index) {
                return acc + _i.actions[index].cost;
            }
        ), _l
    );
    
}

// ##################################################################### //
// ########################### DYNAMIC SMALL ########################### //
// ##################################################################### //

void cpx_build_dynamic_small(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    // TODO: Building the small dynamic model
    todo(_l, "Building the small dynamic model");
}

void cpx_post_warmstart_dynamic_small(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    // TODO: Posting warmstart to the small dynamic model
    todo(_l, "Posting warmstart to the small dynamic model");
}

void store_dynamic_small_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    // TODO: Storing solution of the small dynamic model
    todo(_l, "Storing solution of the small dynamic model");
}

// ##################################################################### //
// ########################### DYNAMIC LARGE ########################### //
// ##################################################################### //

void cpx_build_dynamic_large(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    // TODO: Building the large dynamic model
    todo(_l, "Building the large dynamic model");
}

void cpx_post_warmstart_dynamic_large(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    // TODO: Posting warmstart to the large dynamic model
    todo(_l, "Posting warmstart to the large dynamic model");
}

void store_dynamic_large_sol(CPXENVptr& env, CPXLPptr& lp, hplus::instance& _i, const hplus::environment& _e, const logger& _l) {
    // TODO: Storing solution of the large dynamic model
    todo(_l, "Storing solution of the large dynamic model");
}