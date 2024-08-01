#include "../include/algorithms.hpp"

#include <functional>
#include <algorithm>
#include <csignal>
#include <unistd.h>
#include <numeric>
#include <cplex.h>

// ##################################################################### //
// ############################### CPLEX ############################### //
// ##################################################################### //

/**
 * Create the cplex env and lp
 *
 * @param env: Pointer to the cplex environment
 * @param lp: Pointer to the cplex linear problem
*/
void HPLUS_cpx_init(CPXENVptr* env, CPXLPptr* lp) {

    int cpxerror;
    *env = CPXopenCPLEX(&cpxerror); MYASSERT(!cpxerror);
    *lp = CPXcreateprob(*env, &cpxerror, "HPLUS"); MYASSERT(!cpxerror);

    // log file
    MYASSERT(!CPXsetintparam(*env, CPXPARAM_ScreenOutput, CPX_OFF));
    MYASSERT(!CPXsetlogfilename(*env, (HPLUS_CPLEX_OUT_DIR"/log/"+HPLUS_env.run_name+".log").c_str(), "w"));
    MYASSERT(!CPXsetintparam(*env, CPX_PARAM_CLONELOG, -1));

    // tolerance
    MYASSERT(!CPXsetdblparam(*env, CPXPARAM_MIP_Tolerances_MIPGap, 0));

    // time limit
    MYASSERT(!CPXsetdblparam(*env, CPXPARAM_TimeLimit, (double)HPLUS_env.time_limit));

    // terminate condition
    MYASSERT(!CPXsetterminate(*env, &HPLUS_env.cpx_terminate));

}

/**
 * Delete the cplex env and lp
 *
 * @param env: Pointer to the cplex environment
 * @param lp: Pointer to the cplex linear problem
*/
void HPLUS_cpx_close(CPXENVptr* env, CPXLPptr* lp) {

    CPXfreeprob(*env, lp);
    CPXcloseCPLEX(env);

}

/**
 * Parse cplex status and stores it into the HPLUS_env global variable
 *
 * @param env: Pointer to the cplex environment
 * @param lp: Pointer to the cplex linear problem
*/
void HPLUS_parse_cplex_status(const CPXENVptr* env, const CPXLPptr* lp) {

    switch ( int cpxstatus = CPXgetstat(*env, *lp) ) {
        case CPXMIP_TIME_LIM_FEAS:      // exceeded time limit, found intermediate solution
            HPLUS_env.status = my::status::TIMEL_FEAS;
            break;
        case CPXMIP_TIME_LIM_INFEAS:    // exceeded time limit, no intermediate solution found
            HPLUS_env.status = my::status::TIMEL_NF;
            break;
        case CPXMIP_INFEASIBLE:         // proven to be unfeasible
            HPLUS_env.status = my::status::INFEAS;
            break;
        case CPXMIP_ABORT_FEAS:         // terminated by user, found solution
            HPLUS_env.status = my::status::USR_STOP_FEAS;
            break;
        case CPXMIP_ABORT_INFEAS:       // terminated by user, not found solution
            HPLUS_env.status = my::status::USR_STOP_NF;
            break;
        case CPXMIP_OPTIMAL_TOL:        // found optimal within the tollerance
            HPLUS_env.logger.print_warn("Found optimal within the tolerance.");
            HPLUS_env.status = my::status::OPT;
            break;
        case CPXMIP_OPTIMAL:            // found optimal
            HPLUS_env.status = my::status::OPT;
            break;
        default:                        // unhandled status
            HPLUS_env.logger.raise_error("Error in tsp_cplex: unhandled cplex status: %d.", cpxstatus);
            break;
    }

}

// ##################################################################### //
// ################################ IMAI ############################### //
// ##################################################################### //

/**
 * Build the basic imai model
 *
 * @param env: Pointer to the cplex environment
 * @param lp: Pointer to the cplex linear problem
 * @param inst: Instance to represent through the model
*/
void HPLUS_cpx_build_imai(const CPXENVptr* env, const CPXLPptr* lp, const HPLUS_instance* inst) {

    const unsigned int nvar = inst -> get_nvar();
    const unsigned int nact = inst -> get_nact();
    const unsigned int bfsize = inst -> get_bfsize();
    const HPLUS_action** actions = inst -> get_actions();
    const HPLUS_variable** variables = inst -> get_variables();
    const my::BitField* istate = inst -> get_istate();
    const my::BitField* gstate = inst -> get_gstate();

    // ~~~~~~~~ Adding CPLEX variables ~~~~~~~ //

    unsigned int maxcpxcols = 2 * nact + 2 * bfsize + nact * bfsize;
    double* objs = new double[maxcpxcols];
    double* lbs = new double[maxcpxcols];
    double* ubs = new double[maxcpxcols];
    char* types = new char[maxcpxcols];
    char** names = new char*[maxcpxcols];
    for (unsigned int i = 0; i < maxcpxcols; i++) names[i] = new char[20];

    unsigned int curr_col = 0;

    // actions
    unsigned int act_start = curr_col;
    for (unsigned int i = 0; i < nact; i++) {
        objs[curr_col + i] = actions[i] -> get_cost();
        lbs[curr_col + i] = 0.0;
        ubs[curr_col + i] = 1.0;
        types[curr_col + i] = 'B';
        snprintf(names[curr_col + i], 20, "act(%d)", i);
    }
    curr_col += nact;

    // action timestamps
    unsigned int tact_start = curr_col;
    for (unsigned int i = 0; i < nact; i++) {
        objs[curr_col + i] = 0;
        lbs[curr_col + i] = 0.0;
        ubs[curr_col + i] = nact-1;
        types[curr_col + i] = 'I';
        snprintf(names[curr_col + i], 20, "tact(%d)", i);
    }
    curr_col += nact;

    // variables
    unsigned int var_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i] -> get_range(); j++, count++) {
        objs[curr_col + count] = 0;
        lbs[curr_col + count] = (istate -> operator[](count) || gstate -> operator[](count)) ? 1.0 : 0.0;  // fix variables of initial and goal state to 1
        ubs[curr_col + count] = 1.0;
        types[curr_col + count] = 'B';
        snprintf(names[curr_col + count], 20, "var(%d_%d)", i, j);
    }
    curr_col += bfsize;

    // variable timestamps
    unsigned int tvar_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i] -> get_range(); j++, count++) {
        objs[curr_col + count] = 0;
        lbs[curr_col + count] = 1;
        ubs[curr_col + count] = nact;
        types[curr_col + count] = 'I';
        snprintf(names[curr_col + count], 20, "tvar(%d_%d)", i, j);
    }
    curr_col += bfsize;

    // first archievers
    unsigned int nfa = 0;
    unsigned int fa_start = curr_col;
    for (unsigned int c = 0; c < nact; c++) {
        const my::BitField* eff = actions[c] -> get_eff();
        for (unsigned int i = 0, k = 0; i < nvar; k += variables[i] -> get_range(), i++) for (unsigned int j = 0; j < variables[i] -> get_range(); j++) {
            if (!eff -> operator[](k + j)) continue;    // create a variable only for the effects of the action
            objs[curr_col + nfa] = 0;
            lbs[curr_col + nfa] = 0.0;
            ubs[curr_col + nfa] = 1.0;
            types[curr_col + nfa] = 'B';
            snprintf(names[curr_col + nfa], 20, "fa(%d_%d_%d)", c, i, j);
            nfa++;
        }
    }
    curr_col += nfa;

    MYASSERT(curr_col <= maxcpxcols);

    MYASSERT(!CPXnewcols(*env, *lp, curr_col, objs, lbs, ubs, types, names));

    for (unsigned int i = 0; i < maxcpxcols; i++) delete[] names[i];
    MYDELL(names);
    MYDELL(types);
    MYDELL(ubs);
    MYDELL(lbs);
    MYDELL(objs);

    // ~~~~~~~ Adding CPLEX constraints ~~~~~~ //

    int ind1[2], ind2[3];
    double val1[2], val2[3];
    int nnz1 = 2, nnz2 = 3;
    char sensel = 'L', sensee = 'E';
    double rhs1 = 0, rhs2 = nact;
    int begin = 0;

    std::vector<int*> ind(bfsize);
    std::vector<double*> val(bfsize);
    std::vector<int> nnz(bfsize);
    std::vector<double> rhs(bfsize);

    for (unsigned int i = 0; i < bfsize; i++) {
        ind[i] = new int[nact + 1];
        val[i] = new double[nact + 1];
        nnz[i] = 0;
        rhs[i] = istate -> operator[](i);
        ind[i][nnz[i]] = var_start + i;
        val[i][nnz[i]] = 1;
        nnz[i]++;
    }

    for (unsigned int i = 0, count_fa = 0; i < nact; i++) {
        const my::BitField* pre = actions[i] -> get_pre();
        const my::BitField* eff = actions[i] -> get_eff();
        for (unsigned int j = 0; j < bfsize; j++) {
            if (pre -> operator[](j)) {
                // constraint 1: x_a <= y_vj, vj in pre(a)
                ind1[0] = act_start + i;
                val1[0] = 1;
                ind1[1] = var_start + j;
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr));
                // constraint 4: t_vj <= t_a, vj in pre(a)
                ind1[0] = tvar_start + j;
                val1[0] = 1;
                ind1[1] = tact_start + i;
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr));
            }
            if (eff -> operator[](j)) {
                // constraint 2: z_avj <= x_a, vj in eff(a)
                ind1[0] = fa_start + count_fa;
                val1[0] = 1;
                ind1[1] = act_start + i;
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr));
                // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
                ind2[0] = tact_start + i;
                val2[0] = 1;
                ind2[1] = tvar_start + j;
                val2[1] = -1;
                ind2[2] = fa_start + count_fa;
                val2[2] = nact + 1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz2, &rhs2, &sensel, &begin, ind2, val2, nullptr, nullptr));
                // constraint 3: I(v_j) + sum(z_avj) = y_vj
                ind[j][nnz[j]] = fa_start + count_fa++;
                val[j][nnz[j]] = -1;
                nnz[j]++;
                MYASSERT(nnz[j] <= nact + 1);
            }
        }
    }

    for (unsigned int i = 0; i < bfsize; i++) MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz[i], &rhs[i], &sensee, &begin, ind[i], val[i], nullptr, nullptr));

    for (unsigned int i = 0; i < bfsize; i++) {
        MYDELL(ind[i]);
        MYDELL(val[i]);
    }

    MYASSERT(!CPXwriteprob(*env, *lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"));

    HPLUS_env.logger.print_info("Created CPLEX lp for imai.");

}

/**
 * Check if this solution is the best one and if so, stores it
 *
 * @param env: Pointer to the cplex environment
 * @param lp: Pointer to the cplex linear problem
 * @param inst: Pointer to the current instance
*/
void HPLUS_store_imai_sol(const CPXENVptr* env, const CPXLPptr* lp, HPLUS_instance* inst) {

    // get cplex result (interested only in the sequence of actions used and its ordering)
    unsigned int nact = inst -> get_nact();
    auto* xstar = new double[2 * nact];
    MYASSERT(!CPXgetx(*env, *lp, xstar, 0, 2 * nact - 1));

    // convert to std collections for easier parsing
    std::vector<std::pair<double, unsigned int>> cpx_result;
    for (unsigned int i = 0; i < nact; i++) if (xstar[i] > .5) cpx_result.emplace_back(xstar[nact+i], i);
    MYDELL(xstar);

    // sort cpx_result based on actions timestamps
    std::ranges::sort(cpx_result.begin(), cpx_result.end(),
        [](const std::pair<double, unsigned int> &x, const std::pair<double, unsigned int> &y) {
            return x.first < y.first;
        }
    );

    // get solution from sorted cpx_result
    std::vector<unsigned int> solution;
    std::ranges::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution),
        [](const std::pair<double, unsigned int> &p) {
            return p.second;
        }
    );

    // store solution
    const HPLUS_action** actions = inst -> get_actions();
    inst -> update_best_solution(solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&actions](const unsigned int acc, const unsigned int index) {
                return acc + actions[index] -> get_cost();
            }
        )
    );

}

void HPLUS_run_imai(HPLUS_instance* inst) {

    HPLUS_env.logger.print_info("Running imai algorithm.");

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    HPLUS_cpx_init(&env, &lp);
    HPLUS_cpx_build_imai(&env, &lp, inst);

    MYASSERT(!CPXmipopt(env, lp));

    HPLUS_parse_cplex_status(&env, &lp);

    if (HPLUS_env.found()) HPLUS_store_imai_sol(&env, &lp, inst);

    HPLUS_cpx_close(&env, &lp);

}