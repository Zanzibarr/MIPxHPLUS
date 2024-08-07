#include "../include/algorithms.hpp"

#include <functional>
#include <algorithm>
#include <csignal>
#include <unistd.h>
#include <numeric>
#include <queue>
#include <cplex.h>

// ##################################################################### //
// ############################### CPLEX ############################### //
// ##################################################################### //

/**
 * Create the cplex env and lp
*/
void HPLUS_cpx_init(CPXENVptr& env, CPXLPptr& lp) {

    int cpxerror;
    env = CPXopenCPLEX(&cpxerror); my::assert(!cpxerror, "CPXopenCPLEX failed.");
    lp = CPXcreateprob(env, &cpxerror, "HPLUS"); my::assert(!cpxerror, "CPXcreateprob failed.");

    // log file
    my::assert(!CPXsetintparam(env, CPXPARAM_ScreenOutput, CPX_OFF), "CPXsetintparam failed (CPXPARAM_ScreenOutput).");
    my::assert(!CPXsetlogfilename(env, (HPLUS_CPLEX_OUT_DIR"/log/"+HPLUS_env.run_name+".log").c_str(), "w"), "CPXsetlogfilename failed.");
    my::assert(!CPXsetintparam(env, CPX_PARAM_CLONELOG, -1), "CPXsetintparam (CPX_PARAM_CLONELOG) failed.");

    // tolerance
    my::assert(!CPXsetdblparam(env, CPXPARAM_MIP_Tolerances_MIPGap, 0), "CPXsetdblparam (CPXPARAM_MIP_Tolerances_MIPGap) failed.");

    // time limit
    my::assert(!CPXsetdblparam(env, CPXPARAM_TimeLimit, (double)HPLUS_env.time_limit), "CPXsetdblparam (CPXPARAM_TimeLimit) failed.");

    // terminate condition
    my::assert(!CPXsetterminate(env, &HPLUS_env.cpx_terminate), "CPXsetterminate failed.");

}

/**
 * Delete the cplex env and lp
*/
void HPLUS_cpx_close(CPXENVptr& env, CPXLPptr& lp) {

    CPXfreeprob(env, &lp);
    CPXcloseCPLEX(&env);

}

/**
 * Parse cplex status and stores it into the HPLUS_env global variable
*/
void HPLUS_parse_cplex_status(const CPXENVptr& env, const CPXLPptr& lp) {

    switch ( int cpxstatus = CPXgetstat(env, lp) ) {
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
        case 0:
            HPLUS_env.status = my::status::NOTFOUND;
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
 * Build the basic Imai model
*/
void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst) {

    const unsigned int nvar = inst.get_nvar();
    const unsigned int nact = inst.get_nact();
    const unsigned int bfsize = inst.get_bfsize();
    const std::vector<HPLUS_action>& actions = inst.get_actions();
    const std::vector<HPLUS_variable>& variables = inst.get_variables();
    const my::BitField& istate = inst.get_istate();
    const my::BitField& gstate = inst.get_gstate();

    // ~~~~~~~~ Adding CPLEX variables ~~~~~~~ //

    unsigned int curr_col = 0;

    auto* objs = new double[nact];
    auto* lbs = new double[nact];
    auto* ubs = new double[nact];
    auto* types = new char[nact];
    auto** names = new char*[nact];
    for (unsigned int i = 0; i < nact; i++) names[i] = new char[20];

    // actions
    unsigned int act_start = curr_col;
    for (unsigned int i = 0; i < nact; i++) {
        objs[i] = actions[i].get_cost();
        lbs[i] = 0.0;
        ubs[i] = 1.0;
        types[i] = 'B';
        snprintf(names[i], 20, "act(%d)", i);
    }
    curr_col += nact;

    my::assert(!CPXnewcols(env, lp, nact, objs, lbs, ubs, types, names), "CPXnewcols (actions) failed.");

    // action timestamps
    unsigned int tact_start = curr_col;
    for (unsigned int i = 0; i < nact; i++) {
        objs[i] = 0;
        lbs[i] = 0;
        ubs[i] = nact-1;
        types[i] = 'I';
        snprintf(names[i], 20, "tact(%d)", i);
    }
    curr_col += nact;

    my::assert(!CPXnewcols(env, lp, nact, objs, lbs, ubs, types, names), "CPXnewcols (action timestamps) failed.");

    for (unsigned int i = 0; i < nact; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    objs = new double[bfsize];
    lbs = new double[bfsize];
    ubs = new double[bfsize];
    types = new char[bfsize];
    names = new char*[bfsize];
    for (unsigned int i = 0; i < bfsize; i++) names[i] = new char[20];

    // variables
    unsigned int var_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i].get_range(); j++, count++) {
        objs[count] = 0;
        lbs[count] = (istate[count] || gstate[count]) ? 1.0 : 0.0;  // fix variables of initial and goal state to 1
        ubs[count] = 1;
        types[count] = 'B';
        snprintf(names[count], 20, "var(%d_%d)", i, j);
    }
    curr_col += bfsize;

    my::assert(!CPXnewcols(env, lp, bfsize, objs, lbs, ubs, types, names), "CPXnewcols (variables) failed.");

    // variable timestamps
    unsigned int tvar_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i].get_range(); j++, count++) {
        objs[count] = 0;
        lbs[count] = 1;
        ubs[count] = nact;
        types[count] = 'I';
        snprintf(names[count], 20, "tvar(%d_%d)", i, j);
    }
    curr_col += bfsize;

    my::assert(!CPXnewcols(env, lp, bfsize, objs, lbs, ubs, types, names), "CPXnewcols (variable timestamps) failed.");

    for (unsigned int i = 0; i < bfsize; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    objs = new double[nact * bfsize];
    lbs = new double[nact * bfsize];
    ubs = new double[nact * bfsize];
    types = new char[nact * bfsize];
    names = new char*[nact * bfsize];
    for (unsigned int i = 0; i < nact * bfsize; i++) names[i] = new char[20];

    // first archievers
    unsigned int nfa = 0;
    unsigned int fa_start = curr_col;
    for (unsigned int c = 0; c < nact; c++) {
        const my::BitField& eff = actions[c].get_eff();
        for (unsigned int i = 0, k = 0; i < nvar; k += variables[i].get_range(), i++) for (unsigned int j = 0; j < variables[i].get_range(); j++) {
            if (!eff[k + j]) continue;    // create a variable only for the effects of the action
            objs[nfa] = 0;
            lbs[nfa] = 0;
            ubs[nfa] = 1;
            types[nfa] = 'B';
            snprintf(names[nfa], 20, "fa(%d_%d_%d)", c, i, j);
            nfa++;
        }
    }
    curr_col += nfa;

    my::assert(!CPXnewcols(env, lp, nfa, objs, lbs, ubs, types, names), "CPXnewcols (first archievers) failed.");

    for (unsigned int i = 0; i < nact * bfsize; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

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
        rhs[i] = istate[i];
        ind[i][nnz[i]] = var_start + i;
        val[i][nnz[i]] = 1;
        nnz[i]++;
    }

    for (unsigned int i = 0, count_fa = 0; i < nact; i++) {
        const my::BitField& pre = actions[i].get_pre();
        const my::BitField& eff = actions[i].get_eff();
        for (unsigned int j = 0; j < bfsize; j++) {
            if (pre[j]) {
                // constraint 1: x_a <= y_vj, vj in pre(a)
                ind1[0] = act_start + i;
                val1[0] = 1;
                ind1[1] = var_start + j;
                val1[1] = -1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr), "CPXaddrows (c1) failed.");
                // constraint 4: t_vj <= t_a, vj in pre(a)
                ind1[0] = tvar_start + j;
                val1[0] = 1;
                ind1[1] = tact_start + i;
                val1[1] = -1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr), "CPXaddrows (c4) failed.");
            }
            if (eff[j]) {
                // constraint 2: z_avj <= x_a, vj in eff(a)
                ind1[0] = fa_start + count_fa;
                val1[0] = 1;
                ind1[1] = act_start + i;
                val1[1] = -1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr), "CPXaddrows (c2) failed.");
                // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
                ind2[0] = tact_start + i;
                val2[0] = 1;
                ind2[1] = tvar_start + j;
                val2[1] = -1;
                ind2[2] = fa_start + count_fa;
                val2[2] = nact + 1;
                my::assert(!CPXaddrows(env, lp, 0, 1, nnz2, &rhs2, &sensel, &begin, ind2, val2, nullptr, nullptr), "CPXaddrows (c5) failed.");
                // constraint 3: I(v_j) + sum(z_avj) = y_vj
                ind[j][nnz[j]] = fa_start + count_fa++;
                val[j][nnz[j]] = -1;
                nnz[j]++;
            }
        }
    }

    for (unsigned int i = 0; i < bfsize; i++) my::assert(!CPXaddrows(env, lp, 0, 1, nnz[i], &rhs[i], &sensee, &begin, ind[i], val[i], nullptr, nullptr), "CPXaddrows (c3) failed.");

    for (unsigned int i = 0; i < bfsize; i++) {
        delete[] ind[i]; ind[i] = nullptr;
        delete[] val[i]; val[i] = nullptr;
    }

    my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    HPLUS_env.logger.print_info("Created CPLEX lp for imai.");

}

/**
 * Fact landmarks extracting method from Imai's paper
 */
void HPLUS_imai_extract_fact_landmarks(const HPLUS_instance& inst, std::vector<my::BitField>& fact_landmarks) {

    const unsigned int bfsize = inst.get_bfsize();

    fact_landmarks = std::vector<my::BitField>(bfsize);
    const my::BitField& istate = inst.get_istate();
    for (unsigned int p = 0; p < bfsize; p++) {
        // using the bit at position bfsize as a flag for "full set of variables"
        fact_landmarks[p] = my::BitField(bfsize + 1);
        if (istate[p]) fact_landmarks[p].set(p);                                                                                                    //  L[p] <- {p} for each p in I
        else fact_landmarks[p].set(bfsize);                                                                                                         //  L[p] <- P for each p not in I
    }

    const std::vector<HPLUS_action>& actions = inst.get_actions();
    my::BitField s(inst.get_istate());                                                                                                              //  S <- I

    std::deque<int> actions_queue;
    for (unsigned int i = 0; i < inst.get_nact(); i++)                                                                                                         //  for a in A do
        if(s.contains(actions[i].get_pre())) actions_queue.push_back(i);                                                                            //      insert a into a FIFO queue if pre(a) is in S

    while(!actions_queue.empty()) {                                                                                                                 //  while Q is not empty do

        const HPLUS_action a = actions[actions_queue.front()];                                                                                      //      retrieve an action a from Q
        actions_queue.pop_front();

        const my::BitField& pre_a = a.get_pre();
        const my::BitField& add_a = a.get_eff();

        for (auto p : add_a) {                                                                                                                      //      for p in add(a) do

            s.set(p);                                                                                                                               //          S <- S union {p}

            my::BitField x = my::BitField(bfsize + 1);
            for (auto p : add_a) x.set(p);                                                                                                          //          X <- add(a)

            for (auto pp : pre_a) {                                                                                                                 //          for p' in pre(a)
                // if variable p has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set the flag
                if (fact_landmarks[pp][bfsize]) {
                    x.set(bfsize);
                    // if x is now full we can exit, since all further unions wont change x
                    break;
                } else x.unificate(fact_landmarks[pp]);                                                                                             //              X <- X union L[p']
            }

            // we then check if L[p] != X, and if they are the same we skip,
            // if X = P, then (X intersection L[P]) = L[P], hence we can already skip
            if (!x[bfsize]) {

                // if the set for variable p is the full set of variables,
                // the intersection generates back x -> we can skip the intersection
                if (!fact_landmarks[p][bfsize]) x.intersect(fact_landmarks[p]);                                                                     //          X <- X intersection L[p]

                // we already know that x is not the full set now, so if
                // the set for variable p is the full set, we know that x is not
                // equal to the set for variable p -> we can skip the check
                if (fact_landmarks[p][bfsize] || !x.equals(fact_landmarks[p])) {                                                                    //          if L[p] != X then

                    fact_landmarks[p] = my::BitField(x);                                                                                            //              L[p] <- X
                    for (unsigned int i = 0; i < inst.get_nact(); i++) if (actions[i].get_pre()[p]) {                                               //              for a' in A : p in pre(a)
                        if (s.contains(actions[i].get_pre()) && std::find(actions_queue.begin(), actions_queue.end(), i) == actions_queue.end())    //                  if pre(a') is in S and a' is not in Q
                            actions_queue.push_back(i);                                                                                             //                      insert a' into Q
                    }

                }
            }

        }

    }                                                                                                                                               //  at this point, L[p] contains sets of fact landmarks for p in P

}

/**
 * Fix landmarks as explained in Imai's paper
 */
void HPLUS_imai_fix_landmarks(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst, const std::vector<my::BitField>& fact_landmarks) {

    const unsigned int nact = inst.get_nact();
    const unsigned int bfsize = inst.get_bfsize();
    const my::BitField& istate = inst.get_istate();
    const my::BitField& gstate = inst.get_gstate();

    int nnz_var = 0;
    int* ind_var = new int[bfsize];
    char* lu_var = new char[bfsize];
    double* bd_var = new double[bfsize];

    int nnz_act = 0;
    int* ind_act = new int[nact];
    char* lu_act = new char[nact];
    double* bd_act = new double[nact];

    my::BitField dbvar_checker(bfsize);
    my::BitField dbact_checker(nact);

    for (auto i : gstate) {
        for (unsigned int j = 0; j < bfsize; j++) if (!istate[j]) {                                             // -->  if a variable is in the initial state it's already been fixed when building the
            if(!dbvar_checker[j] && (fact_landmarks[i][bfsize] || fact_landmarks[i][j])) {                      //      model; also fixing actions that have as an effect variables in the initial
                                                                                                                //      state means to fix actions that might not have been used in the optimal solution,
                // fixing fact landmarks                                                                        //      since those variables don't need actions to be used to be archieved.
                dbvar_checker.set(j);
                ind_var[nnz_var] = 2 * nact + j;
                lu_var[nnz_var] = 'L';
                bd_var[nnz_var++] = 1.0;

                // fixing act landmarks
                unsigned int act_count = 0;
                for (unsigned int k = 0; k < nact && act_count < 2; k++) if (inst.get_actions()[k].get_eff()[j]) {
                    act_count++;
                    ind_act[nnz_act] = k;
                    lu_act[nnz_act] = 'L';
                    bd_act[nnz_act] = 1.0;
                }
                if (act_count == 1 && !dbact_checker[ind_act[nnz_act]]) dbact_checker.set(ind_act[nnz_act++]);

            }
        }
    }

    my::assert(!CPXchgbds(env, lp, nnz_var, ind_var, lu_var, bd_var), "CPXchgbds (variables) failed.");
    my::assert(!CPXchgbds(env, lp, nnz_act, ind_act, lu_act, bd_act), "CPXchgbds (actions) failed.");

    delete[] bd_act;
    delete[] lu_act;
    delete[] ind_act;
    delete[] bd_var;
    delete[] lu_var;
    delete[] ind_var;

}

/**
 * Variable elimination method from Imai's paper
 */
void HPLUS_imai_variable_elimination(CPXENVptr& env, CPXLPptr& lp, HPLUS_instance& inst) {

    std::vector<my::BitField> fact_landmarks;

    HPLUS_imai_extract_fact_landmarks(inst, fact_landmarks);
    HPLUS_imai_fix_landmarks(env, lp, inst, fact_landmarks);

}

/**
 * Check if this solution is the best one and if so, stores it
*/
void HPLUS_store_imai_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    // get cplex result (interested only in the sequence of actions used and its ordering)
    unsigned int nact = inst.get_nact();
    auto* xstar = new double[2 * nact];
    my::assert(!CPXgetx(env, lp, xstar, 0, 2 * nact - 1), "CPXgetx failed.");

    // convert to std collections for easier parsing
    std::vector<std::pair<double, unsigned int>> cpx_result;
    for (unsigned int i = 0; i < nact; i++) if (xstar[i] > .5) cpx_result.emplace_back(xstar[nact+i], i);
    delete[] xstar; xstar = nullptr;

    // sort cpx_result based on actions timestamps
    std::sort(cpx_result.begin(), cpx_result.end(),
        [](const std::pair<double, unsigned int> &x, const std::pair<double, unsigned int> &y) {
            return x.first < y.first;
        }
    );

    // get solution from sorted cpx_result
    std::vector<unsigned int> solution;
    std::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution),
        [](const std::pair<double, unsigned int> &p) {
            return p.second;
        }
    );

    // store solution
    const std::vector<HPLUS_action>& actions = inst.get_actions();
    inst.update_best_solution(solution,
        std::accumulate(solution.begin(), solution.end(), 0,
            [&actions](const unsigned int acc, const unsigned int index) {
                return acc + actions[index].get_cost();
            }
        )
    );

}

void HPLUS_run_imai(HPLUS_instance& inst) {

    HPLUS_env.logger.print_info("Running imai algorithm.");

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    double start_time = HPLUS_env.get_time();

    HPLUS_cpx_init(env, lp);
    HPLUS_cpx_build_imai(env, lp, inst);                                            // baseline
    HPLUS_imai_variable_elimination(env, lp, inst);                                 // apply iterative variable elimination

    HPLUS_stats.build_time = HPLUS_env.get_time() - start_time;

    my::assert(!CPXmipopt(env, lp), "CPXmipopt failed.");

    HPLUS_parse_cplex_status(env, lp);

    if (HPLUS_env.found()) HPLUS_store_imai_sol(env, lp, inst);

    HPLUS_cpx_close(env, lp);

}