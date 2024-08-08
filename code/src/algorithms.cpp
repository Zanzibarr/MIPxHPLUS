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
 * Fact landmarks extracting method from Imai's paper
 */
void HPLUS_imai_extract_landmarks(const HPLUS_instance& inst, std::vector<my::BitField>& landmarks_set, my::BitField& fact_landmarks, my::BitField& act_landmarks) {

    const unsigned int nvarstrips = inst.get_nvarstrips();

    landmarks_set = std::vector<my::BitField>(nvarstrips);
    const my::BitField& istate = inst.get_istate();
    for (unsigned int p = 0; p < nvarstrips; p++) {
        // using the bit at position nvarstrips as a flag for "full set of variables"
        landmarks_set[p] = my::BitField(nvarstrips + 1);
        if (istate[p]) landmarks_set[p].set(p);
        else landmarks_set[p].set(nvarstrips);
    }

    const std::vector<HPLUS_action>& actions = inst.get_actions();
    my::BitField s(inst.get_istate());

    std::deque<int> actions_queue;
    for (unsigned int i = 0; i < inst.get_nact(); i++)
        if(s.contains(actions[i].get_pre())) actions_queue.push_back(i);

    while(!actions_queue.empty()) {

        const HPLUS_action a = actions[actions_queue.front()];
        actions_queue.pop_front();

        const my::BitField& pre_a = a.get_pre();
        const my::BitField& add_a = a.get_eff();

        for (auto p : add_a) {

            s.set(p);

            my::BitField x = my::BitField(nvarstrips + 1);
            for (auto p : add_a) x.set(p);

            for (auto pp : pre_a) {
                // if variable p has the "full" flag then the unification
                // generates a "full" bitfield -> no need to unificate, just set the flag
                if (landmarks_set[pp][nvarstrips]) {
                    x.set(nvarstrips);
                    // if x is now full we can exit, since all further unions wont change x
                    break;
                } else x.unificate(landmarks_set[pp]);
            }

            // we then check if L[p] != X, and if they are the same we skip,
            // if X = P, then (X intersection L[P]) = L[P], hence we can already skip
            if (!x[nvarstrips]) {

                // if the set for variable p is the full set of variables,
                // the intersection generates back x -> we can skip the intersection
                if (!landmarks_set[p][nvarstrips]) x.intersect(landmarks_set[p]);

                // we already know that x is not the full set now, so if
                // the set for variable p is the full set, we know that x is not
                // equal to the set for variable p -> we can skip the check
                if (landmarks_set[p][nvarstrips] || !x.equals(landmarks_set[p])) {

                    landmarks_set[p] = my::BitField(x);
                    for (unsigned int i = 0; i < inst.get_nact(); i++) if (actions[i].get_pre()[p]) {
                        if (s.contains(actions[i].get_pre()) && std::find(actions_queue.begin(), actions_queue.end(), i) == actions_queue.end())
                            actions_queue.push_back(i);
                    }

                }
            }

        }

    }

    fact_landmarks.unificate(istate);

    unsigned int cand_act;
    for (auto i : inst.get_gstate()) for (unsigned int j = 0; j < nvarstrips; j++)
        if (!fact_landmarks[j] && (landmarks_set[i][j] || landmarks_set[i][nvarstrips])) {
            fact_landmarks.set(j);                          // fact landmarks
            unsigned int count = 0;
            for (unsigned int act_i = 0; act_i < inst.get_nact() && count <= 1; act_i++) if (actions[act_i].get_eff()[j]) {
                cand_act = act_i;
                count++;
            }
            if (count == 1) act_landmarks.set(cand_act);    // action landmarks
        }

}

void HPLUS_imai_extract_fadd(const HPLUS_instance& inst, const std::vector<my::BitField>& landmarks_set, std::vector<my::BitField>& fadd) {

    const auto& actions = inst.get_actions();
    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvarstrips();

    fadd = std::vector<my::BitField>(nact);

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        fadd[act_i] = my::BitField(nvarstrips);
        const my::BitField& pre = actions[act_i].get_pre();
        const my::BitField& eff = actions[act_i].get_eff();
        my::BitField f_lm_a(nvarstrips);
        for (auto p : pre) for (unsigned int i = 0; i < nvarstrips; i++) if (landmarks_set[p][i] || landmarks_set[p][nvarstrips]) f_lm_a.set(i);
        for (auto p : eff) if (!f_lm_a[p]) fadd[act_i].set(p);
    }

}

void HPLUS_imai_extract_relevant(const HPLUS_instance& inst, std::vector<my::BitField>& fadd,my::BitField& relevant_variables, my::BitField& relevant_actions) {

    const auto& actions = inst.get_actions();
    const auto nact = inst.get_nact();

    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        for (auto p : inst.get_gstate()) if (fadd[act_i][p]) {
            relevant_actions.set(act_i);
            relevant_variables.unificate(actions[act_i].get_pre());
            break;
        }
    }

    std::vector<unsigned int> cand_actions;
    for (unsigned int i = 0; i < nact; i++) if (!relevant_actions[i]) cand_actions.push_back(i);
    bool new_act = true;
    while (new_act) {
        new_act = false;
        for (unsigned int i = 0; i < cand_actions.size(); i++) {
            for (auto p : actions[cand_actions[i]].get_eff()) if (relevant_variables[p]) {
                relevant_actions.set(cand_actions[i]);
                relevant_variables.unificate(actions[cand_actions[i]].get_pre());
                cand_actions.erase(cand_actions.begin() + i);
                new_act = true;
                break;
            }
            if (new_act) break;
        }
    }

}

/**
 * Build the basic Imai model
*/
void HPLUS_cpx_build_imai(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst) {

    const unsigned int nvar = inst.get_nvar();
    const unsigned int nact = inst.get_nact();
    const unsigned int nvarstrips = inst.get_nvarstrips();
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

    // landmarks
    std::vector<my::BitField> landmarks_set;
    my::BitField fact_landmarks(nvarstrips);
    my::BitField act_landmarks(nact);
    HPLUS_imai_extract_landmarks(inst, landmarks_set, fact_landmarks, act_landmarks);

    // first archievers
    std::vector<my::BitField> fadd;
    HPLUS_imai_extract_fadd(inst, landmarks_set, fadd);

    // relevance analysis
    my::BitField relevant_variables(gstate);
    my::BitField relevant_actions(nact);
    HPLUS_imai_extract_relevant(inst, fadd, relevant_variables, relevant_actions);

    // actions
    unsigned int act_start = curr_col;
    for (unsigned int act_i = 0; act_i < nact; act_i++) {
        objs[act_i] = actions[act_i].get_cost();
        lbs[act_i] = act_landmarks[act_i] ? 1 : 0;
        ubs[act_i] = relevant_actions[act_i] ? 1 : 0;
        types[act_i] = 'B';
        snprintf(names[act_i], 20, "act(%d)", act_i);
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

    objs = new double[nvarstrips];
    lbs = new double[nvarstrips];
    ubs = new double[nvarstrips];
    types = new char[nvarstrips];
    names = new char*[nvarstrips];
    for (unsigned int i = 0; i < nvarstrips; i++) names[i] = new char[20];

    // variables
    unsigned int var_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i].get_range(); j++, count++) {
        objs[count] = 0;
        lbs[count] = (fact_landmarks[count] || gstate[count]) ? 1 : 0;
        ubs[count] = (relevant_variables[count] || fact_landmarks[count]) ? 1 : 0;
        types[count] = 'B';
        snprintf(names[count], 20, "var(%d_%d)", i, j);
    }
    curr_col += nvarstrips;

    my::assert(!CPXnewcols(env, lp, nvarstrips, objs, lbs, ubs, types, names), "CPXnewcols (variables) failed.");

    // variable timestamps
    unsigned int tvar_start = curr_col;
    for (unsigned int i = 0, count = 0; i < nvar; i++) for (unsigned int j = 0; j < variables[i].get_range(); j++, count++) {
        objs[count] = 0;
        lbs[count] = istate[count] ? 0 : 1;                     // if a variable is in the initial state, its timestamp can be fixed to 0
        ubs[count] = istate[count] ? 0 : nact;
        types[count] = 'I';
        snprintf(names[count], 20, "tvar(%d_%d)", i, j);
    }
    curr_col += nvarstrips;

    my::assert(!CPXnewcols(env, lp, nvarstrips, objs, lbs, ubs, types, names), "CPXnewcols (variable timestamps) failed.");

    for (unsigned int i = 0; i < nvarstrips; i++) delete[] names[i];
    delete[] names; names = nullptr;
    delete[] types; types = nullptr;
    delete[] ubs; ubs = nullptr;
    delete[] lbs; lbs = nullptr;
    delete[] objs; objs = nullptr;

    objs = new double[nact * nvarstrips];
    lbs = new double[nact * nvarstrips];
    ubs = new double[nact * nvarstrips];
    types = new char[nact * nvarstrips];
    names = new char*[nact * nvarstrips];
    for (unsigned int i = 0; i < nact * nvarstrips; i++) names[i] = new char[20];

    // first archievers
    unsigned int nfa = 0;
    unsigned int fa_start = curr_col;
    for (unsigned int c = 0; c < nact; c++) {
        const my::BitField& eff = actions[c].get_eff();
        for (unsigned int i = 0, k = 0; i < nvar; k += variables[i].get_range(), i++) for (unsigned int j = 0; j < variables[i].get_range(); j++) {
            if (!eff[k + j]) continue;
            objs[nfa] = 0;
            lbs[nfa] = 0;
            ubs[nfa] = fadd[c][k+j] ? 1 : 0;
            types[nfa] = 'B';
            snprintf(names[nfa], 20, "fa(%d_%d_%d)", c, i, j);
            nfa++;
        }
    }
    curr_col += nfa;

    my::assert(!CPXnewcols(env, lp, nfa, objs, lbs, ubs, types, names), "CPXnewcols (first archievers) failed.");

    for (unsigned int i = 0; i < nact * nvarstrips; i++) delete[] names[i];
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

    std::vector<int*> ind(nvarstrips);
    std::vector<double*> val(nvarstrips);
    std::vector<int> nnz(nvarstrips);
    std::vector<double> rhs(nvarstrips);

    for (unsigned int i = 0; i < nvarstrips; i++) {
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
        for (unsigned int j = 0; j < nvarstrips; j++) {
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

    for (unsigned int i = 0; i < nvarstrips; i++) my::assert(!CPXaddrows(env, lp, 0, 1, nnz[i], &rhs[i], &sensee, &begin, ind[i], val[i], nullptr, nullptr), "CPXaddrows (c3) failed.");

    for (unsigned int i = 0; i < nvarstrips; i++) {
        delete[] ind[i]; ind[i] = nullptr;
        delete[] val[i]; val[i] = nullptr;
    }

    my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    HPLUS_env.logger.print_info("Created CPLEX lp for imai.");

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
    HPLUS_cpx_build_imai(env, lp, inst);

    HPLUS_stats.build_time = HPLUS_env.get_time() - start_time;

    my::assert(!CPXmipopt(env, lp), "CPXmipopt failed.");

    HPLUS_parse_cplex_status(env, lp);

    if (HPLUS_env.found()) HPLUS_store_imai_sol(env, lp, inst);

    HPLUS_cpx_close(env, lp);

}