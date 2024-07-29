#include "../include/algorithms.hpp"

// ##################################################################### //
// ############################### CPLEX ############################### //
// ##################################################################### //

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

    // terminate condition
    MYASSERT(!CPXsetterminate(*env, &HPLUS_env.cpx_terminate));

}

// ##################################################################### //
// ################################ IMAI ############################### //
// ##################################################################### //

void HPLUS_cpx_build_imai(CPXENVptr* env, CPXLPptr* lp, HPLUS_instance* inst) {

    // ~~~~~~~~ Adding CPLEX variables ~~~~~~~ //

    // Actions

    unsigned int nact = inst -> get_nact();
    double* act_objs = (double*)malloc(nact * sizeof(double));
    double* act_lb = (double*)calloc(nact, sizeof(double));
    double* act_ub = (double*)malloc(nact * sizeof(double));
    char* act_type = (char*)malloc(nact * sizeof(char));
    char** act_names = (char**)malloc(nact * sizeof(char*));

    const HPLUS_action** actions = inst -> get_actions();
    for (int i = 0; i < nact; i++) {
        act_objs[i] = actions[i] -> get_cost();
        act_ub[i] = 1.0;
        act_type[i] = 'B';
        act_names[i] = (char*)malloc(20 * sizeof(char));
        snprintf(act_names[i], 20, "act(%d)", i);
    }

    HPLUS_env.act_start = 0;
    MYASSERT(!CPXnewcols(*env, *lp, nact, act_objs, act_lb, act_ub, act_type, act_names));

    for (int i = 0; i < nact; i++) { MYASSERT(act_names[i] != nullptr); free(act_names[i]); }
    MYFREE(act_names);
    MYFREE(act_type);
    MYFREE(act_ub);
    MYFREE(act_lb);
    MYFREE(act_objs);

    // Timestamps actions

    double* t_act_objs = (double*)calloc(nact, sizeof(double));
    double* t_act_lb = (double*)calloc(nact, sizeof(double));
    double* t_act_ub = (double*)malloc(nact * sizeof(double));
    char* t_act_type = (char*)malloc(nact * sizeof(char));
    char** t_act_names = (char**)malloc(nact * sizeof(char*));

    for (int i = 0; i < nact; i++) {
        t_act_ub[i] = nact-1;
        t_act_type[i] = 'I';
        t_act_names[i] = (char*)malloc(20*sizeof(char));
        snprintf(t_act_names[i], 20, "tact(%d)", i);
    }

    HPLUS_env.tact_start = nact;
    MYASSERT(!CPXnewcols(*env, *lp, nact, t_act_objs, t_act_lb, t_act_ub, t_act_type, t_act_names));

    for (int i = 0; i < nact; i++) { MYASSERT(t_act_names[i] != nullptr); free(t_act_names[i]); }
    MYFREE(t_act_names);
    MYFREE(t_act_type);
    MYFREE(t_act_ub);
    MYFREE(t_act_lb);
    MYFREE(t_act_objs);

    // Variables

    unsigned int nvar = inst -> get_nvar();
    unsigned int bfsize = inst -> get_bfsize();
    double* var_objs = (double*)calloc(bfsize, sizeof(double));
    double* var_lb = (double*)calloc(bfsize, sizeof(double));
    double* var_ub = (double*)malloc(bfsize * sizeof(double));
    char* var_type = (char*)malloc(bfsize * sizeof(char));
    char** var_names = (char**)malloc(bfsize * sizeof(char*));

    const HPLUS_variable** variables = inst -> get_variables();
    const my::BitField* istate = inst -> get_istate();
    const my::BitField* gstate = inst -> get_gstate();
    for (int i = 0, count = 0; i < nvar; i++) for (int j = 0; j < variables[i] -> get_range(); j++) {
        if (istate -> operator[](count) || gstate -> operator[](count)) var_lb[count] = 1.0;  // fix variables of initial and goal state to 1
        var_ub[count] = 1.0;
        var_type[count] = 'B';
        var_names[count] = (char*)malloc(20 * sizeof(char));
        snprintf(var_names[count], 20, "var(%d,%d)", i, j);
        count++;
    }

    HPLUS_env.var_start = HPLUS_env.tact_start + nact;
    MYASSERT(!CPXnewcols(*env, *lp, bfsize, var_objs, var_lb, var_ub, var_type, var_names));

    for (int i = 0; i < bfsize; i++) { MYASSERT(var_names[i] != nullptr); free(var_names[i]); }
    MYFREE(var_names);
    MYFREE(var_type);
    MYFREE(var_ub);
    MYFREE(var_lb);
    MYFREE(var_objs);

    // Timestamps variables

    double* t_var_objs = (double*)calloc(bfsize, sizeof(double));
    double* t_var_lb = (double*)malloc(bfsize * sizeof(double));
    double* t_var_ub = (double*)malloc(bfsize * sizeof(double));
    char* t_var_type = (char*)malloc(bfsize * sizeof(char));
    char** t_var_names = (char**)malloc(bfsize * sizeof(char*));

    for (int i = 0, count = 0; i < nvar; i++) for (int j = 0; j < variables[i] -> get_range(); j++) {
        t_var_lb[count] = 1;
        t_var_ub[count] = nact;
        t_var_type[count] = 'I';
        t_var_names[count] = (char*)malloc(20*sizeof(char));
        snprintf(t_var_names[count], 20, "tvar(%d,%d)", i, j);
        count++;
    }

    HPLUS_env.tvar_start = HPLUS_env.var_start + bfsize;
    MYASSERT(!CPXnewcols(*env, *lp, bfsize, t_var_objs, t_var_lb, t_var_ub, t_var_type, t_var_names));

    for (int i = 0; i < bfsize; i++) { MYASSERT(t_var_names[i] != nullptr); free(t_var_names[i]); }
    MYFREE(t_var_names);
    MYFREE(t_var_type);
    MYFREE(t_var_ub);
    MYFREE(t_var_lb);
    MYFREE(t_var_objs);

    // First archievers

    double* fa_objs = (double*)calloc(bfsize * nact, sizeof(double));
    double* fa_lb = (double*)calloc(bfsize * nact, sizeof(double));
    double* fa_ub = (double*)malloc(bfsize * nact * sizeof(double));
    char* fa_type = (char*)malloc(bfsize * nact * sizeof(char));
    char** fa_names = (char**)malloc(bfsize * nact * sizeof(char*));
    unsigned int nfa = 0;

    for (int c = 0; c < nact; c++) {
        const my::BitField* eff = actions[c] -> get_eff();
        for (int i = 0, k = 0; i < nvar; k += variables[i] -> get_range(), i++) for (int j = 0; j < variables[i] -> get_range(); j++) {
            if (!eff -> operator[](k + j)) continue;    // create a variable only for the effects of the action
            fa_ub[nfa] = 1.0;
            fa_type[nfa] = 'B';
            fa_names[nfa] = (char*)malloc(20 * sizeof(char));
            snprintf(fa_names[nfa], 20, "fa(%d,%d,%d)", c, i, j);
            nfa++;
        }
    }

    HPLUS_env.fa_start = HPLUS_env.tvar_start + bfsize;
    MYASSERT(!CPXnewcols(*env, *lp, nfa, fa_objs, fa_lb, fa_ub, fa_type, fa_names));

    for (int i = 0; i < nfa; i++) { MYASSERT(fa_names[i] != nullptr); free(fa_names[i]); }
    MYFREE(fa_names);
    MYFREE(fa_type);
    MYFREE(fa_ub);
    MYFREE(fa_lb);
    MYFREE(fa_objs);

    HPLUS_env.cpx_nvar = HPLUS_env.fa_start + nfa;

    // ~~~~~~~ Adding CPLEX constraints ~~~~~~ //

    int ind1[2], ind2[3];
    double val1[2], val2[3];
    int nnz1 = 2, nnz2 = 3;
    char sense = 'L';
    double rhs1 = 0, rhs2 = nact;
    int begin = 0;

    for (unsigned int i = 0, count_fa = 0; i < nact; i++) {
        const my::BitField* pre = actions[i] -> get_pre();
        const my::BitField* eff = actions[i] -> get_eff();
        for (unsigned int j = 0; j < bfsize; j++) {
            if (pre -> operator[](j)) {
                // constraint 1: x_a <= y_vj, vj in pre(a)
                ind1[0] = HPLUS_env.cpx_act_idx(i);
                val1[0] = 1;
                ind1[1] = HPLUS_env.cpx_var_idx(j);
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sense, &begin, ind1, val1, nullptr, nullptr));
                // constraint 4: t_vj <= t_a, vj in pre(a)
                ind1[0] = HPLUS_env.cpx_tvar_idx(j);
                val1[0] = 1;
                ind1[1] = HPLUS_env.cpx_tact_idx(i);
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sense, &begin, ind1, val1, nullptr, nullptr));
            }
            if (eff -> operator[](j)) {
                // constraint 2: z_avj <= x_a, vj in eff(a)
                ind1[0] = HPLUS_env.cpx_fa_idx(count_fa);
                val1[0] = 1;
                ind1[1] = HPLUS_env.cpx_act_idx(i);
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sense, &begin, ind1, val1, nullptr, nullptr));
                // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
                ind2[0] = HPLUS_env.cpx_tact_idx(i);
                val2[0] = 1;
                ind2[1] = HPLUS_env.cpx_tvar_idx(j);
                val2[1] = -1;
                ind2[2] = HPLUS_env.cpx_fa_idx(count_fa);
                val2[2] = nact + 1;
                count_fa++;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz2, &rhs2, &sense, &begin, ind2, val2, nullptr, nullptr));
            }
        }
    }

    // constraint 3: I(v_j) + sum(z_avj) <= y_vj

    int** ind = new int*[bfsize];
    double** val = new double*[bfsize];
    int* nnz = new int[bfsize];
    double* rhs = new double[bfsize];
    sense = 'E';

    for (int i = 0; i < bfsize; i++) {
        ind[i] = new int[nvar + 1];
        val[i] = new double[nvar + 1];
        nnz[i] = 0;
        rhs[i] = istate -> operator[](i);
        ind[i][nnz[i]] = HPLUS_env.cpx_var_idx(i);
        val[i][nnz[i]] = 1;
        nnz[i]++;
    }

    for (int i = 0, fa_count=0; i < nact; i++) {
        for (int j = 0; j < bfsize; j++) {
            if (actions[i] -> get_eff() -> operator[](j)) {
                ind[j][nnz[j]] = HPLUS_env.cpx_fa_idx(fa_count++);
                val[j][nnz[j]] = -1;
                nnz[j]++;
            }
        }
    }

    for (int i = 0; i < bfsize; i++) MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz[i], &rhs[i], &sense, &begin, ind[i], val[i], nullptr, nullptr));

    for (int i = 0; i < bfsize; i++) {
        delete[] ind[i];
        delete[] val[i];
    }
    MYDELL(ind);
    MYDELL(val);

    HPLUS_env.logger.print_info("Created CPLEX lp for imai.");

}

void HPLUS_run_imai(HPLUS_instance* inst) {

    HPLUS_env.logger.print_info("Running imai algorithm.");

    CPXENVptr env = nullptr;
    CPXLPptr lp = nullptr;

    HPLUS_cpx_init(&env, &lp);
    HPLUS_cpx_build_imai(&env, &lp, inst);

    MYASSERT(!CPXmipopt(env, lp));

    // Temporary viewing the solution

    double* xstar = (double*)malloc(2 * inst -> get_nact() * sizeof(double));
    MYASSERT(!CPXgetx(env, lp, xstar, 0, 2 * inst -> get_nact()));

    const HPLUS_action** actions = inst -> get_actions();
    std::vector<std::pair<double, std::string>> result;

    for (int i = 0; i < inst -> get_nact(); i++) {
        std::pair<double, std::string> p;
        p.first = xstar[inst -> get_nact() + i];
        p.second = *actions[i] -> get_name();
        if (xstar[i]) result.push_back(p);
    }

    std::sort(result.begin(), result.end(), [](const std::pair<double, std::string> &x,
                                       const std::pair<double, std::string> &y) {
        return x.first < y.first;
    });

    for (int i = 0; i < result.size(); i++) HPLUS_env.logger.print_info("%.1f : %s.", result[i].first, result[i].second.c_str());

    MYASSERT(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), nullptr));

    // TODO: Integrity check on the returned solution

}