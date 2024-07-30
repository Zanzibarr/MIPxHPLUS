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

    unsigned int nvar = inst -> get_nvar();
    unsigned int nact = inst -> get_nact();
    unsigned int bfsize = inst -> get_bfsize();
    const HPLUS_action** actions = inst -> get_actions();
    const HPLUS_variable** variables = inst -> get_variables();
    const my::BitField* istate = inst -> get_istate();
    const my::BitField* gstate = inst -> get_gstate();

    // ~~~~~~~~ Adding CPLEX variables ~~~~~~~ //

    double* objs;
    double* lbs;
    double* ubs;
    char* types;
    char** names;
    unsigned int nfa = 0;

    auto lambda_add_acts = [nact, actions, &objs, &lbs, &ubs, &types, &names]() {

        for (int i = 0; i < nact; i++) {
            objs[i] = actions[i] -> get_cost();
            lbs[i] = 0.0;
            ubs[i] = 1.0;
            types[i] = 'B';
            snprintf(names[i], 20, "act(%d)", i);
        }

    };

    auto lambda_add_tacts = [nact, &objs, &lbs, &ubs, &types, &names]() {

        for (int i = 0; i < nact; i++) {
            objs[i] = 0;
            lbs[i] = 0.0;
            ubs[i] = nact-1;
            types[i] = 'I';
            snprintf(names[i], 20, "tact(%d)", i);
        }

    };

    auto lambda_add_vars = [nvar, variables, istate, gstate, &objs, &lbs, &ubs, &types, &names]() {

        for (int i = 0, count = 0; i < nvar; i++) for (int j = 0; j < variables[i] -> get_range(); j++) {
            objs[i] = 0;
            if (istate -> operator[](count) || gstate -> operator[](count)) lbs[count] = 1.0;  // fix variables of initial and goal state to 1
            ubs[count] = 1.0;
            types[count] = 'B';
            snprintf(names[count], 20, "var(%d,%d)", i, j);
            count++;
        }

    };

    auto lambda_add_tvars = [nvar, nact, variables, &objs, &lbs, &ubs, &types, &names]() {

        for (int i = 0, count = 0; i < nvar; i++) for (int j = 0; j < variables[i] -> get_range(); j++) {
            objs[i] = 0;
            lbs[count] = 1;
            ubs[count] = nact;
            types[count] = 'I';
            snprintf(names[count], 20, "tvar(%d,%d)", i, j);
            count++;
        }

    };

    auto lambda_add_fa = [nvar, nact, variables, actions, &nfa, &objs, &lbs, &ubs, &types, &names]() {

        for (int c = 0; c < nact; c++) {
            const my::BitField* eff = actions[c] -> get_eff();
            for (int i = 0, k = 0; i < nvar; k += variables[i] -> get_range(), i++) for (int j = 0; j < variables[i] -> get_range(); j++) {
                if (!eff -> operator[](k + j)) continue;    // create a variable only for the effects of the action
                objs[nfa] = 0;
                lbs[nfa] = 0.0;
                ubs[nfa] = 1.0;
                types[nfa] = 'B';
                snprintf(names[nfa], 20, "fa(%d,%d,%d)", c, i, j);
                nfa++;
            }
        }

    };

    unsigned int sizes[5] = {nact, nact, bfsize, bfsize, bfsize * nact};
    std::vector<std::function<void()>> populate_variables = {lambda_add_acts, lambda_add_tacts, lambda_add_vars, lambda_add_tvars, lambda_add_fa};

    for (int i = 0; i < 5; i++) {

        objs = new double[sizes[i]];
        lbs = new double[sizes[i]];
        ubs = new double[sizes[i]];
        types = new char[sizes[i]];
        names = new char*[sizes[i]];
        for (int j = 0; j < sizes[i]; j++) names[j] = new char[20];

        populate_variables[i]();

        MYASSERT(!CPXnewcols(*env, *lp, sizes[i], objs, lbs, ubs, types, names));

        for (int i = 0; i < sizes[i]; i++) { MYASSERT(names[i] != nullptr); delete[] names[i]; }
        MYDELL(names);
        MYDELL(types);
        MYDELL(ubs);
        MYDELL(lbs);
        MYDELL(objs);

    }

    HPLUS_env.act_start = 0;
    HPLUS_env.tact_start = nact;
    HPLUS_env.var_start = HPLUS_env.tact_start + nact;
    HPLUS_env.tvar_start = HPLUS_env.var_start + bfsize;
    HPLUS_env.fa_start = HPLUS_env.tvar_start + bfsize;
    HPLUS_env.cpx_ncols = HPLUS_env.fa_start + nfa;

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

    for (int i = 0; i < bfsize; i++) {
        ind[i] = new int[nact + 1];
        val[i] = new double[nact + 1];
        nnz[i] = 0;
        rhs[i] = istate -> operator[](i);
        ind[i][nnz[i]] = HPLUS_env.cpx_var_idx(i);
        val[i][nnz[i]] = 1;
        nnz[i]++;
    }

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
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr));
                // constraint 4: t_vj <= t_a, vj in pre(a)
                ind1[0] = HPLUS_env.cpx_tvar_idx(j);
                val1[0] = 1;
                ind1[1] = HPLUS_env.cpx_tact_idx(i);
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr));
            }
            if (eff -> operator[](j)) {
                // constraint 2: z_avj <= x_a, vj in eff(a)
                ind1[0] = HPLUS_env.cpx_fa_idx(count_fa);
                val1[0] = 1;
                ind1[1] = HPLUS_env.cpx_act_idx(i);
                val1[1] = -1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz1, &rhs1, &sensel, &begin, ind1, val1, nullptr, nullptr));
                // constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
                ind2[0] = HPLUS_env.cpx_tact_idx(i);
                val2[0] = 1;
                ind2[1] = HPLUS_env.cpx_tvar_idx(j);
                val2[1] = -1;
                ind2[2] = HPLUS_env.cpx_fa_idx(count_fa);
                val2[2] = nact + 1;
                MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz2, &rhs2, &sensel, &begin, ind2, val2, nullptr, nullptr));
                // constraint 3: I(v_j) + sum(z_avj) = y_vj
                ind[j][nnz[j]] = HPLUS_env.cpx_fa_idx(count_fa++);
                val[j][nnz[j]] = -1;
                nnz[j]++;
            }
        }
    }

    for (int i = 0; i < bfsize; i++) MYASSERT(!CPXaddrows(*env, *lp, 0, 1, nnz[i], &rhs[i], &sensee, &begin, ind[i], val[i], nullptr, nullptr));

    MYASSERT(!CPXwriteprob(*env, *lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), nullptr));

    for (int i = 0; i < bfsize; i++) {
        MYDELL(ind[i]);
        MYDELL(val[i]);
    }

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

    MYFREE(xstar);

    std::sort(result.begin(), result.end(), [](const std::pair<double, std::string> &x, const std::pair<double, std::string> &y) {
        return x.first < y.first;
    });

    for (int i = 0; i < result.size(); i++) HPLUS_env.logger.print_info("%.1f : %s.", result[i].first, result[i].second.c_str());

    // TODO: Integrity check on the returned solution

}