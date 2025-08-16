#include "exact.hpp"

void exact::build_base_model(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp) {
    if (BASIC_VERBOSE()) LOG_INFO << "Building base model for exact search";

    auto stopcheck = []() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    unsigned int curr_col{0};
    std::vector<double> objs(inst.m);
    std::vector<double> lbs(inst.m);
    std::vector<double> ubs(inst.m, 1.0);
    std::vector<char> types(inst.m, 'B');

    // -------- actions ------- //
    const unsigned int act_start{curr_col};
    unsigned int count{0};
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        objs[count] = static_cast<double>(inst.actions[act_i].cost);
        lbs[count++] = (inst.fixed_actions[act_i] ? 1 : 0);
    }

    curr_col += count;

    CPX_HANDLE_CALL(CPXnewcols(env, lp, count, objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
    stopcheck();

    objs.clear();
    objs.resize(inst.n, 0.0);
    lbs.clear();
    lbs.resize(inst.n, 0.0);
    ubs.clear();
    ubs.resize(inst.n, 1.0);
    types.clear();
    types.resize(inst.n, 'B');

    // --- first archievers --- //
    const unsigned int fa_start{curr_col};
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        curr_col += inst.actions[act_i].eff_sparse.size();
        CPX_HANDLE_CALL(CPXnewcols(env, lp, inst.actions[act_i].eff_sparse.size(), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
        stopcheck();
    }

    // ------- variables ------ //
    const unsigned int var_start{curr_col};
    count = 0;
    for (unsigned int var_i = 0; var_i < inst.n; var_i++) {
        lbs[count++] = (inst.fixed_facts[var_i] || inst.goal[var_i]) ? 1 : 0;
    }
    curr_col += count;

    CPX_HANDLE_CALL(CPXnewcols(env, lp, count, objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
    stopcheck();

    stats.var_base = inst.n + inst.m + inst.nfadd;

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const auto get_act_idx = [&act_start](unsigned int idx) { return static_cast<int>(act_start + idx); };
    const auto get_var_idx = [&var_start](unsigned int idx) { return static_cast<int>(var_start + idx); };
    const auto get_fa_idx = [&inst, &fa_start](unsigned int act_idx, unsigned int var_count) {
        return static_cast<int>(fa_start + inst.fadd_cpx_start[act_idx] + var_count);
    };

    std::vector<int> ind(inst.m + 1);
    std::vector<double> val(inst.m + 1);
    int nnz{0};
    constexpr char sense_e{'E'}, sense_l{'L'}, sense_g{'G'};
    constexpr double rhs_0{0}, rhs_1{1};
    constexpr int begin{0};

    for (unsigned int var_i = 0; var_i < inst.n; var_i++) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[nnz++] = 1;

        for (const auto& act_i : inst.act_with_eff[var_i]) {
            unsigned int var_count =
                static_cast<unsigned int>(std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), var_i) -
                                          inst.actions[act_i].eff_sparse.begin());
            ind[nnz] = get_fa_idx(act_i, var_count);
            val[nnz++] = -1;
        }

        // if nnz == 1, then we'd have p = 0, meaning we could simply fix this variable to 0
        if (nnz == 1) {
            const char fix = 'B';
            CPX_HANDLE_CALL(CPXchgbds(env, lp, 1, ind.data(), &fix, &rhs_0));
        } else {
            stats.const_base++;
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
        stopcheck();
    }

    for (unsigned int p = 0; p < inst.n; p++) {
        for (unsigned int q = 0; q < inst.n; q++) {
            nnz = 0;
            ind[nnz] = get_var_idx(q);
            val[nnz++] = -1;
            for (const auto& act_i : inst.act_with_eff[p]) {
                if (!inst.actions[act_i].pre[q]) continue;
                unsigned int var_count =
                    static_cast<unsigned int>(std::find(inst.actions[act_i].eff_sparse.begin(), inst.actions[act_i].eff_sparse.end(), p) -
                                              inst.actions[act_i].eff_sparse.begin());
                ind[nnz] = get_fa_idx(act_i, var_count);
                val[nnz++] = 1;
            }
            // if nnz == 1 than we have -p <= 0, hence it's always true, we can ignore this constraint
            if (nnz != 1) {
                stats.const_base++;
                CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
            }
            stopcheck();
        }
    }
    ind.resize(2);
    val.resize(2);

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        for (unsigned int var_count = 0; var_count < inst.actions[act_i].eff_sparse.size(); var_count++) {
            ind[0] = get_act_idx(act_i);
            val[0] = -1;
            ind[1] = get_fa_idx(act_i, var_count);
            val[1] = 1;
            stats.const_base++;
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
        stopcheck();
    }

    // If preprocessing is used, there might be Disjunctive Action Landmarks to add as constraints... those must be counted as acyclicity constraints,
    // since those are not needed for the base model correctness
    if (exec.prep) {
        for (const auto& landmark : inst.landmarks) {
            // TODO: remove once properly tested
            // Debugging -> landmarks should not be empty
            ASSERT(!landmark.empty());
            // If the landmark is composed of only one action, that that action has to be used -> fix it instead of creating a landmark (CPLEX
            // preprocessing would fix it anyways)
            if (landmark.size() == 1) {
                const char fix = 'B';
                ind[0] = static_cast<int>(landmark[0]);
                CPX_HANDLE_CALL(CPXchgbds(env, lp, 1, ind.data(), &fix, &rhs_1));
                continue;
            }
            nnz = 0;
            for (const auto& act_i : landmark) {
                ind[nnz] = static_cast<int>(act_i);
                val[nnz++] = 1;
            }
            stats.const_acyc++;
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz, &rhs_1, &sense_g, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
        stopcheck();
    }
}

void parse_cplex_status(const CPXENVptr& env, const CPXLPptr& lp, const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    if (BASIC_VERBOSE()) LOG_INFO << "Parsing CPLEX status";
    std::vector<double> tmp(1);
    switch (CPXgetx(env, lp, tmp.data(), 0, 0)) {
        case CPXERR_NO_SOLN:  // No solution found
            if (exec.ws == hplus::warmstart::NONE) inst.sol_s = hplus::solution_status::NOTFOUND;
            return;
        default:
            break;
    }

    switch (const int status{CPXgetstat(env, lp)}) {
        case CPXMIP_FAIL_FEAS:  // An error occurred, but a feasible solution has been found
            [[fallthrough]];
        case CPXMIP_MEM_LIM_FEAS:  // exceeded memory limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_TIME_LIM_FEAS:  // exceeded time limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_ABORT_FEAS:  // terminated by user, found solution
            inst.sol_s = hplus::solution_status::FEAS;
            break;
        case CPXMIP_MEM_LIM_INFEAS:  // exceeded memory limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_TIME_LIM_INFEAS:  // exceeded time limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_ABORT_INFEAS:  // terminated by user, not found solution
            if (exec.ws == hplus::warmstart::NONE) inst.sol_s = hplus::solution_status::NOTFOUND;
            break;
        case CPXMIP_INFEASIBLE:  // proven to be infeasible
            inst.sol_s = hplus::solution_status::INFEAS;
            break;
        case CPXMIP_OPTIMAL_TOL:  // found optimal within the tollerance
            [[fallthrough]];
        case CPXMIP_OPTIMAL:  // found optimal
            inst.sol_s = hplus::solution_status::OPT;
            break;
        default:  // unhandled status
            LOG_ERROR << "Error in parse_cpx_status: unhandled cplex status (" << status << ")";
            break;
    }

    switch (inst.sol_s) {
        case hplus::solution_status::OPT:
            stats.status = HPLUS_STATUS_OPT;
            break;
        case hplus::solution_status::INFEAS:
            stats.status = HPLUS_STATUS_INFEAS;
            break;
        case hplus::solution_status::FEAS:
            stats.status = HPLUS_STATUS_FEAS;
            break;
        case hplus::solution_status::NOTFOUND:
            stats.status = HPLUS_STATUS_NOTFOUND;
            break;
        default:
            LOG_ERROR << "Unhandled solution status: " << static_cast<int>(inst.sol_s);
    }
}

void store_cplex_solution(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, const CPXENVptr& env, const CPXLPptr& lp) {
    std::vector<double> plan(inst.m + inst.nfadd, 0.0);
    switch (int code = CPXgetx(env, lp, plan.data(), 0, inst.m + inst.nfadd - 1)) {
        case CPXERR_NO_MEMORY:
            [[fallthrough]];
        case CPXERR_THREAD_FAILED:
            throw std::bad_alloc();
            break;
        case CPXERR_NO_SOLN:
            return;
        case 0:
            break;
        default:
            LOG_ERROR << "Unhandled CPLEX error code: " << code << " at " << __func__ << "(): " << __FILE__ << ":" << __LINE__;
            break;
    }

    // fixing the solution to read the plan (some 0-cost actions are set to 1 even if they are not a first archiever of anything)
    if (exec.alg != hplus::algorithm::CUTS) {
        for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
            bool set_zero{true};
            for (unsigned int var_count = 0; var_count < inst.actions[act_i].eff_sparse.size(); var_count++) {
                if (plan[inst.m + inst.fadd_cpx_start[act_i] + var_count] > HPLUS_CPX_INT_ROUNDING) {
                    ASSERT(plan[act_i] > HPLUS_CPX_INT_ROUNDING);
                    set_zero = false;
                    break;
                }
            }
            if (set_zero) plan[act_i] = 0;
        }
    }

    // convert to a vector of int for easier parsing
    std::vector<unsigned int> cpx_result;
    cpx_result.reserve(inst.m);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (plan[act_i] > HPLUS_CPX_INT_ROUNDING) cpx_result.push_back(act_i);
    }

    std::vector<unsigned int> solution;
    solution.reserve(inst.m);
    binary_set remaining{static_cast<unsigned int>(cpx_result.size()), true}, state{inst.n};
    unsigned int cost{0};

    // Check we are getting ALL the actions that cplex uses
    while (!remaining.empty()) {
        bool intcheck{false};
        for (const auto& i : remaining) {
            if (!state.contains(inst.actions[cpx_result[i]].pre)) continue;

            remaining.remove(i);
            state |= inst.actions[cpx_result[i]].eff;
            solution.push_back(cpx_result[i]);
            intcheck = true;
            cost += inst.actions[cpx_result[i]].cost;
        }
        ASSERT(intcheck);
    }

    // store solution
    hplus::solution sol{solution, cost};
    hplus::update_sol(exec, inst, sol, stats);
}

void exact::get_cplex_solution(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, const CPXENVptr& env, const CPXLPptr& lp) {
    parse_cplex_status(env, lp, exec, inst, stats);

    stats.nodes = CPXgetnodecnt(env, lp);

    if (inst.sol_s > hplus::solution_status::FEAS) return;

    CPX_HANDLE_CALL(CPXgetbestobjval(env, lp, &stats.lower_bound));
    if (stats.lower_bound < 0) stats.lower_bound = 0;

    if (BASIC_VERBOSE()) LOG_INFO << "Reading CPLEX solution";
    store_cplex_solution(exec, inst, stats, env, lp);
}