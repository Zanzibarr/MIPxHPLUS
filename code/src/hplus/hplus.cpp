#include <limits.hxx>
#include <new>

#include "cli_descriptions.hpp"
#include "constants.hpp"
#include "scope_guard.hxx"
#include "solver.hpp"
#include "utils.hpp"

void Solver::solve_hplus_() {
    global_.using_cplex = true;

    const auto& model = params_.get<cli_desc::hplus_alg, std::string>();

    // Initialize CPLEX environment
    hplus_init_cplex_();
    auto _exit_scope = on_scope_exit([this]() { this->hplus_close_cplex_(); });

    try {
        logger_[INFO] << "Building MIP formulation";

        {
            auto _build_timer = scoped_timer("build");
            // Base model is common to all (this always sets up also the global progress and relaxation callback: the latter is always enabled for
            // info
            // about global_.hplus_lp relaxation)
            hplus_build_base_model_();

            // Setup acyclicity constraints based on formulation (add variables and constraints OR setup callbacks)
            if (model == "tl") {
                hplus_add_tl_constraints_();
            } else if (model == "ve") {
                hplus_add_ve_constraints_();
            } else if (model == "base") {
                hplus_enable_candidate_callback_();
            } else {
                logger_[FATAL] << std::format("Unhandled hplus formulation: {}", model);
            }

            if (params_.get<cli_desc::primal_heur, std::string>() != "0") {
                logger_[INFO] << "Posting warm start";
                hplus_post_warm_start_();
            }
        }
    } catch (const EarlyExit& e) {
        switch (e.reason()) {
            case EarlyExit::TIMELIMIT:
                throw e;  // Let Solver::solve() handle this
                break;
            case EarlyExit::INFEASIBLE:
                [[fallthrough]];
            case EarlyExit::OPTIMAL:
                // This should never happen
                integritycheck(false, "EarlyExit::OPTIMAL thrown while building MIP formualtion");
                break;

            default:
                logger_[ERROR] << std::format("Unhandled early exit: {}", e.what());
                break;
        }
    } catch (const std::bad_alloc& e) {
        throw e;  // We cannot solve CPLEX since the build process hasn't ended, so we can just throw the execption
    } catch (const std::exception& e) {
        logger_[FATAL] << std::format("Caught exception in Solver::solve_hplus_(): {}", e.what());
    } catch (...) {
        logger_[FATAL] << std::format("Something wrong happened in Solver::solve_hplus_().");
    }

    logger_[INFO] << "Running CPLEX";
    hplus_run_cplex_();

    hplus_cplex_gather_info_();
}

void Solver::hplus_init_cplex_() {
    int cpxerror = 0;
    global_.hplus_env = CPXopenCPLEX(&cpxerror);
    call_cplex(cpxerror);
    global_.hplus_lp = CPXcreateprob(global_.hplus_env, &cpxerror, "hplus_formulation");
    call_cplex(cpxerror);
    // threads
    call_cplex(CPXsetintparam(global_.hplus_env, CPXPARAM_Threads, static_cast<CPXINT>(params_.get<cli_desc::threads, int>())));
    call_cplex(CPXsetintparam(global_.hplus_env, CPXPARAM_Parallel, CPX_PARALLEL_DETERMINISTIC));
    // log file
    call_cplex(CPXsetintparam(global_.hplus_env, CPXPARAM_ScreenOutput, CPX_OFF));
    if (params_.get<cli_desc::cpxlog, std::string>() != "0") {
        call_cplex(CPXsetlogfilename(global_.hplus_env, params_.get<cli_desc::cpxlog, std::string>().c_str(), "w"));
    }
    call_cplex(CPXsetintparam(global_.hplus_env, CPX_PARAM_CLONELOG, -1));
    call_cplex(CPXsetintparam(global_.hplus_env, CPXPARAM_MIP_Display, 3));
    // tolerance
    call_cplex(CPXsetdblparam(global_.hplus_env, CPXPARAM_MIP_Tolerances_MIPGap, 0));
    // memory/size limits
    call_cplex(CPXsetdblparam(global_.hplus_env, CPXPARAM_MIP_Limits_TreeMemory, 12000));
    call_cplex(CPXsetdblparam(global_.hplus_env, CPXPARAM_WorkMem, 4050));
    call_cplex(CPXsetintparam(global_.hplus_env, CPXPARAM_MIP_Strategy_File, 3));
    // terminate condition
    call_cplex(CPXsetterminate(global_.hplus_env, &GLOBAL_TERMINATE_CONDITION));
    // random seed
    call_cplex(CPXsetintparam(global_.hplus_env, CPXPARAM_RandomSeed, params_.get<cli_desc::seed, int>()));
}

void Solver::hplus_close_cplex_() {
    call_cplex(CPXfreeprob(global_.hplus_env, &global_.hplus_lp));
    call_cplex(CPXcloseCPLEX(&global_.hplus_env));
}

void Solver::hplus_run_cplex_() {
    auto _cpx_timer = scoped_timer("cplex");

    if (const int total_tl = params_.get<cli_desc::time_limit, int>(); total_tl < std::numeric_limits<int>::max()) {
        const double remaining = std::max(0.0, static_cast<double>(total_tl) - elapsed_seconds_());
        call_cplex(CPXsetdblparam(global_.hplus_env, CPXPARAM_TimeLimit, remaining));
    }

    call_cplex(CPXcallbacksetfunc(global_.hplus_env, global_.hplus_lp, global_.hplus_callback_context, hplus_callback_hub_, this));

    double det_start{0};
    call_cplex(CPXgetdettime(global_.hplus_env, &det_start));

    // Solve model
    call_cplex(CPXmipopt(global_.hplus_env, global_.hplus_lp));

    double det_end{0};
    call_cplex(CPXgetdettime(global_.hplus_env, &det_end));
    global_.hplus_dettime = det_end - det_start;
}

void Solver::hplus_build_base_model_() {
    auto stopcheck = []() {
        if (global_limits::time_reached()) {
            throw EarlyExit("building base model", EarlyExit::TIMELIMIT);
        }
    };

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    unsigned int curr_col{0};
    std::vector<double> objs(inst_.m);
    std::vector<double> lbs(inst_.m);
    std::vector<double> ubs(inst_.m, 1.0);
    std::vector<char> types(inst_.m, 'B');

    // -------- actions ------- //
    const unsigned int act_start{curr_col};
    unsigned int count{0};
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        objs[count] = static_cast<double>(inst_.actions[act_i].cost);
        lbs[count++] = (global_.fixed_actions[act_i] ? 1 : 0);
    }

    curr_col += count;

    call_cplex(CPXnewcols(global_.hplus_env, global_.hplus_lp, static_cast<int>(count), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
    stopcheck();

    objs.clear();
    objs.resize(inst_.n, 0.0);
    lbs.clear();
    lbs.resize(inst_.n, 0.0);
    ubs.clear();
    ubs.resize(inst_.n, 1.0);
    types.clear();
    types.resize(inst_.n, 'B');

    // --- first archievers --- //
    const unsigned int fa_start{curr_col};
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        curr_col += inst_.actions[act_i].eff_sparse.size();
        call_cplex(CPXnewcols(global_.hplus_env, global_.hplus_lp, static_cast<int>(inst_.actions[act_i].eff_sparse.size()), objs.data(), lbs.data(),
                              ubs.data(), types.data(), nullptr));
        stopcheck();
    }

    // ------- variables ------ //
    const unsigned int var_start{curr_col};
    count = 0;
    for (unsigned int var_i = 0; var_i < inst_.n; var_i++) {
        lbs[count++] = (global_.fixed_facts[var_i] || inst_.goal[var_i]) ? 1 : 0;
    }
    curr_col += count;

    call_cplex(CPXnewcols(global_.hplus_env, global_.hplus_lp, static_cast<int>(count), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
    stopcheck();

    stats_.counter_set<"n_var_base">(inst_.n + inst_.m + inst_.nfadd);

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    // accessing cplex variables
    const auto get_act_idx = [&act_start](unsigned int idx) { return static_cast<int>(act_start + idx); };
    const auto get_var_idx = [&var_start](unsigned int idx) { return static_cast<int>(var_start + idx); };
    const auto get_fa_idx = [this, &fa_start](unsigned int act_idx, unsigned int var_count) {
        return static_cast<int>(fa_start + global_.hplus_fadd_cpx_start[act_idx] + var_count);
    };

    std::vector<int> ind(inst_.m + 1);
    std::vector<double> val(inst_.m + 1);
    unsigned int nnz{0};
    constexpr char sense_e{'E'};
    constexpr char sense_l{'L'};
    constexpr char sense_g{'G'};
    constexpr double rhs_0{0};
    constexpr double rhs_1{1};
    constexpr int begin{0};

    // Equation 2 - Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    for (unsigned int var_i = 0; var_i < inst_.n; var_i++) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[(nnz++)] = 1;

        for (const auto& act_i : global_.act_with_eff[var_i]) {
            unsigned int var_count = sorted_find(inst_.actions[act_i].eff_sparse, var_i);
            ind[nnz] = get_fa_idx(act_i, var_count);
            val[(nnz++)] = -1;
        }

        // if nnz == 1, then we'd have p = 0, meaning we could simply fix this variable to 0
        if (nnz == 1) {
            const char fix = 'B';
            call_cplex(CPXchgbds(global_.hplus_env, global_.hplus_lp, 1, ind.data(), &fix, &rhs_0));
        } else {
            stats_.counter_inc<"n_const_base">();
            call_cplex(CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, static_cast<int>(nnz), &rhs_0, &sense_e, &begin, ind.data(), val.data(),
                                  nullptr, nullptr));
        }
        stopcheck();
    }

    // Equation 3 - Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    for (unsigned int fact_p = 0; fact_p < inst_.n; fact_p++) {
        for (unsigned int fact_q = 0; fact_q < inst_.n; fact_q++) {
            nnz = 0;
            ind[nnz] = get_var_idx(fact_q);
            val[(nnz++)] = -1;
            for (const auto& act_i : global_.act_with_eff[fact_p]) {
                if (!sorted_contains(inst_.actions[act_i].pre_sparse, fact_q)) {
                    continue;
                }
                unsigned int var_count = sorted_find(inst_.actions[act_i].eff_sparse, fact_p);
                ind[nnz] = get_fa_idx(act_i, var_count);
                val[(nnz++)] = 1;
            }
            // if nnz == 1 than we have -fact_p <= 0, hence it's always true, we can ignore this constraint
            if (nnz != 1) {
                stats_.counter_inc<"n_const_base">();
                call_cplex(CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, static_cast<int>(nnz), &rhs_0, &sense_l, &begin, ind.data(),
                                      val.data(), nullptr, nullptr));
            }
            stopcheck();
        }
    }
    ind.resize(2);
    val.resize(2);

    // Equation 5 - Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        for (unsigned int var_count = 0; var_count < inst_.actions[act_i].eff_sparse.size(); var_count++) {
            ind[0] = get_act_idx(act_i);
            val[0] = -1;
            ind[1] = get_fa_idx(act_i, var_count);
            val[1] = 1;
            stats_.counter_inc<"n_const_base">();
            call_cplex(CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
        stopcheck();
    }

    ind.resize(inst_.m);
    val.resize(inst_.m);

    // Equation 14 - Salvagnin, Zanella: "MIP Formulations for Delete-free AI Planning"
    // If preprocessing is used, there might be Disjunctive Action Landmarks to add as constraints... those must be counted as acyclicity constraints,
    // since those are not needed for the base model correctness
    for (const auto& landmark : global_.landmarks) {
        // If the landmark is composed of only one action, that that action has to be used -> fix it instead of creating a landmark (CPLEX
        // preprocessing would fix it anyways)
        if (landmark.size() == 1) {
            const char fix = 'B';
            ind[0] = static_cast<int>(landmark[0]);
            call_cplex(CPXchgbds(global_.hplus_env, global_.hplus_lp, 1, ind.data(), &fix, &rhs_1));
            continue;
        }
        nnz = 0;
        for (const auto& act_i : landmark) {
            ind[nnz] = static_cast<int>(act_i);
            val[(nnz++)] = 1;
        }
        stats_.counter_inc<"n_const_acyc">();
        call_cplex(CPXaddrows(global_.hplus_env, global_.hplus_lp, 0, 1, static_cast<int>(nnz), &rhs_1, &sense_g, &begin, ind.data(), val.data(),
                              nullptr, nullptr));
    }
    stopcheck();

    global_.hplus_callback_context = CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS | CPX_CALLBACKCONTEXT_RELAXATION;
}

void Solver::hplus_post_warm_start_() {
    const auto& model = params_.get<cli_desc::hplus_alg, std::string>();

    if (model == "tl") {
        hplus_post_tl_warm_start_();
    } else if (model == "ve") {
        hplus_post_ve_warm_start_();
    } else if (model == "base") {
        hplus_post_base_warm_start_();
    } else {
        logger_[FATAL] << std::format("Unhandled hplus formulation: {}", model);
    }
}

void Solver::hplus_post_base_warm_start_() {
    BinarySet state{inst_.n};
    const auto& warm_start{global_.solution};

    myassert(!warm_start.empty(), "No solution provided as warm start");

    const auto ncols = CPXgetnumcols(global_.hplus_env, global_.hplus_lp);
    std::vector<int> ind(static_cast<unsigned int>(ncols));
    std::iota(ind.begin(), ind.end(), 0);
    std::vector<double> val(static_cast<unsigned int>(ncols), 0.0);
    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_NOCHECK};

    for (const auto& act_i : warm_start) {
        val[act_i] = 1;
        int var_count{-1};
        for (const auto& var_i : inst_.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) {
                continue;
            }

            unsigned int fadd_idx = inst_.m + global_.hplus_fadd_cpx_start[act_i] + static_cast<unsigned int>(var_count);
            val[fadd_idx] = 1;
            unsigned int var_idx = inst_.m + inst_.nfadd + var_i;
            val[var_idx] = 1;
        }
        state |= inst_.actions[act_i].eff_sparse;
    }

    call_cplex(CPXaddmipstarts(global_.hplus_env, global_.hplus_lp, 1, ncols, &izero, ind.data(), val.data(), &effortlevel, nullptr));
}

void Solver::hplus_enable_candidate_callback_() { global_.hplus_callback_context |= CPX_CALLBACKCONTEXT_CANDIDATE; }

void Solver::hplus_cplex_gather_info_() {
    // ~~~~~~~~~~~ Solution Status ~~~~~~~~~~~ //
    hplus_parse_cplex_status_();

    // ~~~~~~~~~~ Gather statistics ~~~~~~~~~~ //
    stats_.counter_set<"nodes">(CPXgetnodecnt(global_.hplus_env, global_.hplus_lp));

    double lower_bound = 0;
    call_cplex(CPXgetbestobjval(global_.hplus_env, global_.hplus_lp, &lower_bound));
    try_update_best_bound_(lower_bound);

    bool expected = false;
    if (global_.relax_lb_rootnode_recorded.compare_exchange_strong(expected, true)) {
        if (stats_.counter_get<"nodes">() == 0) {
            // Never left the root: the final bound IS the root bound (also covers runs where the progress callback never fired)
            stats_.gauge_record<"lb_rootnode">(actual_bound_(global_.best_bound));
        } else if (global_.relax_last_root_lb.load() >= 0) {
            // Branched, but no progress callback fired with nodecount > 0: freeze what we observed while still at the root
            stats_.gauge_record<"lb_rootnode">(actual_bound_(global_.relax_last_root_lb));
        }
        // else (pathological: branched without any root progress event): record nothing rather than over-estimate the root bound with the final
        // one; the gauge stays at 0 samples, which is the honest "we never measured it"
    }

    // ~~~~~~~~~~~~ Best solution ~~~~~~~~~~~~ //
    hplus_get_cplex_solution_();

    // Execution-path fingerprint. dettime is the deterministic tick count CPLEX synchronizes threads on (UsrMan, "Determinism of results"):
    // reproducible and independent of wall-clock. Read it asymmetrically: a MISMATCH proves two runs diverged; a MATCH is only strong evidence of
    // the same B&B tree, not proof — the values are a lossy summary, and work in the speculative layer is not fully metered (observed: identical
    // dettime despite an extra candidate-callback invocation that injected a duplicate lazy cut).
    {
        int user_cuts{-1};
        call_cplex(CPXgetnumcuts(global_.hplus_env, global_.hplus_lp, CPX_CUT_USER, &user_cuts));
        logger_[DEBUG] << std::format(
            "CPLEX fingerprint: dettime={:.2f} ticks, simplex_iters={}, nodes={}, nodes_left={}, incumbent_node={}, user_cuts={}",
            global_.hplus_dettime, CPXgetmipitcnt(global_.hplus_env, global_.hplus_lp), CPXgetnodecnt(global_.hplus_env, global_.hplus_lp),
            CPXgetnodeleftcnt(global_.hplus_env, global_.hplus_lp), CPXgetnodeint(global_.hplus_env, global_.hplus_lp), user_cuts);
    }
}

void Solver::hplus_parse_cplex_status_() {
    switch (const int status{CPXgetstat(global_.hplus_env, global_.hplus_lp)}) {
        // ~~~~~~~~~~~ No solution find ~~~~~~~~~~ //
        case CPXMIP_MEM_LIM_INFEAS:  // exceeded memory limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_TIME_LIM_INFEAS:  // exceeded time limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_NODE_LIM_INFEAS:  // exceeded node limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_ABORT_INFEAS:  // terminated by user, not found solution
            [[fallthrough]];
        // ~~~~~~~ Feasible solution found ~~~~~~~ //
        case CPXMIP_FAIL_FEAS:  // An error occurred, but a feasible solution has been found
            [[fallthrough]];
        case CPXMIP_MEM_LIM_FEAS:  // exceeded memory limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_TIME_LIM_FEAS:  // exceeded time limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_NODE_LIM_FEAS:  // exceeded node limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_ABORT_FEAS:  // terminated by user, found solution
            [[fallthrough]];
        case CPXMIP_FEASIBLE:  // found a feasible solution
            [[fallthrough]];
        // ~~~~~~~~ Optimal solution found ~~~~~~~ //
        case CPXMIP_OPTIMAL_TOL:  // found optimal within the tollerance
            [[fallthrough]];
        case CPXMIP_OPTIMAL:  // found optimal
            // Do nothing
            break;
        // ~~~~~~~~~~ Proven infeasible ~~~~~~~~~~ //
        case CPXMIP_INFEASIBLE:  // proven to be infeasible
            throw EarlyExit("solving cplex", EarlyExit::INFEASIBLE);
            break;
        default:  // unhandled status
            logger_[FATAL] << std::format("Error in parse_cpx_status: unhandled cplex status ({})", status);
            break;
    }
}

void Solver::hplus_get_cplex_solution_() {
    std::vector<double> plan(inst_.m + inst_.nfadd, 0.0);
    switch (int code = CPXgetx(global_.hplus_env, global_.hplus_lp, plan.data(), 0, static_cast<int>(inst_.m + inst_.nfadd - 1))) {
        case CPXERR_NO_MEMORY:
            [[fallthrough]];
        case CPXERR_THREAD_FAILED:
            [[fallthrough]];
        case CPXERR_NO_SOLN:
            // We have no solution: we can simply exit (if we found solutions before they will already be stored)
            return;
        case 0:
            break;
        default:
            logger_[FATAL] << std::format("Unhandled CPLEX error code: {} at {}: {}:{}", code, __PRETTY_FUNCTION__, __FILE__, __LINE__);
            break;
    }

    // convert to a integer vector for easier parsing
    std::vector<unsigned int> cpx_result;
    cpx_result.reserve(inst_.m);
    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        if (plan[act_i] > constants::cpx_int_rounding) {
            cpx_result.push_back(act_i);
        }
    }

    std::vector<unsigned int> solution;
    solution.reserve(inst_.m);
    BinarySet remaining{static_cast<unsigned int>(cpx_result.size()), true};
    BinarySet state{inst_.n};
    double cost{0};

    // Check we are getting ALL the actions that cplex uses
    while (!state.superset_of(inst_.goal)) {
        bool intcheck{false};
        for (const auto idx : remaining) {
            if (!bs_contains(state, inst_.actions[cpx_result[idx]].pre_sparse)) {
                continue;
            }
            remaining.remove(idx);
            if (bs_contains(state, inst_.actions[cpx_result[idx]].eff_sparse)) {
                continue;  // adds nothing to a monotone state: prune it from the plan
            }

            state |= inst_.actions[cpx_result[idx]].eff_sparse;
            solution.push_back(cpx_result[idx]);
            cost += inst_.actions[cpx_result[idx]].cost;
            intcheck = true;
            if (state.superset_of(inst_.goal)) {
                break;  // goal reached: whatever is left in remaining is discarded
            }
        }
        integritycheck(intcheck, "No action can be applied on the current state");
    }

    // store solution
    // In deterministic mode we always read CPLEX solution and switch it with our own (which is non-deterministically obtained)
    const auto deterministic = params_.get<cli_desc::deterministic, bool>();
    try_update_best_incumbent_(solution, cost, deterministic);
}
