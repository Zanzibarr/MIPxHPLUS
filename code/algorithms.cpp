#include "algorithms.hpp"

// #include <deque>

// #if HPLUS_INTCHECK == 0
// #define INTCHECK_PQ false
// #endif
// #include "pq.hxx"

// ##################################################################### //
// ############################ CPLEX UTILS ############################ //
// ##################################################################### //

void cpx_init(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::environment& env, const logger& log, bool log_file = true) {
    PRINT_VERBOSE(log, "Initializing CPLEX.");
    int cpxerror;
    cpxenv = CPXopenCPLEX(&cpxerror);
    CPX_HANDLE_CALL(log, cpxerror);
    cpxlp = CPXcreateprob(cpxenv, &cpxerror, env.run_name.c_str());
    CPX_HANDLE_CALL(log, cpxerror);
    if (env.tmp_choice) CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_Preprocessing_Presolve, CPX_OFF));
    // log file
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_ScreenOutput, HPLUS_DEF_CPX_SCREENOUTPUT));
    if (log_file) CPX_HANDLE_CALL(log, CPXsetlogfilename(cpxenv, (HPLUS_CPLEX_OUTPUT_DIR "/log/" + env.run_name + ".log").c_str(), "w"));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPX_PARAM_CLONELOG, HPLUS_DEF_CPX_CLONELOG));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_MIP_Display, HPLUS_DEF_CPX_MIP_DISPLAY));
    // tolerance
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_MIP_Tolerances_MIPGap, HPLUS_DEF_CPX_TOL_GAP));
    // memory/size limits
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_MIP_Limits_TreeMemory, HPLUS_DEF_CPX_TREE_MEM));
    CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_WorkMem, HPLUS_DEF_CPX_WORK_MEM));
    CPX_HANDLE_CALL(log, CPXsetintparam(cpxenv, CPXPARAM_MIP_Strategy_File, HPLUS_DEF_CPX_STRAT_FILE));
    // terminate condition
    CPX_HANDLE_CALL(log, CPXsetterminate(cpxenv, &global_terminate));
}

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const logger& log) {
    CPX_HANDLE_CALL(log, CPXfreeprob(cpxenv, &cpxlp));
    CPX_HANDLE_CALL(log, CPXcloseCPLEX(&cpxenv));
}

[[nodiscard]]
bool parse_cpx_status(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::environment& env, const logger& log) {
    PRINT_VERBOSE(log, "Parsing CPLEX status code.");
    switch (const int cpxstatus{CPXgetstat(cpxenv, cpxlp)}) {
        case CPXMIP_TIME_LIM_FEAS:  // exceeded time limit, found intermediate solution
            [[fallthrough]];
        case CPXMIP_ABORT_FEAS:  // terminated by user, found solution
            env.sol_s = solution_status::FEAS;
            return true;
        case CPXMIP_MEM_LIM_INFEAS:  // exceeded memory limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_MEM_LIM_FEAS:  // exceeded memory limit, found intermediate solution
            log.raise_error("OUT OF MEMORY");
            return false;
        case CPXMIP_TIME_LIM_INFEAS:  // exceeded time limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_ABORT_INFEAS:  // terminated by user, not found solution
            if (!env.warm_start) env.sol_s = solution_status::NOTFOUND;
            return false;
        case CPXMIP_INFEASIBLE:  // proven to be infeasible
            env.sol_s = solution_status::INFEAS;
            return false;
        case CPXMIP_OPTIMAL_TOL:  // found optimal within the tollerance
            PRINT_WARN(log, "Found optimal within the tolerance.");
            [[fallthrough]];
        case CPXMIP_OPTIMAL:  // found optimal
            env.sol_s = solution_status::OPT;
            return true;
        default:  // unhandled status
            log.raise_error("Error in parse_cpx_status: unhandled cplex status: %d.", cpxstatus);
            return false;
    }
}

// ##################################################################### //
// ############################### LM-CUT ############################## //
// ##################################################################### //

// void compute_hmax(const hplus::instance& inst, std::vector<double>& values, const std::vector<int>& reduced_costs) {
//     std::vector<size_t> initial_actions;
//     for (const auto& act_i : inst.act_rem) {
//         if (inst.actions[act_i].pre_sparse.empty()) initial_actions.push_back(act_i);
//     }
//     priority_queue<double> pq{inst.n};
//     values = std::vector<double>(inst.n, std::numeric_limits<double>::infinity());

//     for (const auto& act_i : initial_actions) {
//         const double cost{static_cast<double>(reduced_costs[act_i])};
//         for (const auto& p : inst.actions[act_i].eff_sparse) {
//             if (cost >= values[p]) continue;

//             values[p] = cost;
//             if (pq.has(p))
//                 pq.change(p, cost);
//             else
//                 pq.push(p, cost);
//         }
//     }

//     while (!pq.empty()) {
//         const size_t p{pq.top()};
//         pq.pop();
//         for (const auto& act_i : inst.act_with_pre[p]) {
//             double cost_pre{0};
//             for (const auto& p : inst.actions[act_i].pre_sparse) cost_pre = std::max(cost_pre, values[p]);

//             if (cost_pre >= std::numeric_limits<double>::infinity()) continue;

//             const double new_cost{cost_pre + reduced_costs[act_i]};
//             for (const auto& p_eff : inst.actions[act_i].eff_sparse) {
//                 if (new_cost >= values[p_eff]) continue;

//                 values[p_eff] = new_cost;
//                 if (pq.has(p_eff))
//                     pq.change(p_eff, new_cost);
//                 else
//                     pq.push(p_eff, new_cost);
//             }
//         }
//     }
// }

// void compute_pcf(const hplus::instance& inst, const std::vector<double>& values, std::vector<int>& pcf) {
//     pcf = std::vector<int>(inst.m, -1);
//     for (const auto& act_i : inst.act_rem) {
//         double max{-1};
//         for (const auto& p : inst.actions[act_i].pre_sparse) {
//             if (values[p] > max) {
//                 max = values[p];
//                 pcf[act_i] = p;
//             }
//         }
//     }
// }

// unsigned int compute_cut(const hplus::instance& inst, std::vector<int>& reduced_costs, const std::vector<int>& pcf, std::vector<size_t>& cut) {
//     binary_set A(inst.n), B(inst.n);
//     cut.clear();
//     std::deque<size_t> queue;
//     for (const auto& p : inst.goal) {
//         queue.push_back(p);
//         B.add(p);
//     }
//     while (!queue.empty()) {
//         const size_t p{queue.front()};
//         queue.pop_front();
//         for (const auto& act_i : inst.act_with_eff[p]) {
//             if (reduced_costs[act_i] != 0 || pcf[act_i] == -1) continue;
//             if (!B[pcf[act_i]]) {
//                 B.add(pcf[act_i]);
//                 queue.push_back(pcf[act_i]);
//             }
//         }
//     }

//     binary_set incut(inst.m);
//     for (const auto& act_i : inst.act_rem) {
//         if (!inst.actions[act_i].pre_sparse.empty()) continue;
//         if (B.intersects(inst.actions[act_i].eff)) {
//             if (reduced_costs[act_i] == 0) continue;
//             cut.push_back(act_i);
//             incut.add(act_i);
//         } else {
//             for (const auto q : inst.actions[act_i].eff_sparse) {
//                 if (A[q]) continue;
//                 A.add(q);
//                 queue.push_back(q);
//             }
//         }
//     }

//     while (!queue.empty()) {
//         const size_t p{queue.front()};
//         queue.pop_front();
//         for (const auto& act_i : inst.act_with_pre[p]) {
//             if (pcf[act_i] != static_cast<int>(p)) continue;
//             if (B.intersects(inst.actions[act_i].eff)) {
//                 if (incut[act_i]) continue;
//                 ASSERT(reduced_costs[act_i] > 0);
//                 incut.add(act_i);
//                 cut.push_back(act_i);
//             } else {
//                 for (const auto q : inst.actions[act_i].eff_sparse) {
//                     if (A[q]) continue;
//                     A.add(q);
//                     queue.push_back(q);
//                 }
//             }
//         }
//     }

//     const int choice_cost{[&]() {
//         ASSERT(!cut.empty());
//         int min = reduced_costs[cut[0]];
//         for (const auto act_i : cut) min = std::min(min, reduced_costs[act_i]);
//         return min;
//     }()};

//     ASSERT(choice_cost > 0);

//     for (const size_t act_i : cut) reduced_costs[act_i] -= choice_cost;
//     return choice_cost;
// }

// void compute_lmcut(hplus::instance& inst, const logger& log) {
//     log.print_info("Computing LM-CUT.");
//     std::vector<double> values(inst.n, std::numeric_limits<double>::infinity());
//     std::vector<int> reduced_costs(inst.m);
//     for (const auto& act_i : inst.act_rem) {
//         reduced_costs[act_i] = inst.actions[act_i].cost;
//     }
//     compute_hmax(inst, values, reduced_costs);
//     std::vector<int> pcf(inst.m, -1);
//     compute_pcf(inst, values, pcf);
//     double goal_hmax{0};
//     for (const auto& p : inst.goal) goal_hmax = std::max(goal_hmax, values[p]);
//     unsigned int lmcut{0};
//     std::vector<size_t> cut;
//     while (goal_hmax != 0) {
//         lmcut += compute_cut(inst, reduced_costs, pcf, cut);
//         inst.landmarks.push_back(cut);
//         compute_hmax(inst, values, reduced_costs);
//         compute_pcf(inst, values, pcf);
//         goal_hmax = 0;
//         for (const auto& p : inst.goal) goal_hmax = std::max(goal_hmax, values[p]);
//         // log.print_info("goal_hmax: %d", static_cast<int>(goal_hmax));
//     }

//     log.print_info("lmcut: %d", lmcut);
//     // exit(0);
// }

// ##################################################################### //
// ######################### ALGORITHM HANDLERS ######################## //
// ##################################################################### //

void run_heur(hplus::instance& inst, hplus::environment& env, const logger& log) {
    srand(time(nullptr));
    if (env.heur == HPLUS_CLI_HEUR_GREEDYCOST)
        greedycost::run(inst, env, log);
    else if (env.heur == HPLUS_CLI_HEUR_GREEDYCXE)
        greedycxe::run(inst, env, log);
    else if (env.heur == HPLUS_CLI_HEUR_GREEDYHMAX)
        hmax::run(inst, env, log);
    else if (env.heur == HPLUS_CLI_HEUR_GREEDYHADD)
        hadd::run(inst, env, log);
    else
        log.raise_error("The heuristic specified (%s) is not on the list of possible heuristics... Please use the --h flag for instructions.",
                        env.heur.c_str());
}

void run_model(hplus::instance& inst, hplus::environment& env, hplus::statistics& stats, const logger& log) {
    auto stopchk = [&env]() {
        if (CHECK_STOP()) {
            env.exec_s = exec_status::STOP_TL;
            throw timelimit_exception("Reached time limit.");
        }
    };

    // ~~~~~~~~~~~~ MODEL BUILDING ~~~~~~~~~~~ //
    env.exec_s = exec_status::MODEL_BUILD;

    stats.build = static_cast<double>(env.time_limit) - env.timer.get_time();
    double start_time = env.timer.get_time();

    CPXENVptr cpxenv = nullptr;
    CPXLPptr cpxlp = nullptr;

    cpx_init(cpxenv, cpxlp, env, log);
    stopchk();

    if (env.alg == HPLUS_CLI_ALG_TL)
        tl::build_cpx_model(cpxenv, cpxlp, inst, env, log, stats);
    else if (env.alg == HPLUS_CLI_ALG_VE)
        ve::build_cpx_model(cpxenv, cpxlp, inst, env, log, stats);
    else if (env.alg == HPLUS_CLI_ALG_LM)
        lm::build_cpx_model(cpxenv, cpxlp, inst, env, log, stats);
    stopchk();

    // time limit
    if (static_cast<double>(env.time_limit) > env.timer.get_time()) {
        CPX_HANDLE_CALL(log, CPXsetdblparam(cpxenv, CPXPARAM_TimeLimit, static_cast<double>(env.time_limit) - env.timer.get_time()));
    } else
        throw timelimit_exception("Reached the time limit");

    if (env.warm_start) {  // Post warm starto to CPLEX

        if (env.alg == HPLUS_CLI_ALG_TL)
            tl::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_VE)
            ve::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
        else if (env.alg == HPLUS_CLI_ALG_LM)
            lm::post_cpx_warmstart(cpxenv, cpxlp, inst, env, log);
    }

    // compute_lmcut(inst, log);

    stats.nusercuts = 0;
    lm::cpx_callback_user_handle callback_data{
        .inst = inst, .env = env, .stats = stats, .log = log, .data = {nullptr, nullptr, std::map<int, std::pair<CPXENVptr, CPXLPptr>>()}};
    if (env.alg == HPLUS_CLI_ALG_LM) {
        CPXLONG callback_context = CPX_CALLBACKCONTEXT_CANDIDATE;
        if (env.fract_cuts) {
            lm::cpx_create_lmcut_model(inst, log, callback_data.data.lmcutenv, callback_data.data.lmcutlp);
            callback_context |= CPX_CALLBACKCONTEXT_RELAXATION;
        }
        CPX_HANDLE_CALL(log, CPXcallbacksetfunc(cpxenv, cpxlp, callback_context, lm::cpx_callback_hub, &callback_data));
    }

    stats.build = env.timer.get_time() - start_time;

    // ~~~~~~~~~~~ MODEL EXECUTION ~~~~~~~~~~~ //

    PRINT_INFO(log, "Running CPLEX.");
    env.exec_s = exec_status::CPX_EXEC;

    stats.execution = static_cast<double>(env.time_limit) - env.timer.get_time();
    start_time = env.timer.get_time();

    CPX_HANDLE_CALL(log, CPXmipopt(cpxenv, cpxlp));

    stats.nnodes = CPXgetnodecnt(cpxenv, cpxlp);

    if (parse_cpx_status(cpxenv, cpxlp, env, log)) {  // If CPLEX has found a solution
        CPX_HANDLE_CALL(log, CPXgetbestobjval(cpxenv, cpxlp, &stats.lb));
        if (env.alg == HPLUS_CLI_ALG_TL)
            tl::store_cpx_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_VE)
            ve::store_cpx_sol(cpxenv, cpxlp, inst, log);
        else if (env.alg == HPLUS_CLI_ALG_LM)
            lm::store_cpx_sol(cpxenv, cpxlp, inst, log);
    }

    cpx_close(cpxenv, cpxlp, log);
    if (env.fract_cuts && env.alg == HPLUS_CLI_ALG_LM) {
        lm::cpx_close_lmcut_model(callback_data.data.lmcutenv, callback_data.data.lmcutlp, log);
        for (auto& data : callback_data.data.thread_data) lm::cpx_close_lmcut_model(data.second.first, data.second.second, log);
    }

    stats.execution = env.timer.get_time() - start_time;
}
