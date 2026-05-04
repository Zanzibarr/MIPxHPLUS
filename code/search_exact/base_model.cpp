#include <algorithm>

#include "algorithms.hpp"
#include "bs_utils.hpp"
#include "exact.hpp"
#include "hplus_algs.hpp"
#include "utils.hpp"

namespace {
void parse_cplex_status(const CPXENVptr& env, const CPXLPptr& lp, const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO_S("Parsing CPLEX status");
    std::vector<double> tmp(1);
    switch (CPXgetx(env, lp, tmp.data(), 0, 0)) {
        case CPXERR_NO_SOLN:  // No solution found
            if (exec.ws == hplus::warmstart::NONE) {
                inst.sol_s = hplus::solution_status::NOTFOUND;
            }
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
            [[fallthrough]];
        case CPXMIP_FEASIBLE:  // found a feasible solution
            inst.sol_s = hplus::solution_status::FEAS;
            break;
        case CPXMIP_MEM_LIM_INFEAS:  // exceeded memory limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_TIME_LIM_INFEAS:  // exceeded time limit, no intermediate solution found
            [[fallthrough]];
        case CPXMIP_ABORT_INFEAS:  // terminated by user, not found solution
            if (exec.ws == hplus::warmstart::NONE) {
                inst.sol_s = hplus::solution_status::NOTFOUND;
            }
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
            LOG_ERROR_S("Error in parse_cpx_status: unhandled cplex status (" + std::to_string(status) + ")");
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
            LOG_ERROR_S("Unhandled solution status: " + std::to_string(static_cast<int>(inst.sol_s)));
    }
}

void store_cplex_solution(hplus::instance& inst, hplus::statistics& stats, const CPXENVptr& env, const CPXLPptr& lp) {
    std::vector<double> plan(inst.m + inst.nfadd, 0.0);
    switch (int code = CPXgetx(env, lp, plan.data(), 0, static_cast<int>(inst.m + inst.nfadd - 1))) {
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
            LOG_ERROR_S("Unhandled CPLEX error code: " + std::to_string(code) + " at " + __func__ + "(): " + __FILE__ + ":" +
                        std::to_string(__LINE__));
            break;
    }

    // fixing the solution to read the plan (some 0-cost actions are set to 1 even if they are not a first archiever of anything)
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        bool set_zero{true};
        for (unsigned int var_count = 0; var_count < inst.actions[act_i].eff_sparse.size(); var_count++) {
            if (plan[inst.m + inst.fadd_cpx_start[act_i] + var_count] > HPLUS_CPX_INT_ROUNDING) {
                ASSERT(plan[act_i] > HPLUS_CPX_INT_ROUNDING);
                set_zero = false;
                break;
            }
        }
        if (set_zero) {
            plan[act_i] = 0;
        }
    }

    // If the solution we have is the same/better than the one returned by cplex, we can skip the rest of this function
    double obj{-1};
    CPX_HANDLE_CALL(CPXgetobjval(env, lp, &obj));
    if (obj >= inst.sol.cost - HPLUS_EPSILON) {
        return;
    }

    // convert to a vector of int for easier parsing
    std::vector<unsigned int> cpx_result;
    cpx_result.reserve(inst.m);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        if (plan[act_i] > HPLUS_CPX_INT_ROUNDING) {
            cpx_result.push_back(act_i);
        }
    }

    std::vector<unsigned int> solution;
    solution.reserve(inst.m);
    BinarySet remaining{static_cast<unsigned int>(cpx_result.size()), true};
    BinarySet state{inst.n};
    unsigned int cost{0};

    // Check we are getting ALL the actions that cplex uses
    while (!remaining.empty()) {
        bool intcheck{false};
        for (const auto idx : remaining) {
            if (!bs_contains(state, inst.actions[cpx_result[idx]].pre_sparse)) {
                continue;
            }

            remaining.remove(idx);
            state |= inst.actions[cpx_result[idx]].eff_sparse;
            solution.push_back(cpx_result[idx]);
            intcheck = true;
            cost += inst.actions[cpx_result[idx]].cost;
        }
        // TODO: CPLEX solution not serializable — see issue #3
        ASSERT(intcheck);
    }

    // store solution
    hplus::solution sol{.sequence = solution, .cost = cost};
    hplus::update_sol(inst, sol, stats);
}

}  // namespace

void exact::build_base_model(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    LOG_INFO_S("Building base model for exact search");

    auto stopcheck = []() {
        if (CHECK_STOP()) {
            throw timelimit_exception("Reached time limit.");
        }
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

    CPX_HANDLE_CALL(CPXnewcols(env, lp, static_cast<int>(count), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
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
        CPX_HANDLE_CALL(
            CPXnewcols(env, lp, static_cast<int>(inst.actions[act_i].eff_sparse.size()), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
        stopcheck();
    }

    // ------- variables ------ //
    const unsigned int var_start{curr_col};
    count = 0;
    for (unsigned int var_i = 0; var_i < inst.n; var_i++) {
        lbs[count++] = (inst.fixed_facts[var_i] || inst.goal[var_i]) ? 1 : 0;
    }
    curr_col += count;

    CPX_HANDLE_CALL(CPXnewcols(env, lp, static_cast<int>(count), objs.data(), lbs.data(), ubs.data(), types.data(), nullptr));
    stopcheck();

    STATS.counter_set<"n_var_base">(inst.n + inst.m + inst.nfadd);

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
    unsigned int nnz{0};
    constexpr char sense_e{'E'};
    constexpr char sense_l{'L'};
    constexpr char sense_g{'G'};
    constexpr double rhs_0{0};
    constexpr double rhs_1{1};
    constexpr int begin{0};

    for (unsigned int var_i = 0; var_i < inst.n; var_i++) {
        nnz = 0;
        ind[nnz] = get_var_idx(var_i);
        val[(nnz++)] = 1;

        for (const auto& act_i : inst.act_with_eff[var_i]) {
            unsigned int var_count = sorted_find(inst.actions[act_i].eff_sparse, var_i);
            ind[nnz] = get_fa_idx(act_i, var_count);
            val[(nnz++)] = -1;
        }

        // if nnz == 1, then we'd have p = 0, meaning we could simply fix this variable to 0
        if (nnz == 1) {
            const char fix = 'B';
            CPX_HANDLE_CALL(CPXchgbds(env, lp, 1, ind.data(), &fix, &rhs_0));
        } else {
            STATS.counter_inc<"n_const_base">();
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, static_cast<int>(nnz), &rhs_0, &sense_e, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
        stopcheck();
    }

    for (unsigned int fact_p = 0; fact_p < inst.n; fact_p++) {
        for (unsigned int fact_q = 0; fact_q < inst.n; fact_q++) {
            nnz = 0;
            ind[nnz] = get_var_idx(fact_q);
            val[(nnz++)] = -1;
            for (const auto& act_i : inst.act_with_eff[fact_p]) {
                if (!sorted_contains(inst.actions[act_i].pre_sparse, fact_q)) {
                    continue;
                }
                unsigned int var_count = sorted_find(inst.actions[act_i].eff_sparse, fact_p);
                ind[nnz] = get_fa_idx(act_i, var_count);
                val[(nnz++)] = 1;
            }
            // if nnz == 1 than we have -fact_p <= 0, hence it's always true, we can ignore this constraint
            if (nnz != 1) {
                STATS.counter_inc<"n_const_base">();
                CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, static_cast<int>(nnz), &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
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
            STATS.counter_inc<"n_const_base">();
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
        stopcheck();
    }

    ind.resize(inst.m);
    val.resize(inst.m);

    // If preprocessing is used, there might be Disjunctive Action Landmarks to add as constraints... those must be counted as acyclicity constraints,
    // since those are not needed for the base model correctness
    if (exec.prep) {
        for (const auto& landmark : inst.landmarks) {
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
                val[(nnz++)] = 1;
            }
            STATS.counter_inc<"n_const_acyc">();
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, static_cast<int>(nnz), &rhs_1, &sense_g, &begin, ind.data(), val.data(), nullptr, nullptr));
        }
        stopcheck();
    }
}

void exact::get_cplex_solution(const hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, const CPXENVptr& env,
                               const CPXLPptr& lp) {
    parse_cplex_status(env, lp, exec, inst, stats);

    STATS.counter_set<"nodes">(CPXgetnodecnt(env, lp));

    if (inst.sol_s > hplus::solution_status::FEAS) {
        return;
    }

    // TODO: handle CPXERR_NOT_MIP (3003) — see issue #1
    CPX_HANDLE_CALL(CPXgetbestobjval(env, lp, &stats.lower_bound));
    stats.lower_bound = std::max<double>(stats.lower_bound, 0);

    // This is needed due to the patch applied to the beginning of the candidate/relaxation callback: if the lower and upper bounds match, we have the
    // optimal. This is needed because CPLEX might return without having evaluated our posted optimal solution, so the status he has is FEAS, not OPT
    // (even though the lower bound he gave use matches our best solution).
    // Moreover, this is a nice check to be made, to prevent possible missing status updates...
    if (stats.lower_bound >= stats.cost - HPLUS_EPSILON) {
        stats.lower_bound = static_cast<double>(stats.cost);  // Fix possible precision errors
        stats.status = HPLUS_STATUS_OPT;
        inst.sol_s = hplus::solution_status::OPT;
    }

    // If we never exited the root node we need to store the lower bound for statistics
    // TODO: I don't like this way of getting this statistic... too unreliable (and possibly wrong)... find another way (continuation of same TODO in
    // relax_callback.cpp)
    if (STATS.counter_get<"nodes">() == 0) {
        STATS.gauge_record<"lb_rootnode">(stats.lower_bound);
    }

    LOG_INFO_S("Reading CPLEX solution");
    store_cplex_solution(inst, stats, env, lp);
}