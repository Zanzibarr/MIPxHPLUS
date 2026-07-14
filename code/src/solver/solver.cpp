#include "solver.hpp"

#include <algorithm>
#include <cassert>
#include <exception>
#include <limits>
#include <logger.hxx>
#include <timer.hxx>

#include "cli_descriptions.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;

void Solver::solve() {
    // Reference point for elapsed/remaining time; matches when main() arms the global timer thread.
    global_.start_time = std::chrono::steady_clock::now();
    try {
        // Read instance
        {
            auto _read_timer = scoped_timer("read-instance");
            logger_[INFO] << "Reading input file";
            read_instance_();
        }

        // The total timer should not consider CLI and file input times
        auto _total_timer = scoped_timer("total");

        // Preprocessing
        {
            auto _prep_timer = scoped_timer("preprocess");
            if (params_.get<cli_desc::preprocess, std::string>() != "0") {
                logger_[INFO] << "Preprocessing instance";
                preprocess_();
            }
            prep_setup_helpers_();
        }

        print_info_();

        // LM-Cut
        if (params_.get<cli_desc::lmcut_pcf, std::string>() != "0") {
            auto _lmcut_timer = scoped_timer("lmcut");
            logger_[INFO] << "Executing LM-Cut";
            solve_lmcut_();
        }

        // Primal Heuristic
        if (params_.get<cli_desc::primal_heur, std::string>() != "0") {
            auto _heur_timer = scoped_timer("heur");
            logger_[INFO] << "Computing Primal Heuristic";
            solve_primal_();
        }

        // h+
        if (params_.get<cli_desc::mode, std::string>() == "hplus") {
            logger_[INFO] << "Computing HPLUS";
            solve_hplus_();
        }

    } catch (const EarlyExit& e) {
        switch (e.reason()) {
            case EarlyExit::TIMELIMIT:
                logger_[WARNING] << std::format("Reached time limit while {}.", e.what());
                // Do nothing... we continue our execution as normal
                break;

            case EarlyExit::INFEASIBLE:
                logger_[WARNING] << std::format("Proved infeasibility for this instance while {}.", e.what());

                global_.best_bound = constants::infeas_bound;
                global_.best_incumbent = constants::infeas_bound;
                global_.solution.clear();

                break;

            case EarlyExit::OPTIMAL:
                logger_[WARNING] << std::format("Proved optimality for this instance while {}", e.what());

                myassert(is_same_double(global_.best_bound, global_.best_incumbent), "Optimality proven but bounds are not matching.");
                myassert(!global_.solution.empty() || inst_.goal.empty(), "Optimality proven but no solution is provided.");

                // solve_hplus_ _SHOULD_ not be throwing an OPTIMAL EarlyExit, so we're here only when we exit early due to the primal heuristic
                // matching lmcut's lower bound: in this case we have no way of knowing what would have been the LP/root relaxation, so we write
                // it here (note that since we already proved optimality, there couldn't be an higher LP/root relaxation, so this is correct)
                myassert(!global_.using_cplex, "OPTIMAL EarlyExit thrown while using CPLEX");
                stats_.gauge_record<"lb_relaxation">(global_.best_bound);
                stats_.gauge_record<"lb_rootnode">(global_.best_bound);
                stats_.counter_set<"nodes">(0);

                break;

            default:
                logger_[ERROR] << std::format("Unhandled early exit: {}", e.what());
                break;
        }
    } catch (const std::bad_alloc& e) {
        logger_[WARNING] << "Reached memory limit";
    } catch (const std::exception& e) {
        logger_[FATAL] << std::format("Caught exception in Solver::solve(): {}", e.what());
    } catch (...) {
        logger_[FATAL] << std::format("Something wrong happened in Solver::solve().");
    }
}

void Solver::show() {
    logger_ << std::format("Best bound: {}", global_.best_bound);
    logger_ << std::format("Best incumbent: {}", global_.best_incumbent);
    // logger_ << "Best solution:";
    // for (auto a : global_.solution) {
    //     logger_ << inst_.actions_names[a];
    // }
    logger_ << stats_.stats_report_to_str();
    logger_ << stats_.counter_report_to_str();
    logger_ << stats_.gauge_report_to_str();
}

auto Solver::get() -> std::vector<std::string> {
    std::vector<std::string> solution_names;
    solution_names.reserve(global_.solution.size());
    for (auto act_i : global_.solution) {
        solution_names.emplace_back(inst_.actions_names[act_i]);
    }
    return solution_names;
}

void Solver::print_info_() {
    logger_ << "----------------- Info on the instance -----------------";
    if (!inst_.actions.empty()) {
        const std::string cost_type = inst_.actions[0].cost == 1 ? "unitary costs" : "constant costs";
        logger_ << "Metric:                             " << std::setw(20) << (inst_.equal_costs ? cost_type : "integer costs");
    }
    logger_ << "# facts:                                      " << std::setw(10) << inst_.n;
    logger_ << "# actions:                                    " << std::setw(10) << inst_.m;
    logger_ << "# first adders:                               " << std::setw(10) << inst_.nfadd;
    logger_ << "--------------------------------------------------------";
}

void Solver::try_update_best_bound_(double new_bound) {
    integritycheck(is_lw_or_eq_double(new_bound, global_.best_incumbent), "Proposed lower bound is higher than best incumbent");

    // Early check, before locking mutex
    if (is_gr_or_eq_double(global_.best_bound, new_bound)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(global_.solution_mutex);
        if (is_gr_or_eq_double(global_.best_bound, new_bound)) {
            return;
        }
        global_.best_bound = new_bound;
        logger_[DEBUG] << std::format("Updated best bound: {}", global_.best_bound);
    }
}

void Solver::try_update_best_incumbent_(const std::vector<unsigned int>& new_solution, double new_incumbent) {
    integritycheck(is_gr_or_eq_double(new_incumbent, global_.best_bound), "Proposed incumbent is lower than best bound");

    std::unordered_set<unsigned int> dbcheck;
    double costcheck{0};
    integritycheck(new_solution.size() <= inst_.m, "Invalid number of actions in candidate solution");
    BinarySet state{inst_.n};
    for (const auto& act_i : new_solution) {
        integritycheck(act_i < inst_.m, "Invalid acion in candidate solution");
        integritycheck(!dbcheck.contains(act_i), "Action used multiple times in candidate solution");
        dbcheck.insert(act_i);
        integritycheck(bs_contains(state, inst_.actions[act_i].pre_sparse), "Preconditions not respected in candidate solution");
        state |= inst_.actions[act_i].eff_sparse;
        costcheck += inst_.actions[act_i].cost;
    }
    integritycheck(state.superset_of(inst_.goal), "Solution doesn't lead to the goal state");
    integritycheck(is_same_double(costcheck, new_incumbent),
                   std::format("Proposed cost doesn't match computed cost {}-{}", costcheck, new_incumbent));

    // Early check, before locking mutex
    if (is_gr_or_eq_double(new_incumbent, global_.best_incumbent)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(global_.solution_mutex);
        if (is_gr_or_eq_double(new_incumbent, global_.best_incumbent)) {
            return;
        }
        global_.solution = new_solution;
        global_.best_incumbent = fix_precision(new_incumbent);  // We know all costs are actually integer, so we have to fix precision errors here
        logger_[INFO] << std::format("Updated best solution: {}", global_.best_incumbent);
    }
}
