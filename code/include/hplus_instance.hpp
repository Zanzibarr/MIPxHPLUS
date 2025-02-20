/**
 * @file hplus_instance.hpp
 * @brief Interface for the hplus instance to be used in the hplus master thesis project
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef INST_H
#define INST_H

#include "utils.hpp"
#include "bs.hxx"
#include "log.hxx"
#include <thread>

namespace hplus {
    /** Struct containing all necessary info for an action */
    typedef struct {
        binary_set pre, eff;
        std::vector<size_t> pre_sparse, eff_sparse; 
        unsigned int cost;
        std::string name;
    } action;
    /** Struct containing all necessary info for solving the delete free relaxation of the planning task */
    typedef struct {
        /** Domain */
        bool unary_costs;
        size_t n, m, n_opt, m_opt;
        std::vector<size_t> var_ranges;
        std::vector<action> actions;
        /** Instance */
        binary_set goal;
        std::vector<size_t> best_solution;
        unsigned int best_cost;
        /** Optimization variables */
        binary_set var_e, var_f, act_e, act_f;
        std::vector<binary_set> fadd_e, fadd_f;
        std::vector<int> var_t, act_t;
        std::vector<std::vector<size_t>> act_inv;
        /** Optimization helpers */
        std::vector<size_t> var_opt_conv, act_opt_conv, fadd_checkpoint, act_cpxtoidx;
        std::vector<std::vector<size_t>> act_with_eff, act_with_pre;
    } instance;
    /** Struct containing the environment variables for the execution of the code */
    typedef struct {
        /** Execution/Solution status */
        exec_status exec_s;
        solution_status sol_s;
        /** Parameters for execution */
        std::string input_file, log_name, run_name, alg, heur;
        bool log;
        /** Execution flags */
        bool problem_opt, warm_start, imai_tight_bounds, inv_act, using_cplex;
        /** Time limit */
        unsigned int time_limit;
        time_keeper timer;
    } environment;
    /** Time statistics */
    typedef struct {
        double parsing, optimization, heuristic, build, callback, execution, total;
        pthread_mutex_t callback_time_mutex;
    } statistics;

    // ##################################################################### //
    // ################### READ/WRITE OF HPLUS STRUCTURES ################## //
    // ##################################################################### //
    
    /** Print the time statistics @param _s */
    void print_stats(const statistics& _s, const logger& _l);
    /** Create the instance @param _i from the file found at the path @param _f */
    void create_instance(instance& _i, const environment& _e, statistics& _s, const logger& _l);
    /** Get the remaining variables (the ones not eliminated by an eventual optimization of the instance) of the instance @param _i */
    binary_set var_remaining(const instance& _i);
    /** Get the remaining actions (the ones not eliminated by an eventual optimization of the instance) of the instance @param _i */
    binary_set act_remaining(const instance& _i);
    /** Update the best solution of @param _i with a new solution @param _s (with cost @param _c): if the solution is not better, the solution won't be updated */
    void update_sol(instance& _i, const std::vector<size_t> _s, const unsigned int _c, const logger& _l);
    /** Print the best solution of instance @param _i */
    void print_sol(const instance& _i, const logger& _l);
    /** Perform instance optimization to @param _i using flags stored in the @param _e environment */
    void instance_optimization(instance& _i, const environment& _e, const logger& _l);
    /** Prepare helper data structure for faster action lookup on instance @param _i */
    void prepare_faster_actsearch(instance& _i);

}

#endif /* INST_H */