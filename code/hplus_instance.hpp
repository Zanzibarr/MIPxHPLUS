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

#if HPLUS_INTCHECK == 0
#define INTCHECK_BS false
#endif

#include "bs.hxx"  // For binary_set

namespace hplus {

typedef struct {
    std::vector<size_t> plan;
    unsigned int cost;
} solution;

/** Struct containing all necessary info for an action */
typedef struct {
    binary_set pre, eff;
    std::vector<size_t> pre_sparse, eff_sparse;
    unsigned int cost;
    std::string name;
} action;

/** Struct containing all necessary info for solving the delete free relaxation
 * of the planning task */
typedef struct {
    /** Domain */
    bool equal_costs;
    size_t n, m, n_opt, m_opt;
    std::vector<int> var_ranges;
    std::vector<action> actions;
    /** Instance */
    binary_set goal;
    solution best_sol;
    /** Optimization variables */
    binary_set var_e, var_f, act_e, act_f;
    std::vector<binary_set> fadd_e, fadd_f;
    /** Optimization helpers */
    std::vector<size_t> var_rem, act_rem;
    std::vector<size_t> var_opt_conv, act_opt_conv, act_cpxtoidx;
    size_t n_fadd;
    std::vector<size_t> fadd_cpx_start;
    std::vector<size_t> veg_starts;
    std::vector<binary_set> veg_cumulative_graph;
    std::vector<std::vector<size_t>> act_with_eff, act_with_pre;
} instance;

/** Struct containing the environment variables for the execution of the code */
typedef struct {
    /** Execution/Solution status */
    exec_status exec_s;
    solution_status sol_s;
    /** Parameters for execution */
    int threads;
    std::string input_file, log_name, run_name, alg, heur;
    /** Execution flags */
    bool preprocessing, warm_start, tight_bounds, using_cplex, log, write_lp;
    /** Landmarks model */
    bool minimal_landmark, complete_landmark, sec, fract_cuts, fract_sec;
    /** Time limit */
    unsigned int time_limit;
    time_keeper timer;
    bool tmp_choice;
} environment;

/** Statistics */
typedef struct {
    // Times
    double parsing, preprocessing, heuristic, build, cand_callback, relax_callback, execution, total;
    // Costs
    int hcost, fcost;
    // Cplex stats
    int nnodes, status, nvar_base, nvar_acyclic, nconst_base, nconst_acyclic, usercuts_lm, usercuts_sec;
    double lb;
} statistics;

// ##################################################################### //
// ################### READ/WRITE OF HPLUS STRUCTURES ################## //
// ##################################################################### //

/** Initialize an environment */
void init(environment& env);
/** Initialize the statistics */
void init(statistics& stats);
/** Initialize an instance */
void init(instance& inst);
/** Print the time statistics stats */
void print_stats(const statistics& stats, const logger& log);
/** Create the instance inst from the file stored in the environment @returns
 * false if it detects that the problem is infeasible (no assumptions can be
 * made if it returns true) */
[[nodiscard]]
bool create_instance(instance& inst, environment& env, statistics& stats, const logger& log, const bool silent = false);
/** Update the best solution of inst with a new solution sol (with cost cost):
 * if the solution is not better, the solution won't be updated */
void update_sol(instance& inst, const solution& sol, const logger& log);
/** Print the best solution of instance inst */
void print_sol(const instance& inst, const logger& log);
/** Perform instance optimization to inst using flags stored in the env
 * environment */
void preprocessing(instance& inst, const logger& log);
/** Prepare helper data structure for faster action lookup on instance inst */
void prepare_faster_actsearch(instance& inst, const logger& log);

}  // namespace hplus

#endif /* INST_H */