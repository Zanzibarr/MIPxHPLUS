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

#include "bs.hxx"

namespace hplus {

	typedef struct {
		std::vector<size_t> plan;
		unsigned int		cost;
	} solution;

	/** Struct containing all necessary info for an action */
	typedef struct {
		binary_set			pre, eff;
		std::vector<size_t> pre_sparse, eff_sparse;
		unsigned int		cost;
		std::string			name;
	} action;

	/** Struct containing all necessary info for solving the delete free relaxation of the planning task */
	typedef struct {
		/** Domain */
		bool				equal_costs;
		size_t				n, m, n_opt, m_opt;
		std::vector<size_t> var_ranges;
		std::vector<action> actions;
		/** Instance */
		binary_set goal;
		solution   best_sol;
		/** Optimization variables */
		binary_set						 var_e, var_f, act_e, act_f;
		std::vector<binary_set>			 fadd_e, fadd_f;
		std::vector<int>				 var_t, act_t;
		std::vector<std::vector<size_t>> act_inv;
		/** Optimization helpers */
		std::vector<size_t>				 var_opt_conv, act_opt_conv, fadd_checkpoint, act_cpxtoidx;
		std::vector<std::vector<size_t>> act_with_eff, act_with_pre;
	} instance;

	/** Struct containing the environment variables for the execution of the code */
	typedef struct {
		/** Execution/Solution status */
		exec_status		exec_s;
		solution_status sol_s;
		/** Parameters for execution */
		std::string input_file, log_name, run_name, alg, heur;
		bool		log;
		/** Execution flags */
		bool problem_opt, warm_start, imai_tight_bounds, using_cplex;
		/** Time limit */
		unsigned int time_limit;
		time_keeper	 timer;
	} environment;

	/** Time statistics */
	typedef struct {
		double			parsing, optimization, heuristic, build, callback, execution, total;
		pthread_mutex_t callback_time_mutex;
	} statistics;

	// ##################################################################### //
	// ################### READ/WRITE OF HPLUS STRUCTURES ################## //
	// ##################################################################### //

	/** Print the time statistics stats */
	void print_stats(const statistics& stats, const logger& log);
	/** Create the instance inst from the file stored in the environment @returns false if it detects that the problem is infeasible (no assumptions can be made if it
	 * returns true) */
	[[nodiscard]]
	bool create_instance(instance& inst, environment& env, statistics& stats, const logger& log);
	/** Get the remaining variables (the ones not eliminated by an eventual optimization of the instance) of the instance inst */
	[[nodiscard]]
	binary_set var_remaining(const instance& inst);
	/** Get the remaining actions (the ones not eliminated by an eventual optimization of the instance) of the instance inst */
	[[nodiscard]]
	binary_set act_remaining(const instance& inst);
	/** Update the best solution of inst with a new solution sol (with cost cost): if the solution is not better, the solution won't be updated */
	void update_sol(instance& inst, const solution& sol, const logger& log);
	/** Print the best solution of instance inst */
	void print_sol(const instance& inst, const logger& log);
	/** Perform instance optimization to inst using flags stored in the env environment */
	void instance_optimization(instance& inst, const environment& env, const logger& log);
	/** Prepare helper data structure for faster action lookup on instance inst */
	void prepare_faster_actsearch(instance& inst, const logger& log);

} // namespace hplus

#endif /* INST_H */