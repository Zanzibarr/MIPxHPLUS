/**
 * @file algorithms.cpp
 * @brief Algorithms to solve the delete-free relaxation of the planning task
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include "algorithms.hpp"
#include "pq.hxx"	 // For priority_queue
#include <algorithm> // For std::sort, std::transform
#include <numeric>	 // For std::accumulate
#include <queue>	 // For std::priority_queue (// TODO remove when substituted with priority_queue)
#include <random>	 // For random number generators
#include <set>		 // For std::set

// ##################################################################### //
// ############################ CPLEX UTILS ############################ //
// ##################################################################### //

void cpx_init(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::environment& env, const logger& log, bool log_file) {
	_PRINT_VERBOSE(log, "Initializing CPLEX.");
	int cpxerror;
	cpxenv = CPXopenCPLEX(&cpxerror);
	_ASSERT_LOG(log, !cpxerror);
	cpxlp = CPXcreateprob(cpxenv, &cpxerror, env.run_name.c_str());
	_ASSERT_LOG(log, !cpxerror);
	// log file
	_ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_ScreenOutput, CPX_OFF));
	if (log_file)
		_ASSERT_LOG(log, !CPXsetlogfilename(cpxenv, (HPLUS_CPLEX_OUTPUT_DIR "/log/" + env.run_name + ".log").c_str(), "w"));
	_ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPX_PARAM_CLONELOG, -1));
	// tolerance
	_ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_MIP_Tolerances_MIPGap, 0));
	// memory/size limits
	_ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_MIP_Limits_TreeMemory, 12000));
	_ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_WorkMem, 4096));
	_ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_MIP_Strategy_File, 3));
	// terminate condition
	_ASSERT_LOG(log, !CPXsetterminate(cpxenv, &global_terminate));
}

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp) {
	CPXfreeprob(cpxenv, &cpxlp);
	CPXcloseCPLEX(&cpxenv);
}

bool parse_cpx_status(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Parsing CPLEX status code.");
	switch (int cpxstatus = CPXgetstat(cpxenv, cpxlp)) {
		case CPXMIP_TIME_LIM_FEAS: // exceeded time limit, found intermediate solution
		case CPXMIP_ABORT_FEAS:	   // terminated by user, found solution
			env.sol_s = solution_status::FEAS;
			return true;
		case CPXMIP_TIME_LIM_INFEAS: // exceeded time limit, no intermediate solution found
		case CPXMIP_ABORT_INFEAS:	 // terminated by user, not found solution
			if (!env.warm_start)
				env.sol_s = solution_status::NOTFOUND;
			return false;
		case CPXMIP_INFEASIBLE: // proven to be unfeasible
			env.sol_s = solution_status::INFEAS;
			return false;
		case CPXMIP_OPTIMAL_TOL: // found optimal within the tollerance
			_PRINT_WARN(log, "Found optimal within the tolerance.");
		case CPXMIP_OPTIMAL: // found optimal
			env.sol_s = solution_status::OPT;
			return true;
		default: // unhandled status
			log.raise_error("Error in parse_cpx_status: unhandled cplex status: %d.", cpxstatus);
			return false;
	}
}

// ##################################################################### //
// ############################# HEURISTICS ############################ //
// ##################################################################### //

static inline void greedycost(hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running greedycost algorithm.");

	// greedy choice
	auto find_best_act = [&inst](const std::vector<size_t>& candidates) {
		size_t best_act = 0;
		double best_cost = std::numeric_limits<double>::infinity();
		for (auto act_i : candidates) {
			if (inst.actions[act_i].cost == 0)
				return std::pair<bool, size_t>(true, act_i);
			if (inst.actions[act_i].cost >= best_cost)
				continue;

			best_act = act_i;
			best_cost = inst.actions[act_i].cost;
		}
		return std::pair<bool, size_t>(true, best_act);
	};

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	binary_set state(inst.n); // initial state is empty

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (auto act_i : hplus::act_remaining(inst))
		feasible_actions.add(act_i, inst.actions[act_i].pre);

	while (!state.contains(inst.goal)) {
		const auto& candidates = feasible_actions.find_subsets(state);
		if (candidates.empty()) [[unlikely]] {
			env.sol_s = solution_status::INFEAS;
			return;
		}

		const auto& [_, choice] = find_best_act(candidates);

		heur_sol.plan.push_back(choice);
		heur_sol.cost += inst.actions[choice].cost;
		state |= inst.actions[choice].eff;
		feasible_actions.remove(choice, inst.actions[choice].pre);

		if (_CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static inline void greedycxe(hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running greedycxe algorithm.");

	// greedy choice
	auto find_best_act = [&inst](const std::vector<size_t>& candidates, const binary_set& state) {
		size_t best_act = 0;
		bool   found = false;
		double best_cxe = std::numeric_limits<double>::infinity();
		for (auto act_i : candidates) {
			if (inst.actions[act_i].cost == 0)
				return std::pair<bool, size_t>(true, act_i);
			int neff = 0;
			for (auto var_i : inst.actions[act_i].eff_sparse) {
				if (!state[var_i])
					neff++;
			}
			if (neff == 0)
				continue;
			double cxe = static_cast<double>(inst.actions[act_i].cost) / neff;
			if (cxe >= best_cxe)
				continue;

			found = true;
			best_act = act_i;
			best_cxe = inst.actions[act_i].cost;
		}
		return std::pair<bool, size_t>(found, best_act);
	};

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	binary_set state(inst.n); // initial state is empty

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (auto act_i : hplus::act_remaining(inst))
		feasible_actions.add(act_i, inst.actions[act_i].pre);

	while (!state.contains(inst.goal)) {
		const auto& candidates = feasible_actions.find_subsets(state);
		if (candidates.empty()) [[unlikely]] {
			env.sol_s = solution_status::INFEAS;
			return;
		}

		const auto& [found, choice] = find_best_act(candidates, state);
		if (!found) [[unlikely]] {
			env.sol_s = solution_status::INFEAS;
			return;
		}

		heur_sol.plan.push_back(choice);
		heur_sol.cost += inst.actions[choice].cost;
		state |= inst.actions[choice].eff;
		feasible_actions.remove(choice, inst.actions[choice].pre);

		if (_CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static inline void randheur(hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running rand algorithm.");

	// greedy choice
	auto find_best_act = [](const std::vector<size_t>& candidates) {
		return std::pair<bool, size_t>(true, candidates[rand() % candidates.size()]);
	};

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	binary_set state(inst.n); // initial state is empty

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (auto act_i : hplus::act_remaining(inst))
		feasible_actions.add(act_i, inst.actions[act_i].pre);

	while (!state.contains(inst.goal)) {
		const auto& candidates = feasible_actions.find_subsets(state);
		if (candidates.empty()) [[unlikely]] {
			env.sol_s = solution_status::INFEAS;
			return;
		}

		const auto& [_, choice] = find_best_act(candidates);

		heur_sol.plan.push_back(choice);
		heur_sol.cost += inst.actions[choice].cost;
		state |= inst.actions[choice].eff;
		feasible_actions.remove(choice, inst.actions[choice].pre);

		if (_CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static inline void randr(hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running randr algorithm.");
	// [ ]: (maybe) Choose on CLI
	size_t repetitions = 100;
	// [ ]: Threads (if using threads, make hplus::update_sol thread safe.)
	for (size_t _ = 0; _ < repetitions; _++) {
		randheur(inst, env, log);
		if (env.sol_s == solution_status::INFEAS) [[unlikely]]
			return;
	}
}

[[nodiscard]]
static inline double evaluate_htype_state(const std::vector<size_t>& state, const std::vector<double>& values, double (*h_eqtype)(double, double)) {
	double state_htype = 0;
	for (auto p : state)
		state_htype = h_eqtype(state_htype, values[p]);
	return state_htype;
}

static inline void update_htype_values(const hplus::instance& inst, const binary_set& state, std::vector<double>& values, priority_queue<double>& pq, const binary_set& used_actions, double (*h_eqtype)(double, double)) {
	for (auto p : state) {
		values[p] = 0;
		if (!pq.has(p))
			pq.push(p, 0);
		else
			pq.change(p, 0);
	}

	while (!pq.empty()) {
		size_t p = pq.top();
		pq.pop();
		for (auto act_i : inst.act_with_pre[p]) {
			if (used_actions[act_i])
				continue;

			double cost_pre = evaluate_htype_state(inst.actions[act_i].pre_sparse, values, h_eqtype);
			if (cost_pre >= std::numeric_limits<double>::infinity())
				continue;

			double new_cost = cost_pre + inst.actions[act_i].cost;
			for (auto p_eff : inst.actions[act_i].eff_sparse) {
				if (new_cost >= values[p_eff])
					continue;

				values[p_eff] = new_cost;
				if (pq.has(p_eff))
					pq.change(p_eff, new_cost);
				else
					pq.push(p_eff, new_cost);
			}
		}
	}
}

static inline void init_htype(const hplus::instance& inst, const bs_searcher& feasible_actions, std::vector<double>& values, priority_queue<double>& pq, double (*h_eqtype)(double, double)) {
	const auto& initial_actions = feasible_actions.find_subsets(binary_set(inst.n));
	for (auto act_i : initial_actions) {
		// preconditions of these variables are already met -> hmax/hadd (act.pre) = 0 -> need only the cost of the action to set the hmax/hadd of the effects
		double cost = static_cast<double>(inst.actions[act_i].cost);
		for (auto p : inst.actions[act_i].eff_sparse) {
			if (cost >= values[p])
				continue;

			values[p] = cost;
			if (pq.has(p))
				pq.change(p, cost);
			else
				pq.push(p, cost);
		}
	}
	update_htype_values(inst, binary_set(inst.n), values, pq, binary_set(inst.m), h_eqtype);
}

[[nodiscard]]
static inline bool htype(const hplus::instance& inst, hplus::solution& sol, double (*h_eqtype)(double, double)) {

	auto find_best_act = [&inst, &h_eqtype](const std::vector<size_t>& candidates, const std::vector<double>& values, const binary_set& state) {
		size_t choice = 0;
		bool   found = false;
		double best_cxwe = std::numeric_limits<double>::infinity();
		for (auto act_i : candidates) {
			if (inst.actions[act_i].cost == 0)
				return std::pair<bool, size_t>(true, act_i);
			int weighted_neff = 0, neff = 0;
			for (auto var_i : inst.actions[act_i].eff_sparse) {
				if (!state[var_i]) {
					weighted_neff += values[var_i];
					neff++;
				}
			}
			if (neff == 0) // if no new effects, this action won't change the state... if no action changes the state this problem is infeasible
				continue;
			double cxwe = (weighted_neff == 0 ? std::numeric_limits<double>::infinity() : static_cast<double>(inst.actions[act_i].cost) / weighted_neff); // if weighted_neff is 0 this means that all effects can be archieved with 0 cost actions, while this action is not a 0 cost one... -> we don't want this action
			if (cxwe >= best_cxwe)																														  // if all actions have weighted_neff at 0, the best_cwe is never updated... however, if all actions have weighted_neff at 0, there must be an action with cost 0 among the candidate ones, hence we are not pruning feasible solution with >= insthead of > here.
				continue;

			choice = act_i;
			best_cxwe = cxwe;
			found = true;
		}
		return std::pair<bool, size_t>(found, choice);
	};

	// Reset solution (just to be sure)
	sol.plan.clear();
	sol.plan.reserve(inst.m_opt);
	sol.cost = 0;

	// Initialize helpers
	std::vector<double>	   values(inst.n, std::numeric_limits<double>::infinity());
	binary_set			   state(inst.n), used_actions(inst.m);
	priority_queue<double> pq(inst.n);

	auto find_best_act_v2 = [&inst, &h_eqtype, &used_actions, &pq](const std::vector<size_t>& candidates, const std::vector<double>& values, const binary_set& state) {
		size_t choice = 0;
		bool   found = false;
		double best_goal_cost = std::numeric_limits<double>::infinity();
		for (auto act_i : candidates) {
			std::vector<double> values_copy{ values };
			update_htype_values(inst, inst.actions[act_i].eff - state, values_copy, pq, used_actions, h_eqtype);
			double goal_cost = evaluate_htype_state(inst.goal.sparse(), values_copy, h_eqtype);
			if (goal_cost >= best_goal_cost)
				continue;

			choice = act_i;
			best_goal_cost = goal_cost;
			found = true;
		}
		return std::pair<bool, size_t>(found, choice);
	};

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (auto act_i : hplus::act_remaining(inst))
		feasible_actions.add(act_i, inst.actions[act_i].pre);

	// initialize values and priority queue (needs to be done manually since here the initial state will always be empty)
	init_htype(inst, feasible_actions, values, pq, h_eqtype);

	while (!state.contains(inst.goal)) {
		const auto& candidates = feasible_actions.find_subsets(state);
		if (candidates.empty()) [[unlikely]]
			return false;

		const auto& [found, choice] = find_best_act_v2(candidates, values, state);
		if (!found) [[unlikely]]
			return false;

		used_actions.add(choice);
		update_htype_values(inst, inst.actions[choice].eff - state, values, pq, used_actions, h_eqtype);

		sol.plan.push_back(choice);
		sol.cost += inst.actions[choice].cost;
		state |= inst.actions[choice].eff;
		feasible_actions.remove(choice, inst.actions[choice].pre);

		if (_CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	return true;
}

static inline void hmax(hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running hmax algorithm.");

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	if (!htype(inst, heur_sol, [](double a, double b) { return (a > b ? a : b); })) {
		env.sol_s = solution_status::INFEAS;
		return;
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
	return;
}

static inline void hadd(hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running hadd algorithm.");

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	if (!htype(inst, heur_sol, [](double a, double b) { return a + b; })) {
		env.sol_s = solution_status::INFEAS;
		return;
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
	return;
}

[[nodiscard]]
static inline bool random_walk(const hplus::instance& inst, hplus::solution& sol, const double* plan, const logger& log) {
	binary_set state(inst.n);

	// to be sure
	sol.plan.reserve(inst.m_opt);
	sol.cost = 0;

	auto find_best_act = [&inst, plan](const std::vector<size_t>& candidates) {
		double weight_sum = 0;
		for (auto x : candidates)
			weight_sum += plan[inst.act_cpxtoidx[x]];

		if (weight_sum <= 0)
			return std::pair<bool, size_t>(true, candidates[rand() % candidates.size()]);

		std::random_device				 rd;
		std::mt19937					 gen(rd());
		std::uniform_real_distribution<> dis(0.0, weight_sum);
		double							 random_value = dis(gen);
		double							 cumulative_weight = 0;
		for (auto x : candidates) {
			cumulative_weight += plan[inst.act_cpxtoidx[x]];
			if (random_value <= cumulative_weight)
				return std::pair<bool, size_t>(true, x);
		}

		return std::pair<bool, size_t>(true, candidates.back());
	};

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (auto act_i : hplus::act_remaining(inst))
		feasible_actions.add(act_i, inst.actions[act_i].pre);

	while (!state.contains(inst.goal)) {
		const auto& candidates = feasible_actions.find_subsets(state);
		if (candidates.empty()) [[unlikely]]
			return false;

		const auto& [found, choice] = find_best_act(candidates);
		if (!found) [[unlikely]]
			return false;

		sol.plan.push_back(choice);
		sol.cost += inst.actions[choice].cost;
		state |= inst.actions[choice].eff;
		feasible_actions.remove(choice, inst.actions[choice].pre);

		if (_CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	return true;
}

static inline void relax(hplus::instance& inst, hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running relax algorithm (linear relaxation of Imai's model).");

	// build cplex model
	CPXENVptr cpxenv = nullptr;
	CPXLPptr  cpxlp = nullptr;
	cpx_init(cpxenv, cpxlp, env, log, false);
	cpx_build_imai(cpxenv, cpxlp, inst, env, log, true);

	// change bounds of variables
	int*  indices = new int[inst.n_opt + inst.m_opt + inst.n_opt * inst.m_opt];
	char* ctypes = new char[inst.n_opt + inst.m_opt + inst.n_opt * inst.m_opt];
	for (size_t i = 0; i < inst.m_opt; i++) {
		indices[i] = i;
		ctypes[i] = 'C';
	}
	for (size_t i = 0; i < inst.n_opt; i++) {
		indices[inst.m_opt + i] = 2 * inst.m_opt + i;
		ctypes[inst.m_opt + i] = 'C';
	}
	for (size_t i = 0; i < inst.m_opt * inst.n_opt; i++) {
		indices[inst.m_opt + inst.n_opt + i] = 2 * inst.m_opt + 2 * inst.n_opt + i;
		ctypes[inst.m_opt + inst.n_opt + i] = 'C';
	}
	_ASSERT_LOG(log, !CPXchgctype(cpxenv, cpxlp, inst.n_opt + inst.m_opt + inst.n_opt * inst.m_opt, indices, ctypes));
	delete[] ctypes;
	ctypes = nullptr;
	delete[] indices;
	indices = nullptr;

	// solve model
	_ASSERT_LOG(log, !CPXmipopt(cpxenv, cpxlp));

	// parse cplex status to see what it found
	switch (int cpxstatus = CPXgetstat(cpxenv, cpxlp)) {
		case CPXMIP_TIME_LIM_INFEAS: // exceeded time limit, no intermediate solution found
		case CPXMIP_ABORT_INFEAS:	 // terminated by user, not found solution
		case CPXMIP_INFEASIBLE:		 // proven to be unfeasible
			env.sol_s = solution_status::INFEAS;
			return;
		case CPXMIP_TIME_LIM_FEAS: // exceeded time limit, found intermediate solution
		case CPXMIP_ABORT_FEAS:	   // terminated by user, found solution
		case CPXMIP_OPTIMAL_TOL:   // found optimal
		case CPXMIP_OPTIMAL:
		case CPX_STAT_OPTIMAL:
			break;
		default: // unhandled status
			log.raise_error("Error in parse_cpx_status: unhandled cplex status: %d.", cpxstatus);
	}

	// build a solution from the relaxation
	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;
	double* plan = new double[inst.m_opt];
	_ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt - 1));
	// [ ] Rethink how we reconstruct our solution... this one isn't a good approximate solution
	if (!random_walk(inst, heur_sol, plan, log)) {
		delete[] plan;
		env.sol_s = solution_status::INFEAS;
		return;
	}
	delete[] plan;
	cpx_close(cpxenv, cpxlp);

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static inline void localsearch(hplus::instance& inst, void (*heuristic)(hplus::instance& inst, hplus::environment& env, const logger& log), hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Running local-search algorithm.");
	heuristic(inst, env, log);

	if (env.sol_s == solution_status::INFEAS) [[unlikely]]
		return;

	hplus::solution heur_sol{ .plan = inst.best_sol.plan, .cost = inst.best_sol.cost };

	// [ ]: Local search on top of the initial solution

	hplus::update_sol(inst, heur_sol, log);
}

void find_heuristic(hplus::instance& inst, hplus::environment& env, const logger& log) {
	srand(time(0));
	if (env.heur == "greedycost")
		greedycost(inst, env, log);
	else if (env.heur == "greedycxe")
		greedycxe(inst, env, log);
	else if (env.heur == "rand")
		randheur(inst, env, log);
	else if (env.heur == "randr")
		randr(inst, env, log);
	else if (env.heur == "hmax")
		hmax(inst, env, log);
	else if (env.heur == "hadd")
		hadd(inst, env, log);
	else if (env.heur == "relax")
		relax(inst, env, log);
	else if (env.heur == "local-greedycost")
		localsearch(inst, greedycost, env, log);
	else if (env.heur == "local-greedycxe")
		localsearch(inst, greedycxe, env, log);
	else if (env.heur == "local-rand")
		localsearch(inst, randheur, env, log);
	else if (env.heur == "local-randr")
		localsearch(inst, randr, env, log);
	else if (env.heur == "local-hmax")
		localsearch(inst, hmax, env, log);
	else if (env.heur == "local-hadd")
		localsearch(inst, hadd, env, log);
	else if (env.heur == "local-relax")
		localsearch(inst, relax, env, log);
	else
		log.raise_error("The heuristic specified (%s) is not on the list of possible "
						"heuristics... Please read the Readme.md for instructions.",
						env.heur.c_str());
}

// ##################################################################### //
// ################################ IMAI ############################### //
// ##################################################################### //

void cpx_build_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log, bool relaxed) {
	if (!relaxed)
		_PRINT_VERBOSE(log, "Building imai's model.");

	auto stopchk1 = []() {
		if (_CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	};

	const auto& rem_var = hplus::var_remaining(inst);
	const auto& rem_act = hplus::act_remaining(inst);

	// ====================================================== //
	// ============== TIGHTER TIMESTAMPS BOUNDS ============= //
	// ====================================================== //

	int timestamps_ubound = inst.m_opt;
	if (env.imai_tight_bounds) {
		// number of variables
		if (inst.n_opt < timestamps_ubound)
			timestamps_ubound = inst.n_opt;

		// max number of steps to reach heuristic
		if (env.heur != "none" && !relaxed) {
			unsigned int min_act_cost = inst.actions[0].cost + 1; // +1 to avoid it being 0
			size_t		 n_act_zerocost = 0;
			for (auto act : inst.actions) {
				if (act.cost == 0)
					n_act_zerocost++;
				else if (act.cost < min_act_cost) [[unlikely]]
					min_act_cost = act.cost;
			}
			int nsteps = inst.best_sol.cost / min_act_cost + n_act_zerocost;
			if (nsteps < timestamps_ubound)
				timestamps_ubound = nsteps;
		}
	}
	stopchk1();

	// ====================================================== //
	// =================== CPLEX VARIABLES ================== //
	// ====================================================== //
	// (section 3 of Imai's paper)

	size_t curr_col = 0;

	double* objs = new double[inst.m_opt];
	double* lbs = new double[inst.m_opt];
	double* ubs = new double[inst.m_opt];
	char*	types = new char[inst.m_opt];

	auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
		delete[] types;
		types = nullptr;
		delete[] ubs;
		ubs = nullptr;
		delete[] lbs;
		lbs = nullptr;
		delete[] objs;
		objs = nullptr;

		objs = new double[new_size];
		lbs = new double[new_size];
		ubs = new double[new_size];
		types = new char[new_size];
	};

	auto stopchk2 = [&objs, &lbs, &ubs, &types]() {
		if (_CHECK_STOP()) [[unlikely]] {
			delete[] types;
			types = nullptr;
			delete[] ubs;
			ubs = nullptr;
			delete[] lbs;
			lbs = nullptr;
			delete[] objs;
			objs = nullptr;
			throw timelimit_exception("Reached time limit.");
		}
	};

	// -------- actions ------- //
	size_t act_start = curr_col;
	size_t count = 0;
	for (auto act_i : rem_act) {
		objs[count] = static_cast<double>(inst.actions[act_i].cost);
		lbs[count] = inst.act_f[act_i] ? 1 : 0;
		ubs[count] = 1;
		types[count++] = 'B';
	}
	_INTCHECK_ASSERT_LOG(log, count == inst.m_opt);
	curr_col += inst.m_opt;

	_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// --- action timestamps -- //
	size_t tact_start = curr_col;
	count = 0;
	for (auto act_i : rem_act) {
		objs[count] = 0;
		lbs[count] = inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : 0;
		ubs[count] = inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : timestamps_ubound - 1;
		types[count++] = 'I';
	}
	_INTCHECK_ASSERT_LOG(log, count == inst.m_opt);
	curr_col += inst.m_opt;

	_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	resize_cpx_arrays(inst.n_opt);

	// ------- variables ------ //
	size_t var_start = curr_col;
	count = 0;
	for (auto var_i : rem_var) {
		objs[count] = 0;
		lbs[count] = (inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0;
		ubs[count] = 1;
		types[count++] = 'B';
	}
	_INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
	curr_col += inst.n_opt;

	_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// -- variable timestamps - //
	size_t tvar_start = curr_col;
	count = 0;
	for (auto i : rem_var) {
		objs[count] = 0;
		lbs[count] = inst.var_t[i] >= 0 ? inst.var_t[i] : 0;
		ubs[count] = inst.var_t[i] >= 0 ? inst.var_t[i] : timestamps_ubound;
		types[count++] = 'I';
	}
	_INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
	curr_col += inst.n_opt;

	_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// --- first archievers --- //
	size_t fa_start = curr_col;
	count = 0;
	for (auto act_i : rem_act) {
		size_t count_var = 0;
		for (auto var_i : rem_var) {
			objs[count_var] = 0;
			lbs[count_var] = inst.fadd_f[act_i][var_i] ? 1 : 0;
			ubs[count_var] = (!inst.actions[act_i].eff[var_i] || inst.fadd_e[act_i][var_i]) ? 0 : 1;
			types[count_var++] = 'B';
		}
		_INTCHECK_ASSERT_LOG(log, count_var == inst.n_opt);
		curr_col += inst.n_opt;
		_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
		count++;
		stopchk2();
	}

	delete[] types;
	types = nullptr;
	delete[] ubs;
	ubs = nullptr;
	delete[] lbs;
	lbs = nullptr;
	delete[] objs;
	objs = nullptr;

	// ====================================================== //
	// ================== CPLEX CONSTRAINTS ================= //
	// ====================================================== //
	// (section 3 of Imai's paper)

	// accessing cplex variables
	auto get_act_idx = [&inst, &act_start](size_t idx) { return act_start + inst.act_opt_conv[idx]; };
	auto get_tact_idx = [&inst, &tact_start](size_t idx) { return tact_start + inst.act_opt_conv[idx]; };
	auto get_var_idx = [&inst, &var_start](size_t idx) { return var_start + inst.var_opt_conv[idx]; };
	auto get_tvar_idx = [&inst, &tvar_start](size_t idx) { return tvar_start + inst.var_opt_conv[idx]; };
	auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_idx) { return fa_start + inst.fadd_checkpoint[inst.act_opt_conv[act_idx]] + inst.var_opt_conv[var_idx]; };

	int*		 ind_c1 = new int[inst.m_opt + 1];
	double*		 val_c1 = new double[inst.m_opt + 1];
	int			 ind_c2_4[2], ind_c5[3];
	double		 val_c2_4[2], val_c5[3];
	const int	 nnz_c2_4 = 2, nnz_c5 = 3;
	const char	 sensel = 'L', sensee = 'E';
	const double rhs_c1_2_4 = 0, rhs_c5 = timestamps_ubound;
	const int	 begin = 0;

	std::vector<int*>	 ind_c3(inst.n_opt);
	std::vector<double*> val_c3(inst.n_opt);
	std::vector<int>	 nnz_c3(inst.n_opt);
	std::vector<double>	 rhs_c3(inst.n_opt);

	auto stopchk3 = [&inst, &ind_c1, &val_c1, &ind_c3, &val_c3]() {
		if (_CHECK_STOP()) [[unlikely]] {
			for (size_t var_i = 0; var_i < inst.n_opt; var_i++) {
				delete[] ind_c3[var_i];
				ind_c3[var_i] = nullptr;
				delete[] val_c3[var_i];
				val_c3[var_i] = nullptr;
			}
			delete[] val_c1;
			val_c1 = nullptr;
			delete[] ind_c1;
			ind_c1 = nullptr;
			throw timelimit_exception("Reached time limit.");
		}
	};

	for (auto i : rem_var) {
		ind_c3[inst.var_opt_conv[i]] = new int[inst.m_opt + 1];
		val_c3[inst.var_opt_conv[i]] = new double[inst.m_opt + 1];
		nnz_c3[inst.var_opt_conv[i]] = 0;
		rhs_c3[inst.var_opt_conv[i]] = 0;
		ind_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = get_var_idx(i);
		val_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = 1;
		nnz_c3[inst.var_opt_conv[i]]++;
	}
	stopchk3();

	for (auto act_i : rem_act) {
		const auto& inverse_actions = inst.act_inv[act_i];
		for (auto var_i : inst.actions[act_i].pre_sparse) {
			// constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
			ind_c1[0] = get_act_idx(act_i);
			val_c1[0] = 1;
			ind_c1[1] = get_var_idx(var_i);
			val_c1[1] = -1;
			int nnz0 = 2;
			// (section 4.6 of Imai's paper)
			for (size_t i = 0; i < inverse_actions.size(); i++) {
				if (inst.actions[inverse_actions[i]].eff[var_i]) {
					ind_c1[nnz0] = get_fa_idx(inverse_actions[i], var_i);
					val_c1[nnz0++] = 1;
				}
			}
			_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz0, &rhs_c1_2_4, &sensel, &begin, ind_c1, val_c1, nullptr, nullptr));
			// constraint 4: t_vj <= t_a, vj in pre(a)
			ind_c2_4[0] = get_tvar_idx(var_i);
			val_c2_4[0] = 1;
			ind_c2_4[1] = get_tact_idx(act_i);
			val_c2_4[1] = -1;
			_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
		}
		for (auto var_i : inst.actions[act_i].eff_sparse) {
			// constraint 2: z_avj <= x_a, vj in eff(a)
			ind_c2_4[0] = get_fa_idx(act_i, var_i);
			val_c2_4[0] = 1;
			ind_c2_4[1] = get_act_idx(act_i);
			val_c2_4[1] = -1;
			_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
			// constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
			ind_c5[0] = get_tact_idx(act_i);
			val_c5[0] = 1;
			ind_c5[1] = get_tvar_idx(var_i);
			val_c5[1] = -1;
			ind_c5[2] = get_fa_idx(act_i, var_i);
			val_c5[2] = timestamps_ubound + 1;
			_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c5, &rhs_c5, &sensel, &begin, ind_c5, val_c5, nullptr, nullptr));
			// constraint 3: I(v_j) + sum(z_avj) = y_vj
			ind_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = get_fa_idx(act_i, var_i);
			val_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = -1;
			nnz_c3[inst.var_opt_conv[var_i]]++;
		}
		stopchk3();
	}

	mypause();

	for (size_t var_i = 0; var_i < inst.n_opt; var_i++)
		_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c3[var_i], &rhs_c3[var_i], &sensee, &begin, ind_c3[var_i], val_c3[var_i], nullptr, nullptr));

	for (size_t var_i = 0; var_i < inst.n_opt; var_i++) {
		delete[] ind_c3[var_i];
		ind_c3[var_i] = nullptr;
		delete[] val_c3[var_i];
		val_c3[var_i] = nullptr;
	}
	delete[] val_c1;
	val_c1 = nullptr;
	delete[] ind_c1;
	ind_c1 = nullptr;

	// _ASSERT_LOG(log, !CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

void cpx_post_warmstart_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_INTCHECK_ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

	auto state = binary_set(inst.n);

	const auto& warm_start = inst.best_sol.plan;

	size_t	ncols = CPXgetnumcols(cpxenv, cpxlp);
	int*	cpx_sol_ind = new int[ncols];
	double* cpx_sol_val = new double[ncols];

	int	   izero = 0;
	int	   effortlevel = CPX_MIPSTART_REPAIR;
	size_t nnz = 0, timestamp = 0;

	for (auto act_i : warm_start) {
		cpx_sol_ind[nnz] = inst.act_opt_conv[act_i];
		cpx_sol_val[nnz++] = 1;
		cpx_sol_ind[nnz] = inst.m_opt + inst.act_opt_conv[act_i];
		cpx_sol_val[nnz++] = timestamp;
		timestamp++;
		for (auto var_i : inst.actions[act_i].eff_sparse) {
			if (state[var_i])
				continue;

			cpx_sol_ind[nnz] = 2 * inst.m_opt + inst.var_opt_conv[var_i];
			cpx_sol_val[nnz++] = 1;
			cpx_sol_ind[nnz] = 2 * inst.m_opt + inst.n_opt + inst.var_opt_conv[var_i];
			cpx_sol_val[nnz++] = timestamp;
			cpx_sol_ind[nnz] = 2 * inst.m_opt + 2 * inst.n_opt + inst.fadd_checkpoint[inst.act_opt_conv[act_i]] + inst.var_opt_conv[var_i];
			cpx_sol_val[nnz++] = 1;
			state.add(var_i);
		}
	}

	_ASSERT_LOG(log, !CPXaddmipstarts(cpxenv, cpxlp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));

	delete[] cpx_sol_ind;
	cpx_sol_ind = nullptr;
	delete[] cpx_sol_val;
	cpx_sol_val = nullptr;
}

void store_imai_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Storing imai's solution.");

	// get cplex result (interested only in the sequence of actions [0/inst.m_opt-1]
	// used and its ordering [inst.m_opt/2nact-1])
	double* plan = new double[2 * inst.m_opt];
	_ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, 2 * inst.m_opt - 1));

	// convert to std collections for easier parsing
	std::vector<std::pair<double, size_t>> cpx_result;
	for (size_t i = 0; i < inst.m_opt; i++) {
		if (plan[i] > HPLUS_CPX_INT_ROUNDING)
			cpx_result.emplace_back(plan[inst.m_opt + i], i);
	}
	delete[] plan;
	plan = nullptr;

	// sort cpx_result based on actions timestamps
	std::sort(cpx_result.begin(), cpx_result.end(), [](const std::pair<double, size_t>& x, const std::pair<double, size_t>& y) { return x.first < y.first; });

	// get solution from sorted cpx_result
	std::vector<size_t> solution;
	std::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution), [inst](const std::pair<double, size_t>& p) { return inst.act_cpxtoidx[p.second]; });

	// store solution
	hplus::solution imai_sol{ solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [inst](const size_t acc, const size_t index) { return acc + inst.actions[index].cost; })) };
	hplus::update_sol(inst, imai_sol, log);
}

// ##################################################################### //
// ############################## RANKOOH ############################## //
// ##################################################################### //

void cpx_build_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Building rankooh's model.");

	auto stopchk1 = []() {
		if (_CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	};

	const auto& rem_var = hplus::var_remaining(inst);
	const auto& rem_act = hplus::act_remaining(inst);

	// ====================================================== //
	// ================= VERTEX ELIMINATION ================= //
	// ====================================================== //

	struct node {
		size_t id, deg;
		node(size_t id, size_t deg)
			: id(id), deg(deg) {}
	};
	struct compare_node {
		bool operator()(const node& n1, const node& n2) { return n1.deg > n2.deg; }
	};
	struct triangle {
		size_t first, second, third;
		triangle(size_t first, size_t second, size_t third)
			: first(first), second(second), third(third) {}
	};

	std::vector<std::set<size_t>> graph(inst.n);
	std::vector<binary_set>		  cumulative_graph(inst.n, binary_set(inst.n));
	std::vector<triangle>		  triangles_list;

	// [ ]: (maybe) Implement using pq.hxx
	std::priority_queue<node, std::vector<node>, compare_node> nodes_queue;
	std::vector<size_t>										   degree_counter(inst.n, 0);

	// G_0
	for (auto act_i : rem_act) {
		for (auto var_i : inst.actions[act_i].pre_sparse) {
			for (auto var_j : inst.actions[act_i].eff_sparse) {
				if (var_i == var_j) [[unlikely]]
					continue;

				// added is true if the element was actually inserted in the set
				if (graph[var_i].insert(var_j).second) {
					degree_counter[var_i] += 1;
					degree_counter[var_j] += 1;
				}
			}
			cumulative_graph[var_i] |= (inst.actions[act_i].eff);
		}
	}
	stopchk1();

	for (size_t node_i = 0; node_i < inst.n; node_i++) {
		if (degree_counter[node_i] > 0) [[likely]]
			nodes_queue.emplace(node_i, degree_counter[node_i]);
	}

	// finding minimum degree node
	auto find_min = [&inst, &degree_counter](std::priority_queue<node, std::vector<node>, compare_node>& nodes_queue) {
		int idx = -1;
		while (!nodes_queue.empty() && idx < 0) {
			node tmp = nodes_queue.top();
			nodes_queue.pop();
			if (degree_counter[tmp.id] == tmp.deg)
				idx = tmp.id;
		}
		return idx;
	};

	// G_i (min degree heuristics)
	for (auto _ : rem_var) {
		int idx = find_min(nodes_queue);
		if (idx == -1) [[unlikely]]
			break;

		// graph structure:
		// | \       > |
		// p -> idx -> q
		// | /       > |

		std::set<size_t> new_nodes;

		for (auto p : rem_var) {
			if (graph[p].find(idx) == graph[p].end())
				continue;

			for (auto q : graph[idx]) {
				if (p == q) [[unlikely]]
					continue;

				// add edge p - q
				// check.second is true if the element was actually inserted in the set
				if (graph[p].insert(q).second) {
					degree_counter[p] += 1;
					degree_counter[q] += 1;
				}

				// update the overall graph
				cumulative_graph[p].add(q);
			}

			// remove the edge p - idx
			graph[p].erase(idx);
			degree_counter[p] -= 1;
			new_nodes.insert(p);

			// update triangles list
			for (auto q : graph[idx]) {
				if (p != q) [[likely]]
					triangles_list.emplace_back(p, idx, q);
			}
		}

		// remove the edge idx - q
		for (auto q : graph[idx]) {
			degree_counter[q] -= 1;
			new_nodes.insert(q);
		}
		graph[idx].clear();
		degree_counter[idx] = 0;

		// Update the priority queue
		for (auto node : new_nodes) {
			if (degree_counter[node] > 0)
				nodes_queue.emplace(node, degree_counter[node]);
		}

#if HPLUS_INTCHECK // care: this takes HUGE amount of time
		for (size_t node_i = 0; node_i < inst.n; node_i++) {
			size_t i_cnt = 0;
			i_cnt += graph[node_i].size();
			for (size_t tmp_j = 0; tmp_j < inst.n; tmp_j++) {
				for (auto tmp_k : graph[tmp_j]) {
					if (tmp_k == node_i)
						i_cnt += 1;
				}
			}
			_ASSERT_LOG(log, i_cnt == degree_counter[node_i]);
		}
#endif
		stopchk1();
	}

	// ====================================================== //
	// =================== CPLEX VARIABLES ================== //
	// ====================================================== //

	size_t	curr_col = 0;
	double* objs = new double[inst.m_opt];
	double* lbs = new double[inst.m_opt];
	double* ubs = new double[inst.m_opt];
	char*	types = new char[inst.m_opt];

	auto resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
		delete[] types;
		types = nullptr;
		delete[] ubs;
		ubs = nullptr;
		delete[] lbs;
		lbs = nullptr;
		delete[] objs;
		objs = nullptr;

		objs = new double[new_size];
		lbs = new double[new_size];
		ubs = new double[new_size];
		types = new char[new_size];
	};

	auto stopchk2 = [&objs, &lbs, &ubs, &types]() {
		if (_CHECK_STOP()) [[unlikely]] {
			delete[] types;
			types = nullptr;
			delete[] ubs;
			ubs = nullptr;
			delete[] lbs;
			lbs = nullptr;
			delete[] objs;
			objs = nullptr;
			throw timelimit_exception("Reached time limit.");
		}
	};

	// -------- actions ------- //
	size_t act_start = curr_col;
	size_t count = 0;
	for (auto act_i : rem_act) {
		objs[count] = static_cast<double>(inst.actions[act_i].cost);
		lbs[count] = inst.act_f[act_i] ? 1 : 0;
		ubs[count] = 1;
		types[count++] = 'B';
	}
	_INTCHECK_ASSERT_LOG(log, count == inst.m_opt);

	curr_col += inst.m_opt;

	_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	resize_cpx_arrays(inst.n_opt);

	// --- first archievers --- //
	size_t				fa_start = curr_col;
	std::vector<size_t> fa_individual_start(inst.m_opt);
	count = 0;
	for (auto act_i : rem_act) {
		fa_individual_start[count] = count * inst.n_opt;
		size_t count_var = 0;
		for (auto var_i : rem_var) {
			objs[count_var] = 0;
			lbs[count_var] = inst.fadd_f[act_i][var_i] ? 1 : 0;
			ubs[count_var] = (!inst.actions[act_i].eff[var_i] || inst.fadd_e[act_i][var_i]) ? 0 : 1;
			types[count_var++] = 'B';
		}
		_INTCHECK_ASSERT_LOG(log, count_var == inst.n_opt);
		curr_col += inst.n_opt;
		_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
		count++;
		stopchk2();
	}

	// ------- variables ------ //
	size_t var_start = curr_col;
	count = 0;
	for (auto var_i : rem_var) {
		objs[count] = 0;
		lbs[count] = (inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0;
		ubs[count] = 1;
		types[count++] = 'B';
	}

	_INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
	curr_col += inst.n_opt;

	_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// vertex elimination graph edges
	size_t veg_edges_start = curr_col;
	for (auto var_i : rem_var) {
		count = 0;
		for (auto var_j : rem_var) {
			objs[count] = 0;
			lbs[count] = 0;
			ubs[count] = cumulative_graph[var_i][var_j] ? 1 : 0;
			types[count++] = 'B';
		}
		_INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
		_ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
		stopchk2();
	}
	curr_col += inst.n_opt * inst.n_opt;

	delete[] types;
	types = nullptr;
	delete[] ubs;
	ubs = nullptr;
	delete[] lbs;
	lbs = nullptr;
	delete[] objs;
	objs = nullptr;

	// ====================================================== //
	// ================== CPLEX CONSTRAINTS ================= //
	// ====================================================== //

	// accessing cplex variables
	auto get_act_idx = [&inst, &act_start](size_t idx) { return act_start + inst.act_opt_conv[idx]; };
	auto get_var_idx = [&inst, &var_start](size_t idx) { return var_start + inst.var_opt_conv[idx]; };
	auto get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_idx) { return fa_start + inst.fadd_checkpoint[inst.act_opt_conv[act_idx]] + inst.var_opt_conv[var_idx]; };
	auto get_veg_idx = [&inst, &veg_edges_start](size_t idx_i, size_t idx_j) { return veg_edges_start + inst.var_opt_conv[idx_i] * inst.n_opt + inst.var_opt_conv[idx_j]; };

	int*		 ind = new int[inst.m_opt + 1];
	double*		 val = new double[inst.m_opt + 1];
	int			 nnz = 0;
	const char	 sense_e = 'E', sense_l = 'L';
	const double rhs_0 = 0, rhs_1 = 1;
	const int	 begin = 0;

	auto stopchk3 = [&ind, &val]() {
		if (_CHECK_STOP()) [[unlikely]] {
			delete[] val;
			val = nullptr;
			delete[] ind;
			ind = nullptr;
			throw timelimit_exception("Reached time limit.");
		}
	};

	for (auto var_i : rem_var) {
		nnz = 0;
		ind[nnz] = get_var_idx(var_i);
		val[nnz++] = 1;

		for (auto act_i : inst.act_with_eff[var_i]) {
			ind[nnz] = get_fa_idx(act_i, var_i);
			val[nnz++] = -1;
		}

		_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr));
		stopchk3();
	}

	for (auto var_i : rem_var) {
		for (auto var_j : rem_var) {
			nnz = 0;
			ind[nnz] = get_var_idx(var_j);
			val[nnz++] = -1;
			for (auto act_i : inst.act_with_eff[var_i]) {
				if (inst.actions[act_i].pre[var_j]) [[unlikely]] {
					ind[nnz] = get_fa_idx(act_i, var_i);
					val[nnz++] = 1;
				}
			}
			_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr));
			stopchk3();
		}
	}

	delete[] val;
	val = nullptr;
	delete[] ind;
	ind = nullptr;

	int	   ind_c5_c6_c7[2], ind_c8[3];
	double val_c5_c6_c7[2], val_c8[3];

	for (auto act_i : rem_act) {
		for (auto var_i : inst.actions[act_i].eff_sparse) {
			ind_c5_c6_c7[0] = get_act_idx(act_i);
			val_c5_c6_c7[0] = -1;
			ind_c5_c6_c7[1] = get_fa_idx(act_i, var_i);
			val_c5_c6_c7[1] = 1;
			_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
		}
		stopchk1();
	}

	for (auto act_i : rem_act) {
		for (auto var_i : inst.actions[act_i].pre_sparse) {
			for (auto var_j : inst.actions[act_i].eff_sparse) {
				ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
				val_c5_c6_c7[0] = -1;
				ind_c5_c6_c7[1] = get_fa_idx(act_i, var_j);
				val_c5_c6_c7[1] = 1;
				_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
			}
			stopchk1();
		}
	}

	for (auto var_i : rem_var) {
		for (auto var_j : cumulative_graph[var_i]) {
			ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
			val_c5_c6_c7[0] = 1;
			ind_c5_c6_c7[1] = get_veg_idx(var_j, var_i);
			val_c5_c6_c7[1] = 1;
			_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
			stopchk1();
		}
	}

	for (auto [a, b, c] : triangles_list) {
		ind_c8[0] = get_veg_idx(a, b);
		val_c8[0] = 1;
		ind_c8[1] = get_veg_idx(b, c);
		val_c8[1] = 1;
		ind_c8[2] = get_veg_idx(a, c);
		val_c8[2] = -1;
		_ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr));
		stopchk1();
	}

	// _ASSERT_LOG(log, !CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

void cpx_post_warmstart_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_INTCHECK_ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

	binary_set state(inst.n);

	const auto& warm_start = inst.best_sol.plan;

	size_t	ncols = CPXgetnumcols(cpxenv, cpxlp);
	int*	cpx_sol_ind = new int[ncols];
	double* cpx_sol_val = new double[ncols];

	int	   izero = 0;
	int	   effortlevel = CPX_MIPSTART_REPAIR;
	size_t nnz = 0;

	for (auto act_i : warm_start) {
		cpx_sol_ind[nnz] = inst.act_opt_conv[act_i];
		cpx_sol_val[nnz++] = 1;
		for (auto var_i : inst.actions[act_i].eff_sparse) {
			if (state[var_i])
				continue;

			cpx_sol_ind[nnz] = inst.m_opt + inst.m_opt * inst.n_opt + inst.var_opt_conv[var_i];
			cpx_sol_val[nnz++] = 1;
			cpx_sol_ind[nnz] = inst.m_opt + inst.fadd_checkpoint[inst.act_opt_conv[act_i]] + inst.var_opt_conv[var_i];
			cpx_sol_val[nnz++] = 1;
			state.add(var_i);
		}
	}

	_ASSERT_LOG(log, !CPXaddmipstarts(cpxenv, cpxlp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));
	delete[] cpx_sol_ind;
	cpx_sol_ind = nullptr;
	delete[] cpx_sol_val;
	cpx_sol_val = nullptr;
}

void store_rankooh_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Storing rankooh's solution.");

	double* plan = new double[inst.m_opt + inst.m_opt * inst.n_opt];
	_ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt + inst.m_opt * inst.n_opt - 1));

	// fixing the solution to read the plan (some actions are set to 1 even if
	// they are not a first archiever of anything)
	for (size_t act_i_cpx = 0, fadd_i = inst.m_opt; act_i_cpx < inst.m_opt; act_i_cpx++, fadd_i += inst.n_opt) {
		bool set_zero = true;
		for (size_t var_i_cpx = 0; var_i_cpx < inst.n_opt; var_i_cpx++) {
			if (plan[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
				_INTCHECK_ASSERT_LOG(log, plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
				set_zero = false;
				break;
			}
		}
		if (set_zero)
			plan[act_i_cpx] = 0;
	}

	// convert to std collections for easier parsing
	std::vector<size_t> cpx_result;
	cpx_result.reserve(inst.m_opt);
	for (size_t i = 0; i < inst.m_opt; i++) {
		if (plan[i] > HPLUS_CPX_INT_ROUNDING)
			cpx_result.push_back(inst.act_cpxtoidx[i]);
	}
	delete[] plan;
	plan = nullptr;

	std::vector<size_t> solution;
	solution.reserve(inst.m_opt);
	binary_set remaining(cpx_result.size(), true), state(inst.n);

	while (!remaining.empty()) {
#if HPLUS_INTCHECK
		bool intcheck = false;
#endif
		for (auto i : remaining) {
			if (!state.contains(inst.actions[cpx_result[i]].pre))
				continue;

			remaining.remove(i);
			state |= inst.actions[cpx_result[i]].eff;
			solution.push_back(cpx_result[i]);
#if HPLUS_INTCHECK
			intcheck = true;
#endif
		}
		_INTCHECK_ASSERT_LOG(log, intcheck);
	}

	// store solution
	hplus::solution rankooh_sol{ solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [&inst](const unsigned int acc, const size_t index) { return acc + inst.actions[index].cost; })) };
	hplus::update_sol(
		inst, rankooh_sol, log);
}

// ##################################################################### //
// ########################### DYNAMIC SMALL ########################### //
// ##################################################################### //

void cpx_build_dynamic_small(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Building dynamic small model.");
	// [ ]: Building the small dynamic model
	todo(log, "Building the small dynamic model");
}

void cpx_post_warmstart_dynamic_small(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	// [ ]: Posting warmstart to the small dynamic model
	todo(log, "Posting warmstart to the small dynamic model");
}

void store_dynamic_small_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Storing the dynamic small solution.");
	// [ ]: Storing solution of the small dynamic model
	todo(log, "Storing solution of the small dynamic model");
}

// ##################################################################### //
// ########################### DYNAMIC LARGE ########################### //
// ##################################################################### //

void cpx_build_dynamic_large(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Building dynamic large model.");
	// [ ]: Building the large dynamic model
	todo(log, "Building the large dynamic model");
}

void cpx_post_warmstart_dynamic_large(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	// [ ]: Posting warmstart to the large dynamic model
	todo(log, "Posting warmstart to the large dynamic model");
}

void store_dynamic_large_sol(CPXENVptr& cpxenv, CPXLPptr& cpxlp, hplus::instance& inst, const hplus::environment& env, const logger& log) {
	_PRINT_VERBOSE(log, "Storing the dynamic large solution.");
	// [ ]: Storing solution of the large dynamic model
	todo(log, "Storing solution of the large dynamic model");
}