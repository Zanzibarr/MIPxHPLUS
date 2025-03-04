/**
 * @file algorithms.cpp
 * @brief Algorithms to solve the delete-free relaxation of the planning task
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#include "algorithms.hpp"
#include <numeric> // For std::accumulate
#include <queue>   // For std::priority_queue (// TODO remove when substituted with priority_queue)
#include <random>  // For random number generators
#include <set>	   // For std::set

#if HPLUS_INTCHECK == 0
	#define INTCHECK_PQ false
#endif
#include "pq.hxx" // For priority_queue

// ##################################################################### //
// ############################ CPLEX UTILS ############################ //
// ##################################################################### //

void cpx_init(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::environment& env, const logger& log, bool log_file) {
	PRINT_VERBOSE(log, "Initializing CPLEX.");
	int cpxerror;
	cpxenv = CPXopenCPLEX(&cpxerror);
	ASSERT_LOG(log, !cpxerror);
	cpxlp = CPXcreateprob(cpxenv, &cpxerror, env.run_name.c_str());
	ASSERT_LOG(log, !cpxerror);
	// log file
	ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_ScreenOutput, CPX_OFF));
	if (log_file)
		ASSERT_LOG(log, !CPXsetlogfilename(cpxenv, (HPLUS_CPLEX_OUTPUT_DIR "/log/" + env.run_name + ".log").c_str(), "w"));
	ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPX_PARAM_CLONELOG, -1));
	// tolerance
	ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_MIP_Tolerances_MIPGap, 0));
	// memory/size limits
	ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_MIP_Limits_TreeMemory, 12000));
	ASSERT_LOG(log, !CPXsetdblparam(cpxenv, CPXPARAM_WorkMem, 4096));
	ASSERT_LOG(log, !CPXsetintparam(cpxenv, CPXPARAM_MIP_Strategy_File, 3));
	// terminate condition
	ASSERT_LOG(log, !CPXsetterminate(cpxenv, &global_terminate));
}

void cpx_close(CPXENVptr& cpxenv, CPXLPptr& cpxlp) {
	CPXfreeprob(cpxenv, &cpxlp);
	CPXcloseCPLEX(&cpxenv);
}

bool parse_cpx_status(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Parsing CPLEX status code.");
	switch (const auto cpxstatus = CPXgetstat(cpxenv, cpxlp)) {
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
			PRINT_WARN(log, "Found optimal within the tolerance.");
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

static void greedycost(hplus::instance& inst, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running greedycost algorithm.");

	unsigned int timestamp = 0;
	// greedy choice
	const auto& find_best_act = [&inst, &timestamp](const std::vector<size_t>& candidates) {
		size_t choice = 0;
		auto   best_cost = std::numeric_limits<double>::infinity();
		for (const auto& act_i : candidates) {
			if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp)
				return std::pair(true, act_i);
			if (inst.act_f[act_i]) {
				choice = act_i;
				best_cost = -1;
				continue;
			}

			if (best_cost < 0)
				continue;

			const auto& cost = static_cast<double>(inst.actions[act_i].cost);
			if (cost >= best_cost)
				continue;

			choice = act_i;
			best_cost = cost;
		}
		return std::pair(true, choice);
	};

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	binary_set state(inst.n); // initial state is empty

	// binary_set searcher for faster actions lookup
	auto feasible_actions = bs_searcher(inst.n);
	for (const auto& act_i : inst.act_rem)
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
		timestamp++;

		if (CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static void greedycxe(hplus::instance& inst, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running greedycxe algorithm.");

	unsigned int timestamp = 0;
	// greedy choice
	const auto& find_best_act = [&inst, &timestamp](const std::vector<size_t>& candidates, const binary_set& state) {
		size_t choice = 0;
		bool   found = false;
		auto   best_cxe = std::numeric_limits<double>::infinity();
		for (const auto& act_i : candidates) {
			if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp)
				return std::pair(true, act_i);
			if (inst.act_f[act_i]) {
				choice = act_i;
				best_cxe = -1;
				found = true;
				continue;
			}

			if (best_cxe < 0)
				continue;

			auto neff = 0;
			for (const auto& var_i : inst.actions[act_i].eff_sparse) {
				if (!state[var_i])
					neff++;
			}
			if (neff == 0)
				continue;
			const auto& cxe = static_cast<double>(inst.actions[act_i].cost) / neff;
			if (cxe >= best_cxe)
				continue;

			found = true;
			choice = act_i;
			best_cxe = cxe;
		}
		return std::pair(found, choice);
	};

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	binary_set state(inst.n); // initial state is empty

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (const auto& act_i : inst.act_rem)
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
		timestamp++;

		if (CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static void randheur(hplus::instance& inst, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running rand algorithm.");

	unsigned int timestamp = 0;
	// greedy choice
	const auto& find_best_act = [&inst, &timestamp](const std::vector<size_t>& candidates) {
		size_t choice = 0;
		bool   found = false;
		for (const auto& act_i : candidates) {
			if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp)
				return std::pair(true, act_i);
			if (inst.act_f[act_i]) {
				choice = act_i;
				found = true;
			}
		}
		if (found)
			return std::pair(true, choice);
		return std::pair(true, candidates[rand() % candidates.size()]);
	};

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	binary_set state(inst.n); // initial state is empty

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (const auto& act_i : inst.act_rem)
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
		timestamp++;

		if (CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static void randr(hplus::instance& inst, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running randr algorithm.");
	// TODO: (maybe) Choose on CLI
	constexpr size_t repetitions = 100;
	// TODO: Threads (if using threads, make hplus::update_sol thread safe.)
	for (size_t _ = 0; _ < repetitions; _++) {
		randheur(inst, env, log);
		if (env.sol_s == solution_status::INFEAS) [[unlikely]]
			return;
	}
}

[[nodiscard]]
static double evaluate_htype_state(const std::vector<size_t>& state, const std::vector<double>& values, double (*h_eqtype)(double, double)) {
	double state_hcost = 0;
	for (const auto& p : state)
		state_hcost = h_eqtype(state_hcost, values[p]);
	return state_hcost;
}

[[nodiscard]]
static double evaluate_htype_state(const binary_set& state, const std::vector<double>& values, double (*h_eqtype)(double, double)) {
	double state_hcost = 0;
	for (const auto& p : state)
		state_hcost = h_eqtype(state_hcost, values[p]);
	return state_hcost;
}

static void update_htype_values(const hplus::instance& inst, const binary_set& state, std::vector<double>& values, priority_queue<double>& pq, const binary_set& used_actions, double (*h_eqtype)(double, double)) {
	for (const auto& p : state) {
		values[p] = 0;
		if (!pq.has(p))
			pq.push(p, 0);
		else
			pq.change(p, 0);
	}

	while (!pq.empty()) {
		const auto& p = pq.top();
		pq.pop();
		for (const auto& act_i : inst.act_with_pre[p]) {
			if (used_actions[act_i])
				continue;

			const auto& cost_pre = evaluate_htype_state(inst.actions[act_i].pre_sparse, values, h_eqtype);
			if (cost_pre >= std::numeric_limits<double>::infinity())
				continue;

			const auto& new_cost = cost_pre + static_cast<double>(inst.actions[act_i].cost);
			for (const auto& p_eff : inst.actions[act_i].eff_sparse) {
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

static void init_htype(const hplus::instance& inst, const bs_searcher& feasible_actions, std::vector<double>& values, priority_queue<double>& pq, double (*h_eqtype)(double, double)) {
	const auto& initial_actions = feasible_actions.find_subsets(binary_set(inst.n));
	for (const auto& act_i : initial_actions) {
		// preconditions of these variables are already met -> hmax/hadd (act.pre) = 0 -> need only the cost of the action to set the hmax/hadd of the effects
		const auto& cost = static_cast<double>(inst.actions[act_i].cost);
		for (const auto& p : inst.actions[act_i].eff_sparse) {
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
static bool htype(const hplus::instance& inst, hplus::solution& sol, double (*h_eqtype)(double, double)) {

	unsigned int timestamp = 0;
	// greedy choice
	const auto& find_best_act = [&inst, &timestamp](const std::vector<size_t>& candidates, const std::vector<double>& values, const binary_set& state) {
		size_t choice = 0;
		auto   found = false;
		auto   best_cxwe = std::numeric_limits<double>::infinity();
		for (const auto& act_i : candidates) {
			if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp)
				return std::pair(true, act_i);
			if (inst.act_f[act_i]) {
				choice = act_i;
				best_cxwe = -1;
				found = true;
				continue;
			}

			if (best_cxwe < 0)
				continue;

			unsigned int weighted_neff = 0, neff = 0;
			for (const auto& var_i : inst.actions[act_i].eff_sparse) {
				if (state[var_i])
					continue;

				weighted_neff += static_cast<unsigned int>(values[var_i]);
				neff++;
			}

			if (neff == 0) // if no new effects, this action won't change the state... if no action changes the state this problem is infeasible
				continue;

			auto cxwe = std::numeric_limits<double>::infinity();
			// if weighted_neff is 0 this means that all effects can be archieved with 0 cost actions
			// if this action is not a 0 cost one we don't want this action, otherwise we incourage it
			if (weighted_neff != 0)
				cxwe = static_cast<double>(inst.actions[act_i].cost) / weighted_neff;
			else if (inst.actions[act_i].cost == 0)
				cxwe = 0;

			// if all actions have weighted_neff at 0, there must be at least one 0 action cost that we can use, hence the best_cxwe is always updated
			if (cxwe >= best_cxwe)
				continue;

			choice = act_i;
			best_cxwe = cxwe;
			found = true;
		}
		return std::pair(found, choice);
	};

	// Reset solution (just to be sure)
	sol.plan.clear();
	sol.plan.reserve(inst.m_opt);
	sol.cost = 0;

	// Initialize helpers
	std::vector<double>	   values(inst.n, std::numeric_limits<double>::infinity());
	binary_set			   state(inst.n), used_actions(inst.m);
	priority_queue<double> pq(inst.n);

	const auto& find_best_act_v2 = [&inst, &h_eqtype, &used_actions, &pq, timestamp](const std::vector<size_t>& candidates, const std::vector<double>& values, const binary_set& state) {
		size_t choice = 0;
		bool   found = false;
		auto   best_goal_cost = std::numeric_limits<double>::infinity();
		for (const auto& act_i : candidates) {
			if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp)
				return std::pair(true, act_i);
			if (inst.act_f[act_i]) {
				choice = act_i;
				best_goal_cost = -1;
				found = true;
				continue;
			}

			if (best_goal_cost < 0)
				continue;

			std::vector<double> values_copy{ values };
			update_htype_values(inst, inst.actions[act_i].eff - state, values_copy, pq, used_actions, h_eqtype);
			const auto& goal_cost = evaluate_htype_state(inst.goal, values_copy, h_eqtype);

			if (goal_cost >= best_goal_cost)
				continue;

			choice = act_i;
			best_goal_cost = goal_cost;
			found = true;
		}
		return std::pair(found, choice);
	};

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (const auto& act_i : inst.act_rem)
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
		timestamp++;

		if (CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	return true;
}

static void hmax(hplus::instance& inst, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running hmax algorithm.");

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	if (!htype(inst, heur_sol, [](const double a, const double b) { return (a > b ? a : b); })) {
		env.sol_s = solution_status::INFEAS;
		return;
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

static void hadd(hplus::instance& inst, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running hadd algorithm.");

	hplus::solution heur_sol;
	heur_sol.plan.reserve(inst.m_opt);
	heur_sol.cost = 0;

	if (!htype(inst, heur_sol, [](const double a, const double b) { return a + b; })) {
		env.sol_s = solution_status::INFEAS;
		return;
	}

	hplus::update_sol(inst, heur_sol, log);
	env.sol_s = solution_status::FEAS;
}

[[nodiscard]]
static bool random_walk(const hplus::instance& inst, hplus::solution& sol, const double* plan, const logger& log) {
	PRINT_VERBOSE(log, "Running random walk algorithm.");
	binary_set state(inst.n);

	// to be sure
	sol.plan.reserve(inst.m_opt);
	sol.cost = 0;

	unsigned int timestamp = 0;
	// greedy choice
	const auto& find_best_act = [&inst, &plan, &timestamp](const std::vector<size_t>& candidates) {
		double weight_sum = 0;
		size_t choice = 0;
		bool   found = false;
		for (const auto& act_i : candidates) {
			if (inst.act_t[act_i] >= 0 && static_cast<unsigned int>(inst.act_t[act_i]) == timestamp)
				return std::pair(true, act_i);
			if (inst.act_f[act_i]) {
				choice = act_i;
				found = true;
				continue;
			}
			weight_sum += plan[inst.act_cpxtoidx[act_i]];
		}
		if (found)
			return std::pair(true, choice);

		if (weight_sum <= 0)
			return std::pair(true, candidates[rand() % candidates.size()]);

		std::random_device				 rd;
		std::mt19937					 gen(rd());
		std::uniform_real_distribution<> dis(0.0, weight_sum);
		const double					 random_value = dis(gen);
		double							 cumulative_weight = 0;
		for (const auto& x : candidates) {
			cumulative_weight += plan[inst.act_cpxtoidx[x]];
			if (random_value <= cumulative_weight)
				return std::pair(true, x);
		}

		return std::pair(true, candidates.back());
	};

	// binary_set searcher for faster actions lookup
	bs_searcher feasible_actions = bs_searcher(inst.n);
	for (const auto& act_i : inst.act_rem)
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
		timestamp++;

		if (CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	}

	return true;
}

static void relax(hplus::instance& inst, hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running relax algorithm (linear relaxation of Imai's model).");

	// build cplex model
	CPXENVptr cpxenv = nullptr;
	CPXLPptr  cpxlp = nullptr;
	cpx_init(cpxenv, cpxlp, env, log, false);
	cpx_build_imai(cpxenv, cpxlp, inst, env, log, true);

	// change bounds of variables
	int*  indices = new int[inst.n_opt + inst.m_opt + inst.n_opt * inst.m_opt];
	char* ctypes = new char[inst.n_opt + inst.m_opt + inst.n_opt * inst.m_opt];
	for (size_t i = 0; i < inst.m_opt; i++) {
		indices[i] = static_cast<int>(i);
		ctypes[i] = 'C';
	}
	for (size_t i = 0; i < inst.n_opt; i++) {
		indices[inst.m_opt + i] = static_cast<int>(2 * inst.m_opt + i);
		ctypes[inst.m_opt + i] = 'C';
	}
	for (size_t i = 0; i < inst.m_opt * inst.n_opt; i++) {
		indices[inst.m_opt + inst.n_opt + i] = static_cast<int>(2 * inst.m_opt + 2 * inst.n_opt + i);
		ctypes[inst.m_opt + inst.n_opt + i] = 'C';
	}
	ASSERT_LOG(log, !CPXchgctype(cpxenv, cpxlp, inst.n_opt + inst.m_opt + inst.n_opt * inst.m_opt, indices, ctypes));
	delete[] ctypes;
	ctypes = nullptr;
	delete[] indices;
	indices = nullptr;

	// solve model
	ASSERT_LOG(log, !CPXmipopt(cpxenv, cpxlp));

	// parse cplex status to see what it found
	switch (auto cpxstatus = CPXgetstat(cpxenv, cpxlp)) {
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
	ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt - 1));
	// TODO: Rethink how we reconstruct our solution... this one isn't a good approximate solution
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

static void localsearch(hplus::instance& inst, void (*heuristic)(hplus::instance&, hplus::environment&, const logger&), hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Running local-search algorithm.");
	heuristic(inst, env, log);

	if (env.sol_s == solution_status::INFEAS) [[unlikely]]
		return;

	hplus::solution heur_sol{ .plan = inst.best_sol.plan, .cost = inst.best_sol.cost };

	// TODO: Local search on top of the initial solution
	todo(log, "Local Search");

	hplus::update_sol(inst, heur_sol, log);
}

void find_heuristic(hplus::instance& inst, hplus::environment& env, const logger& log) {
	srand(time(nullptr));
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
		PRINT_VERBOSE(log, "Building imai's model.");

	const auto& stopchk1 = []() {
		if (CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	};

	// ====================================================== //
	// ============== TIGHTER TIMESTAMPS BOUNDS ============= //
	// ====================================================== //

	auto timestamps_ubound = static_cast<unsigned int>(inst.m_opt);
	if (env.imai_tight_bounds) {
		// number of variables
		if (inst.n_opt < timestamps_ubound)
			timestamps_ubound = static_cast<unsigned int>(inst.n_opt);

		// max number of steps to reach heuristic
		if (env.heur != "none" && !relaxed) {
			auto		 min_act_cost = inst.actions[0].cost + 1; // +1 to avoid it being 0
			unsigned int n_act_zerocost = 0;
			for (const auto& act : inst.actions) {
				if (act.cost == 0)
					n_act_zerocost++;
				else if (act.cost < min_act_cost) [[unlikely]]
					min_act_cost = act.cost;
			}
			const auto& nsteps = inst.best_sol.cost / min_act_cost + n_act_zerocost;
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

	const auto& stopchk2 = [&objs, &lbs, &ubs, &types]() {
		if (CHECK_STOP()) [[unlikely]] {
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
	const auto act_start = curr_col;
	size_t	   count = 0;
	for (const auto& act_i : inst.act_rem) {
		objs[count] = static_cast<double>(inst.actions[act_i].cost);
		lbs[count] = (inst.act_f[act_i] ? 1 : 0);
		ubs[count] = 1;
		types[count++] = 'B';
	}
	INTCHECK_ASSERT_LOG(log, count == inst.m_opt);
	curr_col += inst.m_opt;

	ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// --- action timestamps -- //
	const auto tact_start = curr_col;
	count = 0;
	for (const auto& act_i : inst.act_rem) {
		objs[count] = 0;
		lbs[count] = (inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : 0);
		ubs[count] = (inst.act_t[act_i] >= 0 ? inst.act_t[act_i] : timestamps_ubound - 1);
		types[count++] = 'I';
	}
	INTCHECK_ASSERT_LOG(log, count == inst.m_opt);
	curr_col += inst.m_opt;

	ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	resize_cpx_arrays(inst.n_opt);

	// ------- variables ------ //
	const auto var_start = curr_col;
	count = 0;
	for (const auto& var_i : inst.var_rem) {
		objs[count] = 0;
		lbs[count] = ((inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0);
		ubs[count] = 1;
		types[count++] = 'B';
	}
	INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
	curr_col += inst.n_opt;

	ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// -- variable timestamps - //
	const auto tvar_start = curr_col;
	count = 0;
	for (const auto& i : inst.var_rem) {
		objs[count] = 0;
		lbs[count] = (inst.var_t[i] >= 0 ? inst.var_t[i] : 1); // 1 since with no initial state, each variable needs to be archieved first
		ubs[count] = (inst.var_t[i] >= 0 ? inst.var_t[i] : timestamps_ubound);
		types[count++] = 'I';
	}
	INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
	curr_col += inst.n_opt;

	ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// --- first archievers --- //
	const auto fa_start = curr_col;
	count = 0;
	for (const auto& act_i : inst.act_rem) {
		size_t count_var = 0;
		for (const auto& var_i : inst.var_rem) {
			objs[count_var] = 0;
			lbs[count_var] = (inst.fadd_f[act_i][var_i] ? 1 : 0);
			ubs[count_var] = ((!inst.actions[act_i].eff[var_i] || inst.fadd_e[act_i][var_i]) ? 0 : 1);
			types[count_var++] = 'B';
		}
		INTCHECK_ASSERT_LOG(log, count_var == inst.n_opt);
		curr_col += inst.n_opt;
		ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
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
	const auto& get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
	const auto& get_tact_idx = [&inst, &tact_start](size_t idx) { return static_cast<int>(tact_start + inst.act_opt_conv[idx]); };
	const auto& get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
	const auto& get_tvar_idx = [&inst, &tvar_start](size_t idx) { return static_cast<int>(tvar_start + inst.var_opt_conv[idx]); };
	const auto& get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_idx) { return static_cast<int>(fa_start + inst.act_opt_conv[act_idx] * inst.n_opt + inst.var_opt_conv[var_idx]); };

	int*		   ind_c1 = new int[inst.m_opt + 1];
	double*		   val_c1 = new double[inst.m_opt + 1];
	int			   ind_c2_4[2], ind_c5[3];
	double		   val_c2_4[2], val_c5[3];
	constexpr auto sensel = 'L', sensee = 'E';
	const double   rhs_c1_2_4 = 0, rhs_c5 = timestamps_ubound;
	constexpr auto begin = 0;

	std::vector<int*>	 ind_c3(inst.n_opt);
	std::vector<double*> val_c3(inst.n_opt);
	std::vector<int>	 nnz_c3(inst.n_opt);
	std::vector<double>	 rhs_c3(inst.n_opt);

	const auto& stopchk3 = [&inst, &ind_c1, &val_c1, &ind_c3, &val_c3]() {
		if (CHECK_STOP()) [[unlikely]] {
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

	for (const auto& i : inst.var_rem) {
		ind_c3[inst.var_opt_conv[i]] = new int[inst.m_opt + 1];
		val_c3[inst.var_opt_conv[i]] = new double[inst.m_opt + 1];
		nnz_c3[inst.var_opt_conv[i]] = 0;
		rhs_c3[inst.var_opt_conv[i]] = 0;
		ind_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = get_var_idx(i);
		val_c3[inst.var_opt_conv[i]][nnz_c3[inst.var_opt_conv[i]]] = 1;
		nnz_c3[inst.var_opt_conv[i]]++;
	}
	stopchk3();

	for (const auto& act_i : inst.act_rem) {
		const auto&	   inverse_actions = inst.act_inv[act_i];
		constexpr auto nnz_c2_4 = 2;
		for (const auto& var_i : inst.actions[act_i].pre_sparse) {
			// constraint 1: x_a + sum_{inv(a, p)}(z_a'vj) <= y_vj, vj in pre(a)
			ind_c1[0] = get_act_idx(act_i);
			val_c1[0] = 1;
			ind_c1[1] = get_var_idx(var_i);
			val_c1[1] = -1;
			auto nnz0 = 2;
			// (section 4.6 of Imai's paper)
			for (const auto& inverse_action : inverse_actions) {
				if (inst.actions[inverse_action].eff[var_i]) {
					ind_c1[nnz0] = get_fa_idx(inverse_action, var_i);
					val_c1[nnz0++] = 1;
				}
			}
			ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz0, &rhs_c1_2_4, &sensel, &begin, ind_c1, val_c1, nullptr, nullptr));
			// constraint 4: t_vj <= t_a, vj in pre(a)
			ind_c2_4[0] = get_tvar_idx(var_i);
			val_c2_4[0] = 1;
			ind_c2_4[1] = get_tact_idx(act_i);
			val_c2_4[1] = -1;
			ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
		}
		for (const auto& var_i : inst.actions[act_i].eff_sparse) {
			constexpr auto nnz_c5 = 3;
			// constraint 2: z_avj <= x_a, vj in eff(a)
			ind_c2_4[0] = get_fa_idx(act_i, var_i);
			val_c2_4[0] = 1;
			ind_c2_4[1] = get_act_idx(act_i);
			val_c2_4[1] = -1;
			ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c2_4, &rhs_c1_2_4, &sensel, &begin, ind_c2_4, val_c2_4, nullptr, nullptr));
			// constraint 5: t_a + 1 <= t_vj + (|A|+1)(1-z_avj), vj in eff(a)
			ind_c5[0] = get_tact_idx(act_i);
			val_c5[0] = 1;
			ind_c5[1] = get_tvar_idx(var_i);
			val_c5[1] = -1;
			ind_c5[2] = get_fa_idx(act_i, var_i);
			val_c5[2] = timestamps_ubound + 1;
			ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c5, &rhs_c5, &sensel, &begin, ind_c5, val_c5, nullptr, nullptr));
			// constraint 3: I(v_j) + sum(z_avj) = y_vj
			ind_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = get_fa_idx(act_i, var_i);
			val_c3[inst.var_opt_conv[var_i]][nnz_c3[inst.var_opt_conv[var_i]]] = -1;
			nnz_c3[inst.var_opt_conv[var_i]]++;
		}
		stopchk3();
	}

	for (size_t var_i = 0; var_i < inst.n_opt; var_i++)
		ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz_c3[var_i], &rhs_c3[var_i], &sensee, &begin, ind_c3[var_i], val_c3[var_i], nullptr, nullptr));

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

	// ASSERT_LOG(log, !CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

void cpx_post_warmstart_imai(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	INTCHECK_ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

	binary_set state(inst.n);

	const auto& warm_start = inst.best_sol.plan;

	const auto& ncols = static_cast<size_t>(CPXgetnumcols(cpxenv, cpxlp));
	int*		cpx_sol_ind = new int[ncols];
	double*		cpx_sol_val = new double[ncols];

	constexpr auto izero = 0;
	constexpr auto effortlevel = CPX_MIPSTART_AUTO; // FIXME: Why can't I use NOCHECK???
	size_t		   nnz = 0;
	unsigned int   timestamp = 0;

#if HPLUS_INTCHECK
	binary_set				fixed_act_check{ inst.act_f }, fixed_var_check{ inst.var_f };
	std::vector<binary_set> fixed_fadd_check(inst.m);
	for (size_t i = 0; i < inst.m; i++)
		fixed_fadd_check[i] = inst.fadd_f[i];
#endif

	for (const auto& act_i : warm_start) {
		INTCHECK_ASSERT_LOG(log, !(inst.act_t[act_i] >= 0 && timestamp != static_cast<unsigned int>(inst.act_t[act_i])));
		INTCHECK_ASSERT_LOG(log, !inst.act_e[act_i]);
#if HPLUS_INTCHECK
		fixed_act_check.remove(act_i);
#endif
		cpx_sol_ind[nnz] = static_cast<int>(inst.act_opt_conv[act_i]);
		cpx_sol_val[nnz++] = 1;
		cpx_sol_ind[nnz] = static_cast<int>(inst.m_opt + inst.act_opt_conv[act_i]);
		cpx_sol_val[nnz++] = static_cast<int>(timestamp);
		timestamp++;
		for (const auto& var_i : inst.actions[act_i].eff_sparse) {
			if (state[var_i])
				continue;

			INTCHECK_ASSERT_LOG(log, !(inst.var_t[var_i] >= 0 && timestamp != static_cast<unsigned int>(inst.var_t[var_i])));
			INTCHECK_ASSERT_LOG(log, !inst.var_e[var_i]);
			INTCHECK_ASSERT_LOG(log, !inst.fadd_e[act_i][var_i]);
#if HPLUS_INTCHECK
			fixed_var_check.remove(var_i);
			fixed_fadd_check[act_i].remove(var_i);
#endif

			cpx_sol_ind[nnz] = static_cast<int>(2 * inst.m_opt + inst.var_opt_conv[var_i]);
			cpx_sol_val[nnz++] = 1;
			cpx_sol_ind[nnz] = static_cast<int>(2 * inst.m_opt + inst.n_opt + inst.var_opt_conv[var_i]);
			cpx_sol_val[nnz++] = static_cast<int>(timestamp);
			cpx_sol_ind[nnz] = static_cast<int>(2 * inst.m_opt + 2 * inst.n_opt + inst.act_opt_conv[act_i] * inst.n_opt + inst.var_opt_conv[var_i]);
			cpx_sol_val[nnz++] = 1;
		}
		state |= inst.actions[act_i].eff;
	}

#if HPLUS_INTCHECK
	ASSERT_LOG(log, fixed_act_check.empty());
	ASSERT_LOG(log, fixed_var_check.empty());
	for (size_t i = 0; i < inst.m; i++)
		ASSERT_LOG(log, fixed_fadd_check[i].empty());
#endif
	ASSERT_LOG(log, !CPXaddmipstarts(cpxenv, cpxlp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));

	delete[] cpx_sol_ind;
	cpx_sol_ind = nullptr;
	delete[] cpx_sol_val;
	cpx_sol_val = nullptr;
}

void store_imai_sol(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
	PRINT_VERBOSE(log, "Storing imai's solution.");

	// get cplex result (interested only in the sequence of actions [0/inst.m_opt-1]
	// used and its ordering [inst.m_opt/2nact-1])
	double* plan = new double[2 * inst.m_opt];
	ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, 2 * inst.m_opt - 1));

	// convert to std collections for easier parsing
	std::vector<std::pair<double, size_t>> cpx_result;
	for (size_t i = 0; i < inst.m_opt; i++) {
		if (plan[i] > HPLUS_CPX_INT_ROUNDING)
			cpx_result.emplace_back(plan[inst.m_opt + i], i);
	}
	delete[] plan;
	plan = nullptr;

	// sort cpx_result based on actions timestamps
	std::ranges::sort(cpx_result.begin(), cpx_result.end(), [](const std::pair<double, size_t>& x, const std::pair<double, size_t>& y) { return x.first < y.first; });

	// get solution from sorted cpx_result
	std::vector<size_t> solution;
	std::ranges::transform(cpx_result.begin(), cpx_result.end(), std::back_inserter(solution), [inst](const std::pair<double, size_t>& p) { return inst.act_cpxtoidx[p.second]; });

	// store solution
	hplus::solution imai_sol{ solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [inst](const size_t acc, const size_t index) { return acc + inst.actions[index].cost; })) };
	hplus::update_sol(inst, imai_sol, log);
}

// ##################################################################### //
// ############################## RANKOOH ############################## //
// ##################################################################### //

void cpx_build_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Building rankooh's model.");

	const auto& stopchk1 = []() {
		if (CHECK_STOP()) [[unlikely]]
			throw timelimit_exception("Reached time limit.");
	};

	// ====================================================== //
	// ================= VERTEX ELIMINATION ================= //
	// ====================================================== //

	struct node {
		size_t id, deg;
		node(const size_t id, const size_t deg)
			: id(id), deg(deg) {}
	};
	struct compare_node {
		bool operator()(const node& n1, const node& n2) const { return n1.deg > n2.deg; }
	};
	struct triangle {
		size_t first, second, third;
		triangle(const size_t first, const size_t second, const size_t third)
			: first(first), second(second), third(third) {}
	};

	std::vector<std::set<size_t>> graph(inst.n);
	std::vector<binary_set>		  cumulative_graph(inst.n, binary_set(inst.n));
	std::vector<triangle>		  triangles_list;

	// TODO: (maybe) Implement using pq.hxx
	std::priority_queue<node, std::vector<node>, compare_node> nodes_queue;
	std::vector<size_t>										   degree_counter(inst.n, 0);

	// G_0
	for (const auto& act_i : inst.act_rem) {
		for (const auto& var_i : inst.actions[act_i].pre_sparse) {
			for (const auto& var_j : inst.actions[act_i].eff_sparse) {
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
	const auto& find_min = [&degree_counter](std::priority_queue<node, std::vector<node>, compare_node>& nodes_queue) {
		auto idx = -1;
		while (!nodes_queue.empty() && idx < 0) {
			const auto& tmp = nodes_queue.top();
			nodes_queue.pop();
			if (degree_counter[tmp.id] == tmp.deg)
				idx = static_cast<int>(tmp.id);
		}
		return idx;
	};

	// G_i (min degree heuristics)
	for (const auto& _ : inst.var_rem) {
		auto idx = find_min(nodes_queue);
		if (idx == -1) [[unlikely]]
			break;

		// graph structure:
		// | \       > |
		// p -> idx -> q
		// | /       > |

		std::set<size_t> new_nodes;

		for (const auto& p : inst.var_rem) {
			if (!graph[p].contains(idx))
				continue;

			for (const auto& q : graph[idx]) {
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
			for (const auto& q : graph[idx]) {
				if (p != q) [[likely]]
					triangles_list.emplace_back(p, idx, q);
			}
		}

		// remove the edge idx - q
		for (const auto& q : graph[idx]) {
			degree_counter[q] -= 1;
			new_nodes.insert(q);
		}
		graph[idx].clear();
		degree_counter[idx] = 0;

		// Update the priority queue
		for (const auto& x : new_nodes) {
			if (degree_counter[x] > 0)
				nodes_queue.emplace(x, degree_counter[x]);
		}

#if HPLUS_INTCHECK // care: this takes HUGE amount of time
		for (size_t node_i = 0; node_i < inst.n; node_i++) {
			size_t i_cnt = 0;
			i_cnt += graph[node_i].size();
			for (size_t tmp_j = 0; tmp_j < inst.n; tmp_j++) {
				for (const auto& tmp_k : graph[tmp_j]) {
					if (tmp_k == node_i)
						i_cnt += 1;
				}
			}
			ASSERT_LOG(log, i_cnt == degree_counter[node_i]);
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

	const auto& resize_cpx_arrays = [&objs, &lbs, &ubs, &types](size_t new_size) {
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

	const auto& stopchk2 = [&objs, &lbs, &ubs, &types]() {
		if (CHECK_STOP()) [[unlikely]] {
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
	const auto act_start = curr_col;
	size_t	   count = 0;
	for (const auto& act_i : inst.act_rem) {
		objs[count] = static_cast<double>(inst.actions[act_i].cost);
		lbs[count] = inst.act_f[act_i] ? 1 : 0;
		ubs[count] = 1;
		types[count++] = 'B';
	}
	INTCHECK_ASSERT_LOG(log, count == inst.m_opt);

	curr_col += inst.m_opt;

	ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.m_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	resize_cpx_arrays(inst.n_opt);

	// --- first archievers --- //
	const auto			fa_start = curr_col;
	std::vector<size_t> fa_individual_start(inst.m_opt);
	count = 0;
	for (const auto& act_i : inst.act_rem) {
		fa_individual_start[count] = count * inst.n_opt;
		size_t count_var = 0;
		for (const auto& var_i : inst.var_rem) {
			objs[count_var] = 0;
			lbs[count_var] = inst.fadd_f[act_i][var_i] ? 1 : 0;
			ubs[count_var] = (!inst.actions[act_i].eff[var_i] || inst.fadd_e[act_i][var_i]) ? 0 : 1;
			types[count_var++] = 'B';
		}
		INTCHECK_ASSERT_LOG(log, count_var == inst.n_opt);
		curr_col += inst.n_opt;
		ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
		count++;
		stopchk2();
	}

	// ------- variables ------ //
	const auto var_start = curr_col;
	count = 0;
	for (const auto& var_i : inst.var_rem) {
		objs[count] = 0;
		lbs[count] = (inst.var_f[var_i] || inst.goal[var_i]) ? 1 : 0;
		ubs[count] = 1;
		types[count++] = 'B';
	}

	INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
	curr_col += inst.n_opt;

	ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
	stopchk2();

	// vertex elimination graph edges
	const auto veg_edges_start = curr_col;
	for (const auto& var_i : inst.var_rem) {
		count = 0;
		for (const auto& var_j : inst.var_rem) {
			objs[count] = 0;
			lbs[count] = 0;
			ubs[count] = cumulative_graph[var_i][var_j] ? 1 : 0;
			types[count++] = 'B';
		}
		INTCHECK_ASSERT_LOG(log, count == inst.n_opt);
		ASSERT_LOG(log, !CPXnewcols(cpxenv, cpxlp, inst.n_opt, objs, lbs, ubs, types, nullptr));
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
	const auto& get_act_idx = [&inst, &act_start](size_t idx) { return static_cast<int>(act_start + inst.act_opt_conv[idx]); };
	const auto& get_var_idx = [&inst, &var_start](size_t idx) { return static_cast<int>(var_start + inst.var_opt_conv[idx]); };
	const auto& get_fa_idx = [&inst, &fa_start](size_t act_idx, size_t var_idx) { return static_cast<int>(fa_start + inst.act_opt_conv[act_idx] * inst.n_opt + inst.var_opt_conv[var_idx]); };
	const auto& get_veg_idx = [&inst, &veg_edges_start](size_t idx_i, size_t idx_j) { return static_cast<int>(veg_edges_start + inst.var_opt_conv[idx_i] * inst.n_opt + inst.var_opt_conv[idx_j]); };

	int*			 ind = new int[inst.m_opt + 1];
	double*			 val = new double[inst.m_opt + 1];
	int				 nnz = 0;
	constexpr char	 sense_e = 'E', sense_l = 'L';
	constexpr double rhs_0 = 0, rhs_1 = 1;
	constexpr int	 begin = 0;

	const auto& stopchk3 = [&ind, &val]() {
		if (CHECK_STOP()) [[unlikely]] {
			delete[] val;
			val = nullptr;
			delete[] ind;
			ind = nullptr;
			throw timelimit_exception("Reached time limit.");
		}
	};

	for (const auto& var_i : inst.var_rem) {
		nnz = 0;
		ind[nnz] = get_var_idx(var_i);
		val[nnz++] = 1;

		for (const auto& act_i : inst.act_with_eff[var_i]) {
			ind[nnz] = get_fa_idx(act_i, var_i);
			val[nnz++] = -1;
		}

		ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind, val, nullptr, nullptr));
		stopchk3();
	}

	for (const auto& var_i : inst.var_rem) {
		for (const auto& var_j : inst.var_rem) {
			nnz = 0;
			ind[nnz] = get_var_idx(var_j);
			val[nnz++] = -1;
			for (const auto& act_i : inst.act_with_eff[var_i]) {
				if (inst.actions[act_i].pre[var_j]) [[unlikely]] {
					ind[nnz] = get_fa_idx(act_i, var_i);
					val[nnz++] = 1;
				}
			}
			ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind, val, nullptr, nullptr));
			stopchk3();
		}
	}

	delete[] val;
	val = nullptr;
	delete[] ind;
	ind = nullptr;

	int	   ind_c5_c6_c7[2], ind_c8[3];
	double val_c5_c6_c7[2], val_c8[3];

	for (const auto& act_i : inst.act_rem) {
		for (const auto& var_i : inst.actions[act_i].eff_sparse) {
			ind_c5_c6_c7[0] = get_act_idx(act_i);
			val_c5_c6_c7[0] = -1;
			ind_c5_c6_c7[1] = get_fa_idx(act_i, var_i);
			val_c5_c6_c7[1] = 1;
			ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
		}
		stopchk1();
	}

	for (const auto& act_i : inst.act_rem) {
		for (const auto& var_i : inst.actions[act_i].pre_sparse) {
			for (const auto& var_j : inst.actions[act_i].eff_sparse) {
				ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
				val_c5_c6_c7[0] = -1;
				ind_c5_c6_c7[1] = get_fa_idx(act_i, var_j);
				val_c5_c6_c7[1] = 1;
				ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
			}
			stopchk1();
		}
	}

	for (const auto& var_i : inst.var_rem) {
		for (const auto& var_j : cumulative_graph[var_i]) {
			ind_c5_c6_c7[0] = get_veg_idx(var_i, var_j);
			val_c5_c6_c7[0] = 1;
			ind_c5_c6_c7[1] = get_veg_idx(var_j, var_i);
			val_c5_c6_c7[1] = 1;
			ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 2, &rhs_1, &sense_l, &begin, ind_c5_c6_c7, val_c5_c6_c7, nullptr, nullptr));
			stopchk1();
		}
	}

	for (const auto& [a, b, c] : triangles_list) {
		ind_c8[0] = get_veg_idx(a, b);
		val_c8[0] = 1;
		ind_c8[1] = get_veg_idx(b, c);
		val_c8[1] = 1;
		ind_c8[2] = get_veg_idx(a, c);
		val_c8[2] = -1;
		ASSERT_LOG(log, !CPXaddrows(cpxenv, cpxlp, 0, 1, 3, &rhs_1, &sense_l, &begin, ind_c8, val_c8, nullptr, nullptr));
		stopchk1();
	}

	// ASSERT_LOG(log, !CPXwriteprob(cpxenv, cpxlp, (HPLUS_CPLEX_OUTPUT_DIR "/lp/" + env.run_name + ".lp").c_str(), "LP"));
}

void cpx_post_warmstart_rankooh(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	INTCHECK_ASSERT_LOG(log, env.sol_s != solution_status::INFEAS && env.sol_s != solution_status::NOTFOUND);

	binary_set state(inst.n);

	const auto& warm_start = inst.best_sol.plan;

	const auto& ncols = static_cast<size_t>(CPXgetnumcols(cpxenv, cpxlp));
	int*		cpx_sol_ind = new int[ncols];
	double*		cpx_sol_val = new double[ncols];

	constexpr auto izero = 0;
	constexpr auto effortlevel = CPX_MIPSTART_AUTO;
	size_t		   nnz = 0;

#if HPLUS_INTCHECK
	unsigned int			timestamp = 0;
	binary_set				fixed_act_check{ inst.act_f }, fixed_var_check{ inst.var_f };
	std::vector<binary_set> fixed_fadd_check(inst.m);
	for (size_t i = 0; i < inst.m; i++)
		fixed_fadd_check[i] = inst.fadd_f[i];
#endif

	for (const auto& act_i : warm_start) {
#if HPLUS_INTCHECK
		ASSERT_LOG(log, !(inst.act_t[act_i] >= 0 && timestamp != static_cast<unsigned int>(inst.act_t[act_i])));
		ASSERT_LOG(log, !inst.act_e[act_i]);
		fixed_act_check.remove(act_i);
		timestamp++;
#endif
		// set the action indicator variable
		cpx_sol_ind[nnz] = static_cast<int>(inst.act_opt_conv[act_i]);
		cpx_sol_val[nnz++] = 1;
		for (const auto& var_i : inst.actions[act_i].eff_sparse) {
			if (state[var_i])
				continue;

#if HPLUS_INTCHECK
			ASSERT_LOG(log, !(inst.var_t[var_i] >= 0 && timestamp != static_cast<unsigned int>(inst.var_t[var_i])));
			ASSERT_LOG(log, !inst.var_e[var_i]);
			ASSERT_LOG(log, !inst.fadd_e[act_i][var_i]);
			fixed_var_check.remove(var_i);
			fixed_fadd_check[act_i].remove(var_i);
#endif

			// set the fact indicator variable
			cpx_sol_ind[nnz] = static_cast<int>(inst.m_opt + inst.m_opt * inst.n_opt + inst.var_opt_conv[var_i]);
			cpx_sol_val[nnz++] = 1;
			// set the fadd indicator variable
			cpx_sol_ind[nnz] = static_cast<int>(inst.m_opt + inst.act_opt_conv[act_i] * inst.n_opt + inst.var_opt_conv[var_i]);
			cpx_sol_val[nnz++] = 1;
		}
		state |= inst.actions[act_i].eff;
	}

#if HPLUS_INTCHECK
	ASSERT_LOG(log, fixed_act_check.empty());
	ASSERT_LOG(log, fixed_var_check.empty());
	for (size_t i = 0; i < inst.m; i++)
		ASSERT_LOG(log, fixed_fadd_check[i].empty());
#endif

	ASSERT_LOG(log, !CPXaddmipstarts(cpxenv, cpxlp, 1, nnz, &izero, cpx_sol_ind, cpx_sol_val, &effortlevel, nullptr));
	delete[] cpx_sol_ind;
	cpx_sol_ind = nullptr;
	delete[] cpx_sol_val;
	cpx_sol_val = nullptr;
}

void store_rankooh_sol(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
	PRINT_VERBOSE(log, "Storing rankooh's solution.");

	double* plan = new double[inst.m_opt + inst.m_opt * inst.n_opt];
	ASSERT_LOG(log, !CPXgetx(cpxenv, cpxlp, plan, 0, inst.m_opt + inst.m_opt * inst.n_opt - 1));

	// fixing the solution to read the plan (some actions are set to 1 even if
	// they are not a first archiever of anything)
	for (size_t act_i_cpx = 0, fadd_i = inst.m_opt; act_i_cpx < inst.m_opt; act_i_cpx++, fadd_i += inst.n_opt) {
		bool set_zero = true;
		for (size_t var_i_cpx = 0; var_i_cpx < inst.n_opt; var_i_cpx++) {
			if (plan[fadd_i + var_i_cpx] > HPLUS_CPX_INT_ROUNDING) {
				INTCHECK_ASSERT_LOG(log, plan[act_i_cpx] > HPLUS_CPX_INT_ROUNDING);
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
		for (const auto& i : remaining) {
			if (!state.contains(inst.actions[cpx_result[i]].pre))
				continue;

			remaining.remove(i);
			state |= inst.actions[cpx_result[i]].eff;
			solution.push_back(cpx_result[i]);
#if HPLUS_INTCHECK
			intcheck = true;
#endif
		}
		INTCHECK_ASSERT_LOG(log, intcheck);
	}

	// store solution
	hplus::solution rankooh_sol{ solution, static_cast<unsigned int>(std::accumulate(solution.begin(), solution.end(), 0, [&inst](const unsigned int acc, const size_t index) { return acc + inst.actions[index].cost; })) };
	hplus::update_sol(inst, rankooh_sol, log);
}

// ##################################################################### //
// ########################### DYNAMIC SMALL ########################### //
// ##################################################################### //

void cpx_build_dynamic_small(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Building dynamic small model.");
	// TODO: Building the small dynamic model
	todo(log, "Building the small dynamic model");
}

void cpx_post_warmstart_dynamic_small(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	// TODO: Posting warmstart to the small dynamic model
	todo(log, "Posting warmstart to the small dynamic model");
}

void store_dynamic_small_sol(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
	PRINT_VERBOSE(log, "Storing the dynamic small solution.");
	// TODO: Storing solution of the small dynamic model
	todo(log, "Storing solution of the small dynamic model");
}

// ##################################################################### //
// ########################### DYNAMIC LARGE ########################### //
// ##################################################################### //

void cpx_build_dynamic_large(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	PRINT_VERBOSE(log, "Building dynamic large model.");
	// TODO: Building the large dynamic model
	todo(log, "Building the large dynamic model");
}

void cpx_post_warmstart_dynamic_large(CPXENVptr& cpxenv, CPXLPptr& cpxlp, const hplus::instance& inst, const hplus::environment& env, const logger& log) {
	// TODO: Posting warmstart to the large dynamic model
	todo(log, "Posting warmstart to the large dynamic model");
}

void store_dynamic_large_sol(const CPXENVptr& cpxenv, const CPXLPptr& cpxlp, hplus::instance& inst, const logger& log) {
	PRINT_VERBOSE(log, "Storing the dynamic large solution.");
	// TODO: Storing solution of the large dynamic model
	todo(log, "Storing solution of the large dynamic model");
}