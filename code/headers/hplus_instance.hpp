#ifndef _HPLUS_INST_H
#define _HPLUS_INST_H

#include "utils.hpp"

/**
 * Struct representing an action with preconditions, effects and cost
 */
class HPLUS_action  {

    public:

        //TODO: Edit if I'm working in STRIPS
        /**
         * Value each variable must have for the action to be taken
         * 
         * [p0, p1, ..., pn] for each variable vj, value(vj) must be pj for the action to be taken (-1 if a specific value isn't required)
         */
        int* pre;

        //TODO: Edit if I'm working in STRIPS
        /**
         * Value each variable has after the action has been taken
         * 
         * [e0, e1, ..., en] for each variable vj, value(vj) = ej after the action has been taken (-1 if the action doesn't change that variable's value)
         */
        int* eff;   //TODO: Ask: Do I need to make an add and a remove effect or just effects is ok (considering I don't have remove effects)?

        /**
         * Cost of the action
         */
        int cost;   //TODO: Ask: int or double?

        /**
         * Create a blank HPLUS_action
         */
        HPLUS_action();

        /**
         * Create an HPLUS_action from another (deep copy)
         * 
         * @param from_action: The HPLUS_action to copy
         */
        HPLUS_action(const HPLUS_action* from_action);

        /**
         * Destructor for HPLUS_action
         */
        ~HPLUS_action();

};

//TODO: Consider split in HPLUS_domain and HPLUS_problem
class HPLUS_instance {

    public:

        /**
         * Create an HPLUS_instance from a file produced by the fast downward translator
         * 
         * @param file_name: Path of the file where to read the instance from
         */
        HPLUS_instance(std::string file_path);

        /**
         * Create an HPLUS_instance from another (deep copy)
         * 
         * @param from_inst: The HPLUS_instance to copy
         */
        HPLUS_instance(const HPLUS_instance* from_inst);

        /**
         * Destructor for HPLUS_instance
         */
        ~HPLUS_instance();

        /**
         * Get the number of variables this problem has (referred as n)
         * 
         * @param nvar A pointer to the variable to save the return value into: int
         */
        const void get_nvar(int* nvar);

        //TODO: Remove if I'm working in STRIPS
        /**
         * Get the ranges of each variable
         * 
         * @param ranges_list A pointer to the variable to save the return value into: 
         * this variable must be an array of integers, already allocated, with a length >= n (only the first n values will be set)
         * ->
         * [r0, r1, ..., rn] for each variable vj, value(vj) is in [0, rj[
         */
        const void get_var_ranges(int* ranges_list);

        /**
         * Get the number of actions this problem has (referred as m)
         * 
         * @param nact A pointer to the variable to save the return the value into: int
         */
        const void get_nact(int* nact);

        /**
         * Get the list of possible actions
         * 
         * @param actions_list A pointer to the variable to save the return value into:
         * this variable must be an array of HPLUS_action, already allocated, with a length >= m (only the first m values will be set)
         */
        const void get_actions(HPLUS_action* actions_list);

        //TODO: Edit if I'm working in STRIPS
        /**
         * Get the initial state
         * 
         * @param istate: A pointer to the variable to save the return value into:
         * this variable must be an array of integers, already allocated, with a length >= n (only the first n values will be set)
         * ->
         * [i0, i1, ..., in] for each variable vj, value(vj) = ij in the initial state
         */
        const void get_istate(int* istate);

        //TODO: Edit if I'm working in STRIPS
        /**
         * Get the goal state
         * 
         * @param istate: A pointer to the variable to save the return value into:
         * this variable must be an array of integers, already allocated, with a length >= n (only the first n values will be set)
         * ->
         * [g0, g1, ..., gn] for each variable vj, value(vj) must be gj in the goal state (-1 if a specific value isn't required)
         */
        const void get_gstate(int* gstate);

        /**
         * Compare a solution found with the best found so far and stores it if it's the new best one
         * 
         * @param solution: ordered sequence of HPLUS_action that represent the solution found
         * @param nact: number of actions this solution has
         * @param cost: cost of this solution
         */
        void update_best_solution(const HPLUS_action* solution, const int* nact, const int* cost);

        /**
         * Get the best solution found so far
         * 
         * @param solution: A pointer to the variable to save the solution into:
         * this variable must be a list of HPLUS_action, already allocated, with a length >= m (only the first nact variables will be set)
         * @param nact: A pointer to the variable to save the number of actions the solution has into: int
         * @param cost: A pointer to the variable to save the cost of the solution into: int
         */
        const void get_best_solution(HPLUS_action* solution, int* nact, int* cost);

        /**
         * Get the cost of the best solution found so far
         * 
         * @param cost: A pointer to the variable to save the return value into: int
         */
        const void get_best_cost(int* cost);

    private:

        // DOMAIN

        //TODO: Probably unneccessary
        short version;

        //TODO: Probably unnecessary, maybe useful for mathematical model
        /**
         * This instance uses generic or unitary costs
         */
        bool use_costs;

        /**
         * Number of variables in the problem (n)
         */
        int n_var;

        //TODO: Remove if I'm working in STRIPS
        /**
         * Range of each variable
         * 
         * [r0, r1, ..., rn] the range for variable vj is [0, rj[
         */
        int* var_ranges;

        /**
         * Number of actions in the problem (m)
         */
        int n_act;

        /**
         * List of actions that can be done
         */
        HPLUS_action* actions;

        // PROBLEM

        //TODO: Edit if I'm working in STRIPS
        /**
         * Initial state of the problem
         * 
         * [i0, i1, ..., in] for each variable vj, value(vj) = ij in the initial state
         */
        int* initial_state;

        //TODO: Edit if I'm working in STRIPS
        /**
         * Goal state of the problem
         * 
         * [g0, g1, ..., gn] for each variable vj, value(vj) must be gj in the goal state (-1 if a specific value isn't required)
         */
        int* goal_state;

        /**
         * Best solution found so far as a ordered sequence of actions
         */
        HPLUS_action* best_solution;

        /**
         * Number of actions in the best solution found so far
         */
        int best_nact;

        /**
         * Cost of the best solution found so far
         */
        int best_cost;

        // UTILS

        void parse_inst_file(const FILE* file);

};

#endif