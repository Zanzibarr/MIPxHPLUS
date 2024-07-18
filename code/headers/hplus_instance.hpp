#ifndef _HPLUS_INST_H
#define _HPLUS_INST_H

#include "utils.hpp"
#include "logging.hpp"

/**
 * Struct representing an action with preconditions, effects and cost
 */
class HPLUS_action  {

    public:

        /**
         * Value each variable must have for the action to be taken
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) must be 0, value(v1) must be 1, value(v2) must be 3
         */
        BitField* pre;

        /**
         * Value each variable has after the action has been taken
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) will be 0, value(v1) will be 1, value(v2) will be 3
         */
        BitField* eff;

        /**
         * Cost of the action
         */
        int cost;

        /**
         * Create a blank HPLUS_action
         */
        HPLUS_action(unsigned int size_bf);

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

class HPLUS_domain {

    public:

        /**
         * Create an HPLUS_instance from a file produced by the fast downward translator
         * 
         * @param file_name: Path of the file where to read the instance from
         */
        HPLUS_domain(const std::string file_path, Logger* logger);

        /**
         * Create an HPLUS_instance from another (deep copy)
         * 
         * @param from_inst: The HPLUS_instance to copy
         */
        HPLUS_domain(const HPLUS_domain* from_dom);

        /**
         * Destructor for HPLUS_instance
         */
        ~HPLUS_domain();

        /**
         * true if unitary cost is used, false if generic cost is used
         */
        const bool unitary_cost();

        /**
         * Get the number of variables this problem has (referred as n)
         * 
         * @param nvar A pointer to the variable to save the return value into: int
         */
        const void get_nvar(int* nvar);

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

    private:

        /**
         * This instance uses generic or unitary costs
         */
        bool use_costs;

        /**
         * Number of variables in the problem (n)
         */
        int n_var;

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
        HPLUS_action** actions;

        /**
         * Logger to use for info/debugging
         */
        Logger* logger;

        // UTILS

        void parse_inst_file(const FILE* file);

};

class HPLUS_problem {

    public:

        HPLUS_problem(const HPLUS_domain* domain, const int* istate, const int* gstate);

        ~HPLUS_problem();

        /**
         * Get the initial state
         * 
         * @param istate: A pointer to the variable to save the return value into:
         * this variable must be a BitField of sum(rj) bits (where [0, rj[ is the range of variable vj)
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const void get_istate(BitField* istate);

        /**
         * Get the goal state
         * 
         * @param istate: A pointer to the variable to save the return value into:
         * this variable must be a BitField of sum(rj) bits (where [0, rj[ is the range of variable vj)
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const void get_gstate(BitField* gstate);

        /**
         * Compare a solution found with the best found so far and stores it if it's the new best one
         * 
         * @param solution: ordered sequence of actions (represented by their index in the domain) that represent the solution found
         * @param nact: number of actions this solution has
         * @param cost: cost of this solution
         */
        void update_best_solution(const int* solution, const int* nact, const int* cost);

        /**
         * Get the best solution found so far
         * 
         * @param solution: A pointer to the variable to save the solution into:
         * this variable must be a list of integer, already allocated, with a length >= m (only the first nact variables will be set)
         * @param nact: A pointer to the variable to save the number of actions the solution has into: int
         * @param cost: A pointer to the variable to save the cost of the solution into: int
         */
        const void get_best_solution(int* solution, int* nact, int* cost);

        /**
         * Get the cost of the best solution found so far
         * 
         * @param cost: A pointer to the variable to save the return value into: int
         */
        const void get_best_cost(int* cost);

    private:
    
        /**
         * Domain associated to this problem
         */
        HPLUS_domain* domain;

        /**
         * Initial state of the problem (bit field)
         */
        BitField* initial_state;

        /**
         * Goal state of the problem (bit field)
         */
        BitField* goal_state;

        /**
         * Best solution found so far as a ordered sequence of indexes corresponding to actions
         */
        int* best_solution;

        /**
         * Number of actions in the best solution found so far
         */
        int best_nact;

        /**
         * Cost of the best solution found so far
         */
        int best_cost;

};

#endif