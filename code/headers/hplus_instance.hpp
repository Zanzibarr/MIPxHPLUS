#ifndef _HPLUS_INST_H
#define _HPLUS_INST_H

#include "utils.hpp"
#include "logging.hpp"

/**
 * Struct representing an action with preconditions, effects and cost
 */
struct HPLUS_action  {

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
     * DUMMY constructor for array declarations
     */
    HPLUS_action() { pre = NULL; eff = NULL; cost = INFINITY; }

    /**
     * Constructor HPLUS_action
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
        bool unitary_cost() const;

        /**
         * Get the number of variables this problem has (referred as n)
         * 
         * @return The number of variables this problem has
         */
        int get_nvar() const;

        /**
         * Get the ranges of each variable
         * 
         * @return The list of ranges for each variable
         * ->
         * [r0, r1, ..., rn] for each variable vj, value(vj) is in [0, rj[
         */
        const int* get_var_ranges() const;

        /**
         * Get the size of the BitFields used in this domain
         * 
         * @return The size of the bitfields used in this domain
         */
        int get_bfsize() const;

        /**
         * Get the number of actions this domain has (referred as m)
         * 
         * @param The number of actions this domain has
         */
        int get_nact() const;

        /**
         * Get the list of possible actions
         * 
         * @return The list of actions this domain has
         */
        const HPLUS_action* get_actions() const;

        const Logger* get_logger() const;

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
         * Size of bitfields used to represent states
         */
        int bf_size;

        /**
         * Number of actions in the problem (m)
         */
        int n_act;

        /**
         * List of actions that can be done
         */
        HPLUS_action* actions;

        /**
         * Logger to use for info/debugging
         */
        Logger* logger;

        // UTILS

        void parse_inst_file(const FILE* file);

};

class HPLUS_problem {

    public:

        HPLUS_problem(HPLUS_domain* domain, const unsigned int* istate, const unsigned int* gstate);

        ~HPLUS_problem();

        /**
         * Get the initial state
         * 
         * @return The initial state as a BitField
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const BitField* get_istate() const;

        /**
         * Get the goal state
         * 
         * @return The goal state as a BitField
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const BitField* get_gstate() const;

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
        void get_best_solution(int* solution, int* nact, int* cost) const;

        /**
         * Get the cost of the best solution found so far
         * 
         * @return The cost of the best solution found so far
         */
        int get_best_cost() const;

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