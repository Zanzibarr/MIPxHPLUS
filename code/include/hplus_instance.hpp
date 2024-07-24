#ifndef _HPLUS_INST_H
#define _HPLUS_INST_H

#include "utils.hpp"

// ##################################################################### //
// ########################### HPLUS_VARIABLE ########################## //
// ##################################################################### //

class HPLUS_variable {

    public:
        
        /**
         * @param range: The range of values [0, range - 1] this variable can be in
         * @param name: The name of the variable
         * @param val_names: A list of name for each value the variable can be in
         */
        HPLUS_variable(const unsigned int range, const std::string name, const std::vector<std::string> val_names);

        ~HPLUS_variable();

        /**
         * @return The range of values this variable can be set to
         */
        unsigned int get_range() const;

        /**
         * @return The name of the variable (pointer to a single string)
         */
        const std::string* get_name() const;

        /**
         * @return The list of names for each value the variable can be set to (list of strings)
         */
        const std::string* get_val_names() const;

    private:

        /**
         * Range of the variable
         */
        unsigned int range;

        /**
         * Name of the variable
         */
        std::string name;

        /**
         * Name for each value the variable can be set to
         */
        std::string* val_names;

};

// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

class HPLUS_action  {

    public:

        /**
         * @param pre_bf: BitField of the precondition state for this action -> to prevent accidental deletes this will be set to nullptr after
         * @param eff_bf: BitField of the effects on the state this action has -> to prevent accidental deletes this will be set to nullptr after
         * @param cost: Cost of the action
         * @param name: Name of the action
         */
        HPLUS_action(BitField* pre_bf, BitField* eff_bf, const unsigned int cost, const std::string name);

        ~HPLUS_action();

        /**
         * @return The BitField representing the precondition state for this action
         */
        const BitField* get_pre() const;

        /**
         * @return The BitField representing the effects on the state this action has
         */
        const BitField* get_eff() const;

        /**
         * @return The cost of this action
         */
        unsigned int get_cost() const;

        /**
         * @return The name of this action
         */
        const std::string* get_name() const;

    private:

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
        unsigned int cost;

        /**
         * Symbolic name of the action
         */
        std::string name;

};

// ##################################################################### //
// ########################### HPLUS_INSTANCE ########################## //
// ##################################################################### //

class HPLUS_instance {

    public:

        /**
         * Create an HPLUS_instance from a file produced by the fast downward translator
         * 
         * @param file_path: Path of the file where to read the instance from
         * @param problem: Pointer to the HPLUS_problem to build from this domain
         * @param logger: Logger to associate to this domain
         */
        HPLUS_instance(const std::string file_path, const Logger* logger);

        ~HPLUS_instance();

        /**
         * @return true if unitary cost is used, false if generic cost is used
         */
        bool unitary_cost() const;

        /**
         * @return The number of variables this problem has
         */
        unsigned int get_nvar() const;

        /**
         * @return The number of actions this domain has
         */
        unsigned int get_nact() const;

        /**
         * @return The size of the bitfields used in this domain
         */
        unsigned int get_bfsize() const;

        /**
         * @return The list of variables this domain has
         */
        const HPLUS_variable** get_variables() const;

        /**
         * @return The list of actions this domain has
         */
        const HPLUS_action** get_actions() const;

        /**
         * @return The logger associated to this domain
         */
        const Logger* get_logger() const;

        /**
         * @return The initial state as a BitField
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const BitField* get_istate() const;

        /**
         * @return The goal state as a BitField
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const BitField* get_gstate() const;

        /**
         * Compare a solution found with the best found so far and stores it if it's the new best one
         * @param solution: ordered sequence of actions (represented by their index in the domain) that represent the solution found
         * @param nact: number of actions this solution has
         * @param cost: cost of this solution
         */
        void update_best_solution(const std::vector<unsigned int> solution, const unsigned int cost);

        /**
         * Get the best solution found so far
         * @param solution: Vector where to save the solution into (cleared and resized)
         * @param nact: Integer where to save the number of actions into
         * @param cost: Integer where to save the cost into
         */
        void get_best_solution(std::vector<unsigned int>* solution, unsigned int* cost) const;

        /**
         * @return The cost of the best solution found so far
         */
        unsigned int get_best_cost() const;

    private:

        /**
         * Version read from the SAS file generated by Fast Downward
         */
        int version;

        /**
         * This instance uses generic or unitary costs
         */
        bool use_costs;

        /**
         * Number of variables in the problem (n)
         */
        unsigned int n_var;

        /**
         * Number of actions in the problem (m)
         */
        unsigned int n_act;

        /**
         * Number of bits required to store a state in this problem
         */
        unsigned int bf_size;
        
        /**
         * List of variables this domain has
         */
        const HPLUS_variable** variables;

        /**
         * List of actions this domain has
         */
        const HPLUS_action** actions;

        /**
         * Logger used to print info/debug the execution of this domain
         */
        const Logger* logger;

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
        unsigned int* best_solution;

        /**
         * Number of actions in the best solution found so far
         */
        unsigned int best_nact;

        /**
         * Cost of the best solution found so far
         */
        unsigned int best_cost;

        // UTILS

        void parse_inst_file(std::ifstream* file);

};

// ##################################################################### //
// ########################### HPLUS_PROBLEM ########################### //
// ##################################################################### //

// class HPLUS_problem {

//     public:

//         /**
//          * @param domain: Domain to associate to this problem
//          * @param istate: Initial state of the problem -> to prevent accidental deletes this will be set to nullptr after
//          * @param gstate: Goal state of the problem -> to prevent accidental deletes this will be set to nullptr after
//          */
//         HPLUS_problem(HPLUS_domain* domain, BitField* istate, BitField* gstate);

//         ~HPLUS_problem();

//     private:

// };

#endif