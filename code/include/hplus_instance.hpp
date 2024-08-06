#ifndef HPLUS_INST_H
#define HPLUS_INST_H

#include "utils.hpp"

// ##################################################################### //
// ########################### HPLUS_VARIABLE ########################## //
// ##################################################################### //

class HPLUS_variable {

    public:
        
        explicit HPLUS_variable() = default;

        /**
         * @param range: The range of values [0, range - 1] this variable can be in
         * @param name: The name of the variable
         * @param val_names: A list of name for each value the variable can be in
         */
        explicit HPLUS_variable(unsigned int range, const std::string& name, const std::vector<std::string>& val_names);

        /**
         * @return The range of values this variable can be set to
         */
        unsigned int get_range() const;

        /**
         * @return The name of the variable (pointer to a single string)
         */
        const std::string& get_name() const;

        /**
         * @return The list of names for each value the variable can be set to (list of strings)
         */
        const std::vector<std::string>& get_val_names() const;

    private:

        unsigned int range_;
        std::string name_;
        std::vector<std::string> val_names_;

};

// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

class HPLUS_action  {

    public:

        explicit HPLUS_action() = default;

        /**
         * @param pre_bf: BitField of the precondition state for this action
         * @param eff_bf: BitField of the effects on the state this action has
         * @param cost: Cost of the action
         * @param name: Name of the action
         */
        explicit HPLUS_action(my::BitField& pre_bf, my::BitField& eff_bf, unsigned int cost, const std::string& name);

        /**
         * @return The BitField representing the precondition state for this action
         */
        const my::BitField& get_pre() const;

        /**
         * @return The BitField representing the effects on the state this action has
         */
        const my::BitField& get_eff() const;

        /**
         * @return The cost of this action
         */
        unsigned int get_cost() const;

        /**
         * @return The name of this action
         */
        const std::string& get_name() const;

    private:

        /**
         * Value each variable must have for the action to be taken
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) must be 0, value(v1) must be 1, value(v2) must be 3
         */
        my::BitField pre_;

        /**
         * Value each variable has after the action has been taken
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) will be 0, value(v1) will be 1, value(v2) will be 3
         */
        my::BitField eff_;

        unsigned int cost_;
        std::string name_;

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
         */
        explicit HPLUS_instance(const std::string& file_path);

        /**
         * @return The version of the fast downward translator used to generate this instance
         */
        int get_version() const;

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
        const std::vector<HPLUS_variable>& get_variables() const;

        /**
         * @return The list of actions this domain has
         */
        const std::vector<HPLUS_action>& get_actions() const;

        /**
         * @return The initial state as a BitField
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const my::BitField& get_istate() const;

        /**
         * @return The goal state as a BitField
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) = 0, value(v1) = 1, value(v2) = 3
         */
        const my::BitField& get_gstate() const;

        /**
         * Compare a solution found with the best found so far and stores it if it's the new best one
         * @param solution: ordered sequence of actions (represented by their index in the domain) that represent the solution found
         * @param cost: cost of this solution
         */
        void update_best_solution(const std::vector<unsigned int>& solution, unsigned int cost);

        /**
         * Get the best solution found so far
         * @param solution: Vector where to save the solution into (cleared and resized)
         * @param cost: Integer where to save the cost into
         */
        void get_best_solution(std::vector<unsigned int>& solution, unsigned int& cost) const;

        /**
         * Prints the best solution found so far
        */
        void print_best_sol() const;

        /**
         * @return The cost of the best solution found so far
         */
        unsigned int get_best_cost() const;

    private:

        int version_;
        bool use_costs_;

        unsigned int n_var_;
        unsigned int n_act_;
        unsigned int bf_size_;

        std::vector<HPLUS_variable> variables_;
        std::vector<HPLUS_action> actions_;

        my::BitField initial_state_;
        my::BitField goal_state_;

        std::vector<unsigned int> best_solution_;
        unsigned int best_nact_;
        unsigned int best_cost_;

        // UTILS

        void parse_inst_file_(std::ifstream* file);

};

#endif