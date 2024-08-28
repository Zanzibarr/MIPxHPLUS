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
        unsigned int get_nvar_strips() const;

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

        // ====================================================== //
        // ===================== ALGORITHMS ===================== //
        // ====================================================== //

        /**
         * Extracts landmarks
         * (Section 4.1 of Imai's paper)
         *
         * @param landmarks_set (output) Fact landmarks for each variable (strips representation) (landmarks_set[4] is a bitfield representing all fact landmarks for variable 4); each bitfield has one more bit (in the last position) to represent the full set
         * @param fact_landmarks (output) Set of fact landmarks for the goal state
         * @param act_landmarks (output) Set of act landmarks for the goal state
         */
        void landmarks_extraction(
            std::vector<my::BitField>& landmarks_set,
            my::BitField& fact_landmarks,
            my::BitField& act_landmarks
        ) const;

        /**
         * Extract first adders
         * (Section 4.2 of Imai's paper)
         *
         * @param landmarks_set Fact landmarks for each variable -> You can get this from the landmarks_extraction method
         * @param fadd (output) List of first adders for each action
         */
        void first_adders_extraction(
            const std::vector<my::BitField>& landmarks_set,
            std::vector<my::BitField>& fadd
        ) const;

        /**
         * Extract relevant variables/actions
         * (Section 4.2 of Imai's paper)
         *
         * @param fact_landmarks Fact landmarks for each variable -> You can get this from the landmarks_extraction method
         * @param fadd List of first adders for each action -> You can get this from the first_adders_extraction method
         * @param relevant_variables (output) Set of relevant variables
         * @param relevant_actions (output) Set of relevant actions
         */
        void relevance_analysis(
            const my::BitField& fact_landmarks,
            const std::vector<my::BitField>& fadd,
            my::BitField& relevant_variables,
            my::BitField& relevant_actions
        ) const;

        /**
         * Extract dominated actions
         * (Section 4.3 of Imai's paper)
         *
         * @param landmarks_set Fact landmarks for each variable -> You can get this from the landmarks_extraction method
         * @param fadd List of first adders for each action -> You can get this from the first_adders_extraction method
         * @param eliminated_actions List of actions that have been (so far) eliminated
         * @param fixed_actions List of actions that have been (so far) fixed
         * @param dominated_actions (output) Set of dominated actions
         */
        void dominated_actions_extraction(
            const std::vector<my::BitField>& landmarks_set,
            const std::vector<my::BitField>& fadd,
            const my::BitField& eliminated_actions,
            const my::BitField& fixed_actions,
            my::BitField& dominated_actions
        ) const;

        /**
         * Immediate action application
         * (Section 4.4 of Imai's paper)
         *
         * @param act_landmarks Set of act landmarks for the goal state
         * @param eliminated_variables List of variables that have been (so far) eliminated
         * @param fixed_variables (edited) List of variables that have been (so far) fixed
         * @param eliminated_actions List of actions that have been (so far) eliminated
         * @param fixed_actions (edited) List of actions that have been (so far) fixed
         * @param eliminated_first_archievers List of first archievers that have been (so far) eliminated
         * @param fixed_first_archievers (edited) List of first archievers that have been (so far) fixed
         * @param fixed_var_timestamps (edited) List of timestamp for variables to be fixed
         * @param fixed_act_timestamps (edited) List of timestamp for actions to be fixed
         */
        void immediate_action_application(
            const my::BitField& act_landmarks,
            const my::BitField& eliminated_variables,
            my::BitField& fixed_variables,
            const my::BitField& eliminated_actions,
            my::BitField& fixed_actions,
            std::vector<my::BitField>& eliminated_first_archievers,
            std::vector<my::BitField>& fixed_first_archievers,
            std::vector<int>& fixed_var_timestamps,
            std::vector<int>& fixed_act_timestamps
        ) const;

        /**
         * Extract inverse oactions
         * (Section 4.6 of Imai's paper)
         *
         * @param eliminated_actions List of actions that have been (so far) eliminated
         * @param fixed_actions List of actions that have been (so far) fixed
         * @param inverse_actions (output) Set of inverse action for each action
         */
        void inverse_actions_extraction(
            const my::BitField& eliminated_actions,
            const my::BitField& fixed_actions,
            std::vector<my::BitField>& inverse_actions
        ) const;

        /**
         * Model enhancement form Imai's paper
         * (Section 4 of Imai's paper)
         *
         * @param eliminated_variables (output) List of variables that can be eliminated (set to 0)
         * @param fixed_variables (output) List of variables that can be fixed (set to 1)
         * @param eliminated_actions (output) List of actions that can be eliminated (set to 0)
         * @param fixed_actions (output) List of actions that can be fixed (set to 1)
         * @param inverse_actions (output) List of inverse actions for each action (if an action is used, it's inverse isn't in the optimal plan)
         * @param eliminated_first_archievers (output) List of first archievers that can be eliminated (set to 0)
         * @param fixed_first_archievers (output) List of first archievers that can be fixed (set to 1)
         * @param fixed_var_timestamps (output) List of timestamps for variables that can be fixed (for models that use timestamps)
         * @param fixed_act_timestamps (output) List of timestamps for actions that can be fixed (for models that use timestamps)
         */
        void extract_imai_enhancements(
            my::BitField& eliminated_variables,
            my::BitField& fixed_variables,
            my::BitField& eliminated_actions,
            my::BitField& fixed_actions,
            std::vector<my::BitField>& inverse_actions,
            std::vector<my::BitField>& eliminated_first_archievers,
            std::vector<my::BitField>& fixed_first_archievers,
            std::vector<int>& fixed_var_timestamps,
            std::vector<int>& fixed_act_timestamp
        ) const;

    private:

        int version_;
        bool use_costs_;

        unsigned int n_var_;
        unsigned int n_act_;
        unsigned int nvarstrips_;

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