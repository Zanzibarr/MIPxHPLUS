#ifndef HPLUS_INST_H
#define HPLUS_INST_H

#include "utils.hpp"
#include <thread>

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
         * @return The BitField representing the precondition state for this action (note that the initial state variables have been removed)
         */
        const my::BitField& get_pre() const;

        /**
         * @return The precomputed sparse list of precondition of this action (note that the initial state variables have been removed) 
         */
        const std::vector<unsigned int>& get_pre_sparse() const;

        /**
         * @return The BitField representing the add effects on the state this action has (note that the initial state variables have been removed)
         */
        const my::BitField& get_eff() const;

        /**
         * @return The precomputed sparse list of add effects of this action (note that the initial state variables have been removed) 
         */
        const std::vector<unsigned int>& get_eff_sparse() const;

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
         * Precomputed list of preconditions
         */
        std::vector<unsigned int> sparse_pre_;

        /**
         * Value each variable has after the action has been taken
         * ->
         * [1000 0100 0001] -> Assuming a size of 3 and a range of 4 for each variable, value(v0) will be 0, value(v1) will be 1, value(v2) will be 3
         */
        my::BitField eff_;

        /**
         * Precomputed list of add effects
         */
        std::vector<unsigned int> sparse_eff_;

        unsigned int cost_;
        std::string name_;

};

// ##################################################################### //
// ########################### HPLUS_INSTANCE ########################## //
// ##################################################################### //

extern class HPLUS_instance {

    public:

        explicit HPLUS_instance() = default;

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
         * @return The number of actions this domain has
         */
        unsigned int get_nact() const;

        /**
         * @return The number of variables (after binary expansion) this domain has
         */
        unsigned int get_nvar() const;

        /**
         * @return The list of actions this domain has
         */
        const std::vector<HPLUS_action>& get_actions() const;

        /**
         * @return The goal state as a BitField (note that the initial state variables have been removed)
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
        void get_best_solution(std::vector<unsigned int>& solution, unsigned int& cost);

        /**
         * Prints the best solution found so far
        */
        void print_best_sol();

        // ====================================================== //
        // ==================== OPTIMIZATIONS =================== //
        // ====================================================== //

        void initial_heuristic();

        void model_optimization();

        void optimized_heuristic();

        // set of fixed variables for model optimization
        my::BitField model_opt_fixed_var;
        // set of fixed variables for model optimization
        my::BitField model_opt_fixed_act;
        // set of fixed first archievers for model optimization
        std::vector<my::BitField> model_opt_eliminated_fa;
        // set of eliminated first archievers for model optimization
        std::vector<my::BitField> model_opt_fixed_fa;
        // set of variable timestamps for model optimization
        std::vector<int> model_opt_timestamps_var;
        // set of action timestamps for model optimization
        std::vector<int> model_opt_timestamps_act;

        /**
         * @return the optimized number of variables (original nvar - # eliminated variables)
         */
        unsigned int get_nvar_opt() const;

        /**
         * @return the optimized number of actions (original nact - # eliminated actions)
         */
        unsigned int get_nact_opt() const;

        /**
         * @return the eliminated variables
         */
        const my::BitField& get_model_opt_eliminated_var() const;

        /**
         * @return the eliminated actions
         */
        const my::BitField& get_model_opt_eliminated_act() const;

        /**
         * @param cpxidx the index of the cplex action to convert
         *
         * @return the index of the action given the corresponding cplex index
         */
        unsigned int cpxidx_to_actidx(unsigned int cpxidx) const;

        int actidx_to_cpxidx(unsigned int actidx) const;
        int varidx_to_cpxidx(unsigned int varidx) const;

    private:

        int version_;
        bool use_costs_;

        unsigned int n_act_;
        unsigned int n_var_;

        std::vector<HPLUS_action> actions_;

        my::BitField goal_state_;

        std::vector<unsigned int> best_solution_;
        unsigned int best_nact_;
        unsigned int best_cost_;

        // ====================================================== //
        // ==================== OPTIMIZATIONS =================== //
        // ====================================================== //

        /**
         * Extracts landmarks
         * (Section 4.1 of Imai's paper)
         */
        void landmarks_extraction(
            std::vector<my::BitField>& landmarks_set,
            my::BitField& fact_landmarks,
            my::BitField& act_landmarks
        ) const;

        /**
         * Extract first adders
         * (Section 4.2 of Imai's paper)
         */
        void first_adders_extraction(
            const std::vector<my::BitField>& landmarks_set,
            std::vector<my::BitField>& fadd
        ) const;

        /**
         * Extract relevant variables/actions
         * (Section 4.2 of Imai's paper)
         */
        void relevance_analysis(
            const my::BitField& fact_landmarks,
            const std::vector<my::BitField>& fadd
        );

        /**
         * Extract dominated actions
         * (Section 4.3 of Imai's paper)
         */
        void dominated_actions_elimination(
            const std::vector<my::BitField>& landmarks_set,
            const std::vector<my::BitField>& fadd
        );

        /**
         * Immediate action application
         * (Section 4.4 of Imai's paper)
         */
        void immediate_action_application(
            const my::BitField& act_landmarks
        );

        // /**
        //  * Inverse action extraction
        //  * (Section 4.6 of Imai's paper)
        //  */
        // void inverse_actions_extraction(
        //     const my::BitField& eliminated_actions,
        //     const my::BitField& fixed_actions,
        //     std::map<unsigned int, std::vector<unsigned int>>& inverse_actions
        // ) const;

        /**
         * Model enhancement form Imai's paper
         * (Section 4 of Imai's paper)
         */
        void imai_model_enhancements();

        // set of eliminated variables for model optimization
        my::BitField model_opt_eliminated_var_;
        // set of eliminated actions for model optimization
        my::BitField model_opt_eliminated_act_;
        // optimized number of variables
        unsigned int model_opt_nvar_;
        // optimized number of actions
        unsigned int model_opt_nact_;

        std::vector<int> model_opt_varidx_to_cpxidx_;
        std::vector<int> model_opt_actidx_to_cpxidx_;
        std::vector<unsigned int> model_opt_cpxidx_to_actidx_;

        // UTILS

        pthread_mutex_t solution_read_write_;

        void parse_inst_file_(std::ifstream* file);

} HPLUS_inst;

#endif