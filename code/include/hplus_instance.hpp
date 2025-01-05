#ifndef HPLUS_INST_H
#define HPLUS_INST_H

#include "utils.hpp"

// ##################################################################### //
// ############################ HPLUS_ACTION ########################### //
// ##################################################################### //

class HPLUS_action {

    public:

        explicit HPLUS_action() = default;
        explicit HPLUS_action(const my::binary_set& preconditions, const my::binary_set& effects, unsigned int cost, const std::string& name);

        const my::binary_set& get_pre() const;
        const std::vector<size_t>& get_pre_sparse() const;
        const my::binary_set& get_eff() const;
        const std::vector<size_t>& get_eff_sparse() const;
        unsigned int get_cost() const;
        const std::string& get_name() const;

    private:

        my::binary_set pre_;
        my::binary_set eff_;
        std::vector<size_t> pre_sparse_;
        std::vector<size_t> eff_sparse_;
        unsigned int cost_;
        std::string name_;

};

// ##################################################################### //
// ########################### HPLUS_INSTANCE ########################## //
// ##################################################################### //

extern class HPLUS_instance {

    public:
    
        explicit HPLUS_instance() = default;
        explicit HPLUS_instance(const std::string& instance_file);

        int get_version() const;
        bool unitary_cost() const;

        /**
         * @brief Get the number of variables this problem has
         * 
         * @param simplified set this to true to get the number of variables post simplification
         * @return size_t Number of variables pre/post simplification
         */
        size_t get_n_var(bool simplified = false) const;
        /**
         * @brief Get the number of actions this problem has
         * 
         * @param simplified set this to true to get the number of actions post simplification
         * @return size_t Number of actions pre/post simplification
         */
        size_t get_n_act(bool simplified = false) const;
        /**
         * @brief Get a set containing the variables indexes this problem has
         * 
         * @return const my::binary_set& Set of variables post simplification
         */
        const my::binary_set get_remaining_variables() const;
        /**
         * @brief Get a set containing the actions indexes this problem has
         * 
         * @return const my::binary_set& Set of actions post simplification
         */
        const my::binary_set get_remaining_actions() const;
        const std::vector<HPLUS_action>& get_actions() const;
        const my::binary_set& get_goal_state() const;

        void update_best_sol(const std::vector<size_t>& solution, unsigned int cost);
        void get_best_sol(std::vector<size_t>& solution, unsigned int& cost);
        unsigned int get_best_sol_cost();
        void print_best_sol();
        
        void problem_simplification();

        const my::binary_set& get_fixed_variables() const;
        const my::binary_set& get_fixed_actions() const;
        const std::vector<my::binary_set>& get_eliminated_fa() const;
        const std::vector<my::binary_set>& get_fixed_fa() const;
        const std::vector<int> get_timestamps_var() const;
        const std::vector<int> get_timestamps_act() const;

        void store_cycle(std::vector<size_t>& cycle);
        const std::vector<std::vector<size_t>>& get_cycles() const;

        size_t var_idx_post_simplification(size_t var_idx) const;
        size_t act_idx_post_simplification(size_t act_idx) const;
        size_t fa_idx_post_simplification(size_t act_idx, size_t var_idx) const;
        size_t cpx_idx_to_act_idx(size_t cpx_idx) const;

    private:

        int version_;
        bool unitary_costs_;

        size_t n_var_;
        size_t n_act_;
        size_t n_var_post_simpl_;
        size_t n_act_post_simpl_;

        std::vector<HPLUS_action> actions_;

        my::binary_set goal_state_;

        std::vector<size_t> best_solution_;
        unsigned int best_cost_;

        my::binary_set eliminated_var_;
        my::binary_set fixed_var_;
        my::binary_set eliminated_act_;
        my::binary_set fixed_act_;
        std::vector<my::binary_set> eliminated_fa_;
        std::vector<my::binary_set> fixed_fa_;
        std::vector<int> timestamps_var_;
        std::vector<int> timestamps_act_;
        // std::map<size_t, std::vector<size_t>>& inverse_act_;

        std::vector<size_t> var_idx_post_simplification_;
        std::vector<size_t> act_idx_post_simplification_;
        std::vector<size_t> fa_individual_start_;
        std::vector<size_t> cpx_idx_to_act_idx_;

        std::vector<std::vector<size_t>> dynamic_model_cycles_;

        pthread_mutex_t solution_read_write_;

        void landmarks_extraction(std::vector<my::binary_set>& landmarks_set, my::binary_set& fact_landmarks, my::binary_set& act_landmarks) const;
        void first_adders_extraction(const std::vector<my::binary_set>& landmarks_set, std::vector<my::binary_set>& fadd) const;
        void relevance_analysis(const my::binary_set& fact_landmarks, const std::vector<my::binary_set>& fadd);
        void dominated_actions_elimination(const std::vector<my::binary_set>& landmarks_set, const std::vector<my::binary_set>& fadd);
        void immediate_action_application(const my::binary_set& act_landmarks);
        //[ ]: This might not be worth doing
        // void inverse_actions_extraction();
        void imai_model_enhancements();

        void parse_inst_file_(const std::string& instance_path_);

} HPLUS_inst;

// ##################################################################### //
// ############################# HPLUS_ENV ############################# //
// ##################################################################### //

extern struct HPLUS_environment {

    my::execution_status exec_status;
    my::solution_status sol_status;

    int cpx_terminate;

    std::string input_file;
    bool log;
    std::string log_name;
    std::string run_name;

    std::string alg;

    bool problem_simplification_enabled;
    bool imai_tighter_var_bound_enabled;
    bool heuristic_enabled;
    bool warm_start_enabled;

    unsigned int time_limit;

} HPLUS_env;

// ##################################################################### //
// ############################ HPLUS_STATS ############################ //
// ##################################################################### //

extern struct HPLUS_statistics {

    double parsing_time;
    double simplification_time;
    double heuristic_time;
    double build_time;
    double callback_time;
    double execution_time;
    double total_time;

    pthread_mutex_t callback_time_mutex_;

    void print() const;

} HPLUS_stats;

#endif