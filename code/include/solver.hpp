#pragma once

#include <cplex.h>

#include <binary_set.hxx>
#include <cstdint>
#include <list>
#include <logger.hxx>
#include <mutex>
#include <parameters.hxx>
#include <pq.hxx>
#include <random>
#include <stack>
#include <stats_registry.hxx>
#include <unordered_set>
#include <utility>

// Keep this: we need it for the "using namespce utilz"
#include "constants.hpp"
#include "utils.hpp"

struct Action {
    std::vector<unsigned int> pre_sparse;
    std::vector<unsigned int> eff_sparse;
    unsigned int cost;
};

struct Instance {
    bool unit_costs{};
    unsigned int n{};
    unsigned int m{};
    unsigned int nfadd{};
    std::vector<Action> actions;
    std::vector<std::string> actions_names;
    BinarySet goal;
};

class Solver {
   public:
    Solver(const ParameterRegistry& params, Logger& logger) : params_(params), logger_(logger) { global_.using_cplex = false; };
    void solve();
    void show();
    auto get() -> std::vector<std::string>;

   private:
    void read_instance_();

    // Used to print a bound computed: it needs to consider the prefix computed (or not) during the preprocessing
    auto actual_bound_(double value) const -> double {
        return is_gr_or_eq_double(value, constants::infeas_bound) ? constants::infeas_bound : value + global_.cost_prefix;
    }

    // ~~~~~~~~~~~~ preprocessing ~~~~~~~~~~~~ //
    // Haslum, Slaney, Thiebaux: "Minimal landmarks for optimal deletefree planning"
    // Imai, Fukunaga: "On a Practical, Integer-Linear Programming Model for Delete-Free Tasks and its Use as a Heuristic for Cost-Optimal Planning"
    void preprocess_();
    // Recomputes landmarks and fixed facts/actions; returns whether the landmark sets differ from the ones in input
    [[nodiscard]]
    auto prep_fact_landmarks_(std::vector<std::vector<unsigned int>>& landmarks) -> bool;
    // Returns whether at least one effect entry was deleted
    [[nodiscard]]
    auto prep_first_adders_(std::vector<std::vector<unsigned int>>& landmarks) -> bool;
    void prep_relevance_backward_(BinarySet& relevant_variables);
    void prep_relevance_forward_(BinarySet& relevant_variables);
    // Returns whether at least one action was eliminated
    [[nodiscard]]
    auto prep_dominated_actions_(std::vector<std::vector<unsigned int>>& landmarks) -> bool;
    [[nodiscard]]
    auto prep_orbital_probing_() -> bool;
    [[nodiscard]]
    auto prep_initial_action_sequencing_(std::vector<std::vector<unsigned int>>& landmarks) -> bool;
    void prep_eliminated_facts_(std::vector<std::vector<unsigned int>>& landmarks);
    void prep_eliminated_actions_();
    void prep_fixed_actions_();
    void prep_setup_helpers_();

    void print_info_();

    // ~~~~~~~~~~~ landmarks utils ~~~~~~~~~~~ //
    // Haslum, Slaney, Thiebaux: "Minimal landmarks for optimal deletefree planning"
    // TODO Salvagnin, Zanella: "Tighter Bounds for h+ MIP Planning via LM-Cut Strengthening and Cut Generation"
    void landmark_minimalization_greedy_(std::vector<unsigned int>& landmark, const BinarySet& reachable_state);
    void landmark_minimalization_(std::vector<unsigned int>& landmark, const std::vector<unsigned int>& unapplicable, BinarySet reachable_state);
    // ATTENTION: check_landmark_(..) is meant to be a check for debugging... it's not optimized to be used as a routine
    void check_landmark_(const std::vector<unsigned int>& landmark);

    // ~~~~~~~~~~~~~~~~ lmcut ~~~~~~~~~~~~~~~~ //
    // Domshlak, Helmert: "Landmarks, Critical Paths and Abstractions: What's the Difference Anyway?"
    // Lauer, Fickert: "Beating LM-cut with LM-cut: Quick Cutting and Practical Tie Breaking for the Precondition Choice Function"
    // TODO Salvagnin, Zanella: "Tighter Bounds for h+ MIP Planning via LM-Cut Strengthening and Cut Generation"
    using lmcut_hmax_function = std::pair<int, double> (Solver::*)(const std::vector<unsigned int>&);
    void solve_lmcut_();

    [[nodiscard]]
    auto lmcut_lmcut_(const lmcut_hmax_function& hmax, char minimization) -> std::pair<std::vector<std::vector<unsigned int>>, double>;
    [[nodiscard]]
    auto lmcut_int_separation_(const std::vector<unsigned int>& used_actions, const lmcut_hmax_function& hmax, char minimization)
        -> std::pair<bool, std::vector<std::vector<unsigned int>>>;
    [[nodiscard]]
    auto lmcut_relax_separation_(const std::vector<double>& actions_weights, const lmcut_hmax_function& hmax, char minimization)
        -> std::pair<bool, std::vector<std::vector<unsigned int>>>;

    [[nodiscard]]
    auto lmcut_hmax_arbitrary_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double>;
    [[nodiscard]]
    auto lmcut_hmax_inverse_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double>;
    [[nodiscard]]
    auto lmcut_hmax_vdm_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double>;
    [[nodiscard]]
    auto lmcut_hmax_random_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double>;
    [[nodiscard]]
    auto lmcut_hmax_gzd_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double>;
    [[nodiscard]]
    auto lmcut_hmax_bd_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double>;
    [[nodiscard]]
    auto lmcut_hmax_gzd_bd_(const std::vector<unsigned int>& preconditions) -> std::pair<int, double>;

    void lmcut_init_();
    [[nodiscard]]
    auto lmcut_compute_private_(const lmcut_hmax_function& hmax, char minimization) -> std::pair<std::vector<std::vector<unsigned int>>, double>;
    void lmcut_update_and_enqueue_effects_values_(priority_queue<double>& queue, unsigned int act_i);
    void lmcut_compute_goal_section_(const lmcut_hmax_function& hmax);
    void lmcut_update_hmax_values_(const std::vector<unsigned int>& changed_actions, const lmcut_hmax_function& hmax);
    [[nodiscard]]
    auto lmcut_compute_cut_(const lmcut_hmax_function& hmax, char minimization) -> std::pair<std::vector<unsigned int>, double>;

    // ~~~~~~~~~~~ primal heuristic ~~~~~~~~~~ //
    // Salvagnin, Zanella: "MIP Formulations for Delete-free AI Planning"
    using primal_hmax_function = double (*)(double, double);
    using greedychoice_function = std::pair<bool, unsigned int> (Solver::*)(const std::list<unsigned int>&, const BinarySet&);
    void solve_primal_();

    void primal_greedy_();
    void primal_hmaxadd_init_values_(const primal_hmax_function& h_eqtype);
    void primal_update_hmax_values_(const std::vector<unsigned int>& new_facts);
    void primal_update_hadd_values_(const std::vector<unsigned int>& new_facts);

    [[nodiscard]]
    auto primal_greedychoice_cost_(const std::list<unsigned int>& candidates, const BinarySet& state) -> std::pair<bool, unsigned int>;
    [[nodiscard]]
    auto primal_greedychoice_cxe_(const std::list<unsigned int>& candidates, const BinarySet& state) -> std::pair<bool, unsigned int>;
    [[nodiscard]]
    auto primal_greedychoice_hmax_(const std::list<unsigned int>& candidates, const BinarySet& state) -> std::pair<bool, unsigned int>;
    [[nodiscard]]
    auto primal_greedychoice_hadd_(const std::list<unsigned int>& candidates, const BinarySet& state) -> std::pair<bool, unsigned int>;

    // ~~~~~~~~~~~ computing hplus ~~~~~~~~~~~ //
    // Imai, Fukunaga: "On a Practical, Integer-Linear Programming Model for Delete-Free Tasks and its Use as a Heuristic for Cost-Optimal Planning"
    // Rankooh, Rintanen: "Efficient Computation and Informative Estimation of h+ by Integer and Linear Programming"
    // Bonet, Castillo: "A Complete Algorithm for Generating Landmarks"
    // Haslum, Slaney, Thiebaux: "Minimal landmarks for optimal deletefree planning"
    // Salvagnin, Zanella: "MIP Formulations for Delete-free AI Planning"
    // TODO Salvagnin, Zanella: "Tighter Bounds for h+ MIP Planning via LM-Cut Strengthening and Cut Generation"
    void solve_hplus_();

    void hplus_init_cplex_();
    void hplus_close_cplex_();
    void hplus_run_cplex_();

    void hplus_build_base_model_();
    void hplus_add_tl_constraints_();
    void hplus_add_ve_constraints_();
    void hplus_enable_candidate_callback_();

    void hplus_post_warm_start_();
    void hplus_post_tl_warm_start_();
    void hplus_post_ve_warm_start_();
    void hplus_post_base_warm_start_();

    static auto CPXPUBLIC hplus_callback_hub_(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle) -> int;
    void hplus_progress_callback_(CPXCALLBACKCONTEXTptr context);

    void hplus_candidate_callback_(CPXCALLBACKCONTEXTptr context);
    void hplus_candidate_get_info_();
    void hplus_reject_candidate_with_new_sol_(CPXCALLBACKCONTEXTptr context, const std::vector<unsigned int>& solution);
    void hplus_separate_cand_sec_cut_(CPXCALLBACKCONTEXTptr context);
    void hplus_separate_cand_lmfront_cut_(CPXCALLBACKCONTEXTptr context);
    void hplus_separate_cand_lmcomp_cut_(CPXCALLBACKCONTEXTptr context);
    void hplus_separate_cand_lmcut_cut_(CPXCALLBACKCONTEXTptr context, char minimization);
    void hplus_reject_cand_sec_(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& cycles);
    void hplus_reject_cand_lm_(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& landmarks);

    void hplus_relaxation_callback_(CPXCALLBACKCONTEXTptr context);
    void hplus_separate_relax_sec_cut_(CPXCALLBACKCONTEXTptr context);
    void hplus_separate_relax_lm_cut_(CPXCALLBACKCONTEXTptr context, bool minimization);
    void hplus_separate_relax_lmcut_cut_(CPXCALLBACKCONTEXTptr context, char minimization);
    void hplus_reject_relax_sec_(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& cycles);
    void hplus_reject_relax_lm_(CPXCALLBACKCONTEXTptr context, const std::vector<std::vector<unsigned int>>& landmarks);

    void hplus_cplex_gather_info_();
    void hplus_parse_cplex_status_();
    void hplus_get_cplex_solution_();

    // ~~~~~~~~~~~ solution updates ~~~~~~~~~~ //
    void try_update_best_bound_(double new_bound);
    void try_update_best_incumbent_(const std::vector<unsigned int>& new_solution, double new_incumbent);

    // ~~~~~~~~~~~~~~ time helpers ~~~~~~~~~~~~ //
    // Wall-clock seconds since solve() started (matches the global timer thread's clock).
    [[nodiscard]] auto elapsed_seconds_() const -> double {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - global_.start_time).count();
    }

    // ~~~~~~~~~~~~~ data members ~~~~~~~~~~~~ //
    Instance inst_;
    struct GlobalData {
        // Wall-clock reference set at solve() entry; used for elapsed/remaining time.
        std::chrono::steady_clock::time_point start_time;

        // Best solution
        double best_bound = 0;
        double best_incumbent = constants::infeas_bound;
        std::vector<unsigned int> solution;
        double cost_prefix = 0;
        std::vector<std::string> solution_prefix;
        std::mutex solution_mutex;

        // Preprocessing and helpers
        BinarySet eliminated_facts;    // Used only during preprocessing, after which it's a binary set of size 1
        BinarySet eliminated_actions;  // Used only during preprocessing, after which it's a binary set of size 1
        BinarySet fixed_facts;
        BinarySet fixed_actions;
        std::vector<std::vector<unsigned int>> act_with_pre;
        std::vector<std::vector<unsigned int>> act_with_eff;
        std::vector<unsigned int> goal_sparse;
        std::vector<unsigned int> initial_actions;  // Actions that are applicable from the initial (empty) state

        // LM-Cut data
        std::vector<std::vector<unsigned int>> landmarks;

        // Primal heuristic data
        std::vector<double> primal_h_values;                                    // hmax/hadd values for each fact
        std::stack<std::pair<unsigned int, double>> primal_trail;               // trail for fact values changes
        std::stack<std::tuple<unsigned int, int, double>> primal_action_trail;  // trail for hmax changes (action, old_pcf, old_pcf_val)
        priority_queue<double> primal_pq;                                       // priority queue fo rhmax/hadd updates
        BinarySet primal_used_actions;                                          // Actions used in the current solution (used only for hmax/hadd)
        BinarySet primal_trail_flags;                                           // tracks which facts are in trail (avoids duplicates)
        BinarySet primal_action_trail_flags;                                    // tracks which actions are in action_trail (hmax only)
        std::vector<int> primal_hmax_pcf;                                       // argmax precondition index per action (-1 = unset)
        std::vector<double> primal_pcf_hmax;                                    // primal_h_values[primal_hmax_pcf[i]]

        // CPLEX
        bool using_cplex = false;  // for debugging purposes
        CPXENVptr hplus_env;
        CPXLPptr hplus_lp;
        CPXLONG hplus_callback_context;
        double hplus_dettime = -1;  // deterministic ticks consumed by CPXmipopt (execution-path fingerprint)

        // Cplex base model helpers
        std::vector<unsigned int> hplus_fadd_cpx_start;

        // Vertex elimination graph
        std::vector<unsigned int> hplus_veg_starts;
        std::vector<std::vector<unsigned int>> hplus_veg_cumulative_graph;

        // Gather info about root node relaxation
        std::atomic<bool> relax_lb_rootnode_recorded{false};
        std::atomic<double> relax_last_root_lb{-1.0};

    } global_;

    struct LocalData {
        // Random generation
        std::mt19937_64 rng;
        bool rng_ready;

        // LM-Cut data
        std::vector<int> lmcut_pcf;
        std::vector<double> lmcut_hmax_values;
        std::vector<double> lmcut_initial_hmax_values;  // For VDM
        std::vector<double> lmcut_pcf_hmax;             // hmax_values[pcf[act]]
        std::vector<double> lmcut_reduced_costs;
        std::unordered_set<unsigned int> lmcut_goal_section;  // Persistent V* for GZD

        // Candidate callback data
        std::vector<double> cand_xstar;
        BinarySet cand_used_actions;
        std::vector<unsigned int> cand_reachable_action_sequence;
        std::vector<unsigned int> cand_unreachable_actions;
        std::vector<unsigned int> cand_unused_actions;
        BinarySet cand_reachable_state;

        // Relaxation callback data
        std::vector<double> relax_xstar;
    };
    // ATTENTION: This data structure is shared for ALL SOLVER INSTANCES in the same thread.. if only one Solver instance is ever used concurrently,
    // then this just means that each thread has its own data and no problem occurs
    inline static thread_local LocalData local_;

    // ~~ statistics, parameters and logger ~~ //
    const ParameterRegistry& params_;
    Logger& logger_;
    StatsRegistry stats_;

    // ~~~~~~~~ early exit conditions ~~~~~~~~ //
    class EarlyExit : public std::exception {
       public:
        enum Reason : std::uint8_t { TIMELIMIT, INFEASIBLE, OPTIMAL };
        EarlyExit(const char* msg, Reason rsn) : message(msg), reason_(rsn) {}
        [[nodiscard]] auto what() const noexcept -> const char* override { return message.c_str(); }
        [[nodiscard]] auto reason() const noexcept -> Reason { return reason_; };

       private:
        std::string message;
        Reason reason_;
    };

    // ~~~~~~~~~~~~~~~~~ rng ~~~~~~~~~~~~~~~~~ //
    // Thread-local generator. Each thread's stream is derived from (seed, logical worker index).
    // Determinism requires a *stable* index: assign it explicitly at worker startup via seed_rng_(idx),
    // so the stream a thread gets depends on its logical role, not on scheduling. Threads that never
    // call seed_rng_ (e.g. single-threaded use) fall back to a lazily assigned index of 0.
    // SplitMix64 finalizer: maps a counter-like input to a well-distributed 64-bit value,
    // so closely-spaced (seed, index) pairs yield uncorrelated mt19937_64 streams.
    [[nodiscard]]
    static auto splitmix64(std::uint64_t state) -> std::uint64_t {
        constexpr std::uint64_t GOLDEN = 0x9E3779B97F4A7C15ULL;
        constexpr std::uint64_t MIX_A = 0xBF58476D1CE4E5B9ULL;
        constexpr std::uint64_t MIX_B = 0x94D049BB133111EBULL;
        state += GOLDEN;
        state = (state ^ (state >> 30)) * MIX_A;
        state = (state ^ (state >> 27)) * MIX_B;
        return state ^ (state >> 31);
    }

    // Seed the calling thread's generator from its logical worker index. Call once at worker startup.
    void seed_rng_(std::uint64_t index) {
        const auto seed = static_cast<std::uint64_t>(params_.get<cli_desc::seed, int>());
        local_.rng.seed(splitmix64(seed ^ (index + 1)));
        local_.rng_ready = true;
    }

    [[nodiscard]]
    auto rng_() -> std::mt19937_64& {
        if (!local_.rng_ready) {
            seed_rng_(0);
        }
        return local_.rng;
    }
};

// ~~~~~~~~~~~~~~~~ utils ~~~~~~~~~~~~~~~~ //
#define scoped_timer(name) make_scoped_timer<name>(stats_)

// ~~~~~~~~~~~~~~~ asserts ~~~~~~~~~~~~~~~ //
#define integritycheck(condition, message)                                                                                           \
    {                                                                                                                                \
        if (!(condition)) [[unlikely]] {                                                                                             \
            logger_[FATAL] << std::format("Assert check failed at {}: {}:{}: {}", __PRETTY_FUNCTION__, __FILE__, __LINE__, message); \
        }                                                                                                                            \
    }

#ifndef NDEBUG
#define myassert(condition, message) integritycheck(condition, message)
#else
#define myassert(condition, message) \
    {                                \
    }
#endif

#define call_cplex(code)                                                                                                                          \
    {                                                                                                                                             \
        switch (code) {                                                                                                                           \
            case 1001: /*CPXERR_NO_MEMORY*/                                                                                                       \
                [[fallthrough]];                                                                                                                  \
            case 1234: /*CPXERR_THREAD_FAILED*/                                                                                                   \
                throw std::bad_alloc();                                                                                                           \
                break;                                                                                                                            \
            case 11: /*CPX_STAT_ABORT_TIME_LIM*/                                                                                                  \
                [[fallthrough]];                                                                                                                  \
            case 13: /*CPX_STAT_ABORT_USER*/                                                                                                      \
                [[fallthrough]];                                                                                                                  \
            case 0:                                                                                                                               \
                break;                                                                                                                            \
            default:                                                                                                                              \
                logger_[FATAL] << std::format("Unhandled CPLEX error code: {} at {}: {}:{}", std::to_string(code), __PRETTY_FUNCTION__, __FILE__, \
                                              __LINE__);                                                                                          \
                break;                                                                                                                            \
        }                                                                                                                                         \
    }

#define unimplemented()                                                                                     \
    {                                                                                                       \
        logger_[FATAL] << std::format("UNIMPLEMENTED: {}: {}:{}", __PRETTY_FUNCTION__, __FILE__, __LINE__); \
    }
