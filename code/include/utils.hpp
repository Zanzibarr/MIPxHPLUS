#ifndef UTILS_H
#define UTILS_H

// ##################################################################### //
// ############################## IMPORTS ############################## //
// ##################################################################### //

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <cstdarg>
#include <string>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <queue>
#include <tuple>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <thread>

// ##################################################################### //
// ######################### PATHS AND FOLDERS ######################### //
// ##################################################################### //

#ifndef HPLUS_HOME_DIR
#define HPLUS_HOME_DIR "placeholder"                // overwritten by cmake
#endif
#define HPLUS_LOG_DIR HPLUS_HOME_DIR"/logs/AAA_output_logs"
#define HPLUS_CPLEX_OUTPUT_DIR HPLUS_HOME_DIR"/logs/cpxout"

// ##################################################################### //
// ########################## C.L.I. ARGUMENTS ######################### //
// ##################################################################### //

#define HPLUS_CLI_INPUT                 "-i"
#define HPLUS_CLI_LOG                   "-l"
#define HPLUS_CLI_LOG_NAME              "-ln"
#define HPLUS_CLI_RUN_NAME              "-rn"
#define HPLUS_CLI_ALG                   "-a"
#define HPLUS_CLI_TIME_LIMIT            "-t"

#define HPLUS_CLI_ALG_IMAI              "imai"
#define HPLUS_CLI_ALG_RANKOOH           "rankooh"

#define HPLUS_CLI_OPT_NO_SIMPL          "-nos"
#define HPLUS_CLI_OPT_IMAI_NO_TB        "-notb"
#define HPLUS_CLI_OPT_NO_HEUR           "-noheur"
#define HPLUS_CLI_OPT_NO_WARM_START     "-nowarm"

// ##################################################################### //
// ########################### DEFAULT VALUES ########################## //
// ##################################################################### //

#define HPLUS_DEF_INPUT_FILE "N/A"
#define HPLUS_DEF_LOG_ENABLED false
#define HPLUS_DEF_LOG_FILE "hplus_log.log"
#define HPLUS_DEF_RUN_NAME "Default run name"
#define HPLUS_DEF_ALG HPLUS_CLI_ALG_RANKOOH
#define HPLUS_DEF_TIME_LIMIT 60
#define HPLUS_DEF_PROB_SIMPL_ENABLED true
#define HPLUS_DEF_IMAI_VAR_BOUND_ENABLED true
#define HPLUS_DEF_HEURISTIC_ENABLED true
#define HPLUS_DEF_WARM_START_ENABLED true

// ##################################################################### //
// ####################### PRINTING AND DEBUGGING ###################### //
// ##################################################################### //

#ifndef HPLUS_VERBOSE
#define HPLUS_VERBOSE 10                            // overwritten by cmake
#endif
#ifndef HPLUS_WARN
#define HPLUS_WARN 1                                // overwritten by cmake
#endif
#ifndef HPLUS_INTCHECK
#define HPLUS_INTCHECK 1                            // overwritten by cmake
#endif

#define LINE "--------------------------------------------------"
#define THICK_LINE "##################################################"

// ##################################################################### //
// ############################ MY NAMESPACE ########################### //
// ##################################################################### //

namespace my {

    class binary_set {

        public:

            explicit binary_set() = default;
            explicit binary_set(size_t capacity, bool full_flag = false);
            binary_set(const binary_set& other_set);

            void add(size_t element);
            void remove(size_t element);
            void clear();

            // set intersection
            binary_set operator&(const binary_set& other_set) const;
            binary_set& operator&=(const binary_set& other_set);
            // set union
            binary_set operator|(const binary_set& other_set) const;
            binary_set& operator|=(const binary_set& other_set);
            // set difference
            binary_set operator-(const binary_set& other_set) const;
            binary_set& operator-=(const binary_set& other_set);
            // set complement
            binary_set operator!() const;

            bool operator[](size_t element) const;
            size_t size() const;

            bool operator==(const binary_set& other_set) const;
            bool operator!=(const binary_set& other_set) const;
            bool intersects(const binary_set& other_set) const;
            bool contains(const binary_set& other_set) const;

            std::vector<size_t> sparse() const;
            operator std::string() const;

            class Iterator {
                public:
                    Iterator(const binary_set* set, size_t starting_element);
                    Iterator& operator++();
                    size_t operator*() const;
                    bool operator!=(const Iterator& other) const;
                private:
                    const binary_set* set_;
                    size_t current_element_;
            };

            Iterator begin() const;
            Iterator end() const;

        private:

            std::vector<char> set_;
            size_t capacity_;

    };

    class subset_searcher {

        public:
            
            explicit subset_searcher();
            void add(size_t value, const binary_set& set);
            std::vector<size_t> find_subsets(const binary_set& set);
            ~subset_searcher();

        private:

            struct treenode {
                std::vector<size_t> values;
                treenode* left;
                treenode* right;
                treenode() { this -> left = nullptr; this -> right = nullptr; }
            }* root;

    };

    class logger {

        public:

            explicit logger() = default;
            explicit logger(const std::string& run_title, bool log_enabled, const std::string& log_name = HPLUS_DEF_LOG_FILE);

            void print(const char* str, ...) const;
            void print_info(const char* str, ...) const;
            void print_warn(const char* str, ...) const;
            void raise_error(const char* str, ...) const;

        private:

            std::string log_file_;
            bool log_enabled_;
            static void format_output_(const char* str, va_list ptr, FILE* log_file, bool print_log, bool show_time = true, bool error = false);

    };

    struct time_keeper {

        std::chrono::steady_clock::time_point timer;

        void start_timer();
        double get_time() const;

    };

    enum solution_status {
        OPT = 0,
        FEAS = 1,
        INFEAS = 2,
        NOTFOUND = 404
    };

    enum execution_status {
        STARTING = 0,
        PARSING = 1,
        PROBLEM_SIMPL = 10,
        HEURISTIC = 20,
        MODEL_BUILD = 30,
        CPX_EXECUTION = 40,
        EXITING = 100
    };

    std::vector<std::string> split_string(const std::string& str, char del);
    void assert(bool condition, const std::string& msg);
    bool isint(const std::string& str, const int from = INT_MIN, const int to = INT_MAX);
    void todo(const std::string& msg);
    void pause(const std::string msg);

}

extern my::logger mylog;
extern my::time_keeper timer;

#endif