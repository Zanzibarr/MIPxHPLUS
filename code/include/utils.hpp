#ifndef UTILS_H
#define UTILS_H

// ##################################################################### //
// ############################## INCLUDE ############################## //
// ##################################################################### //

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
#include <chrono>
#include <climits>
#include <cstdarg>
// #include <iterator>
// #include <functional>
// #include <unistd.h>
// #include <cplex.h>

// ##################################################################### //
// ######################### PATHS AND FOLDERS ######################### //
// ##################################################################### //

#ifndef HPLUS_HOME_DIR
#define HPLUS_HOME_DIR "cmake_handles_this" // overwritten by cmake
#endif
#define HPLUS_CODE_DIR HPLUS_HOME_DIR"/code"
#define HPLUS_INST_DIR HPLUS_CODE_DIR"/instances"
#define HPLUS_LOG_DIR HPLUS_CODE_DIR"/logs"
#define HPLUS_CPLEX_OUT_DIR HPLUS_CODE_DIR"/cpxout"

// ##################################################################### //
// ############################## DEFAULTS ############################# //
// ##################################################################### //

#define HPLUS_DEF_LOG_OPTION false
#define HPLUS_DEF_LOG_FILE "default.log"
#define HPLUS_DEF_RUN_NAME "Default run name"
#define HPLUS_DEF_TIME_LIMIT 60

// ##################################################################### //
// ############################ PARSING CLI ############################ //
// ##################################################################### //

#define HPLUS_CLI_INPUT_FILE_FLAG "-i"          // flag for parsing input file
#define HPLUS_CLI_LOG_FLAG "-l"                 // flag for using a log file
#define HPLUS_CLI_LOG_NAME_FLAG "-ln"           // flag for parsing the log name
#define HPLUS_CLI_RUN_NAME_FLAG "-rn"           // flag for parsing the run name
#define HPLUS_CLI_ALG_FLAG "-a"                 // flag for parsing the algorithm to use

#define HPLUS_CLI_IMAI "imai"
#define HPLUS_CLI_IMAI_BASE "-base"
#define HPLUS_CLI_RANKOOH "rankooh"

#define HPLUS_CLI_TIMELIMIT_FLAG "-t"           // flag for parsing the time limit

// ##################################################################### //
// ####################### PRINTING AND DEBUGGING ###################### //
// ##################################################################### //

#ifndef HPLUS_VERBOSE
#define HPLUS_VERBOSE 1     // overwritten by cmake
#endif
#ifndef HPLUS_WARN
#define HPLUS_WARN 1        // overwritten by cmake
#endif
#ifndef HPLUS_INTCHECK
#define HPLUS_INTCHECK 1    // overwritten by cmake
#endif

#define LINE "------------------------------------------"
#define THICK_LINE "##############################################"

namespace my {

    // ##################################################################### //
    // ############################## BITFIELD ############################# //
    // ##################################################################### //

    /**
     * BitField used to compactly store size bits into a single data structure
     */
    class BitField {

        public:

            explicit BitField() = default;
            explicit BitField(unsigned int num_bits, bool full_flag = false);
            BitField(const BitField& other_bitfield);

            void set(unsigned int i);
            void unset(unsigned int i);
            bool operator[](unsigned int i) const;

            unsigned int size() const;

            BitField operator&(const BitField& other_bitfield) const;        // set intersection
            BitField& operator&=(const BitField& other_bitfield);
            BitField operator|(const BitField& other_bitfield) const;        // set union
            BitField& operator|=(const BitField& other_bitfield);
            BitField operator-(const BitField& other_bitfield) const;        // set difference
            BitField& operator-=(const BitField& other_bitfield);
            BitField operator!() const;                                      // set complement

            bool operator==(const BitField& other_bitfield) const;           // wether this equals other_bitfield
            bool operator!=(const BitField& other_bitfield) const;           // wether this is not equal to other_bitfield
            bool intersects(const BitField& other_bitfield) const;           // wether this intersects other_bitfield
            bool contains(const BitField& other_bitfield) const;             // wether this contains other_bitfield

            operator std::string() const;

            // Iterator class for iterating over bits set to 1
            class Iterator {

                public:

                    Iterator(const BitField *bitField, unsigned int index) : bf_(bitField), index_(index) { if (index_ < bf_->size() && !(*bf_)[index_]) ++(*this); }

                    Iterator& operator++() {
                        do {
                            ++index_;
                        } while (index_ < bf_ -> size() && !(*bf_)[index_]);
                        return *this;
                    }

                    Iterator operator++(int) {
                        Iterator tmp = *this;
                        ++(*this);
                        return tmp;
                    }

                    unsigned int operator*() const { return index_; }

                    bool operator!=(const Iterator &other) const { return index_ != other.index_; }

                private:

                    const BitField* bf_;
                    unsigned int index_;

            };

            Iterator begin() const { return Iterator(this, 0); }
            Iterator end() const { return Iterator(this, size_); }

        private:
            std::vector<char> field_;
            unsigned int size_;

    };

    // ##################################################################### //
    // ############################### LOGGER ############################## //
    // ##################################################################### //

    /**
     * Logger used during the execution of this program
     */
    class Logger {

        public:

            Logger() = default;

            /**
             * @param run_title Title of the run to be logged
             * @param log_name (optional, def = default.log) Name of the log file
             */
            explicit Logger(const std::string& run_title, const std::string& log_name = HPLUS_DEF_LOG_FILE);

            /**
             * Code executes only if HPLUS_VERBOSE != 0
             * Prints and logs a formatted string
             */
            void print(const char* str, ...) const;

            /**
             * Code executes only if HPLUS_VERBOSE >= 10
             * Prints and logs a formatted string (info format)
             */
            void print_info(const char* str, ...) const;

            /**
             * Code executes only if WARN == 1
             * Prints and logs a formatted string (warning format)
             */
            void print_warn(const char* str, ...) const;

            /**
             * Prints and logs an unhandled error (error format)
             * Terminates the execution of the program (exit(1))
             */
            void raise_error(const char* str, ...) const;

        private:
            std::string log_file_name_;
            static void format_output_(const char* str, va_list ptr, FILE* log_file, bool show_time = true);

    };

    // ##################################################################### //
    // ############################### UTILS ############################### //
    // ##################################################################### //

    enum status {
        OPT = 0,
        TIMEL_FEAS = 1,
        USR_STOP_FEAS = 2,
        INFEAS = 100,
        TIMEL_NF = 101,
        USR_STOP_NF = 102,
        NOTFOUND = 103
    };

    /**
     * Splits a string into a vector of strings based on the specified delimiter
     */
    std::vector<std::string> split_string(const std::string& str, char del);

    /**
     * Interrupts the execution of the code if the condition is false
     */
    void assert(bool condition, const std::string message);

    /**
     * Asserts that a string is a number (if specified also checks the range (both inclusive))
     */
    bool isint(const std::string& str, const int from = INT_MIN, const int to = INT_MAX);

    /**
     * Interrupts the execution with a todo message
     */
    void todo();

}

// ##################################################################### //
// ############################ GLOBAL INFO ############################ //
// ##################################################################### //

/**
 * Environment for the execution of the code
 */
extern struct HPLUS_Environment {

    my::status status;

    /**
     * @return true/false based on wether a solution has been found (looks at the status)
    */
    bool found() const;

    int cpx_terminate;

    // Logging

    my::Logger logger;
    std::chrono::steady_clock::time_point timer;

    std::string infile;
    bool log;
    std::string log_name;
    std::string run_name;

    // Algorithm tools

    std::string alg;
    unsigned int time_limit;

    bool imai_baseline;

    // Time control

    /**
     * Starts the timer for measuring execution time
     */
    void start_timer();

    /**
     * Returns the execution time in seconds since the timer was started
     */
    double get_time() const;

} HPLUS_env;

/**
 * Statistics for the execution of the code
 */
extern struct HPLUS_Statistics {

    double parsing_time;
    double build_time;
    double exec_time;

    void print() const;

} HPLUS_stats;

#endif