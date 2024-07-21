#ifndef _UTILS_H
#define _UTILS_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <format>
#include <chrono>

// directory organization
#define HOME_DIR "/Users/matteozanella/Documents/git/thesis_master/code" //TODO: Maybe not hardcoded?
#define LOG_DIR "logs"
#define DEF_LOG_FILE "default.log"

// verbose parameter for logging
#ifndef VERBOSE
#define VERBOSE 0
#endif

// integrity checks switch
#define INTCHECKS VERBOSE>0

/**
 * Environment for the execution of the code
 */
struct Environment {

    std::string run_name;
    std::string log_name;

};

/**
 * Statistics for the execution of the code
 */
struct Statistics {

};

/**
BitField used to compactly store size bits into a single data structure
 */
class BitField {

    public:
        /**
         * Constructor of the BitField
         * 
         * @param size: number of bits the BitField has
         */
        BitField(unsigned int size);

        /**
         * Destructor of the BitField
         */
        ~BitField();
        
        /**
         * Set the i th bit to 1
         * 
         * @param i: Index of the bit to set to 1
         */
        void set(const unsigned int i);

        /**
         * Set the i th bit to 0
         * 
         * @param i: Index of the bit to set to 0
         */
        void unset(const unsigned int i);

        /**
         * Access the i th bit
         * 
         * @param i: Index of the bit to access
         * @return The value of the bit in i th position
         */
        bool operator [] (const unsigned int i) const;

        /**
         * Bitwise and of two BitFields
         * 
         * @param bf: BitField to do the and with
         * @return The resulting BitField
         */
        BitField operator & (const BitField bf) const;

        /**
         * Returns the size of the BitField (in bits)
         * 
         * @return The number of bits the BitField contains
         */
        unsigned int size() const;

        std::string view() const;

    private:
        char* field;
        unsigned int len;

};

/**
 * Logger used during the execution of this program
 */
class Logger {

    public:
        /**
         * Constructor for the logger (to be called at the start of the program)
         * 
         * @param run_title Title of the run to be logged
         * @param log_name (optional, def = default.log) Name of the log file
         */
        Logger(const std::string run_title, const std::string log_name = DEF_LOG_FILE);
        /**
         * Starts the timer for logging execution time
         */
        void reset_timer();
        /**
         * Code executes only if VERBOSE >= 10
         * Prints and logs a formatted string (info format)
         */
        void print_info(const char* str, ...) const;
        /**
         * Code executes only if VERBOSE >= 1
         * Prints and logs a formatted string (warning format)
         */
        void print_warn(const char* str, ...) const;
        /**
         * Prints and logs an unhandled error (error format)
         * Terminates the execution of the program (exit(1))
         */
        void raise_error(const char* str, ...) const;

    private:
        char log_file_name[100];
        std::chrono::steady_clock::time_point s_timer;
        void _format_output(const char* str, va_list ptr, FILE* log_file) const;
        
};

/**
 * Utils functions
 */
namespace my {

    /**
     * Splits a string into a vector of strings based on the specified delimiter
     */
    void split(const std::string str, std::vector<std::string>* tokens, const char del);

    /**
     * Asserts that the value obtained is equal to the expected one
     */
    void asserteq(const std::string value, const std::string expected, const Logger* logger);

    /**
     * Asserts that the value obtained is equal to the expected one
     */
    void asserteq(const int value, const int expected, const Logger* logger);

    /**
     * Asserts that a string is a number (if specified also checks the range (both inclusive))
     */
    void assertisint(const std::string str, const Logger* logger, const int from = INT_MIN, const int to = INT_MAX);

}

#endif