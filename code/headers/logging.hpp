#ifndef _LOGGING_H
#define _LOGGING_H

#include "utils.hpp"

class Logger {
    private:
        char log_file_name[100];
        void _format_output(const char* str, va_list ptr, FILE* log_file);

    public:
        /**
         * Constructor for the logger (to be called at the start of the program)
         * 
         * @param run_title Title of the run to be logged
         * @param log_name (optional, def = default.log) Name of the log file
         */
        Logger(const char* run_title, std::string log_name = DEF_LOG_FILE);
        /**
         * Code executes only if VERBOSE >= 10
         * Prints and logs a formatted string (info format)
         */
        void print_info(const char* str, ...);
        /**
         * Code executes only if VERBOSE >= 1
         * Prints and logs a formatted string (warning format)
         */
        void print_warn(const char* str, ...);
        /**
         * Prints and logs an unhandled error (error format)
         * Terminates the execution of the program (exit(1))
         */
        void raise_error(const char* str, ...);
};

#endif