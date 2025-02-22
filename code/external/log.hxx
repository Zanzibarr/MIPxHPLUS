/**
 * @file log.hpp
 * @brief Logger with formatted output
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef LOG_H
#define LOG_H

#ifndef _ASSERT
#define _ASSERT(cond) {                                                                                     \
if (!(cond)) {                                                                                              \
    std::cerr << "Assert check failed at " << __func__ << "(): " << __FILE__ << ":"<< __LINE__ << "\n";     \
    exit(1);                                                                                                \
}                                                                                                           \
}
#endif

#include <iostream>
#include <string>
#include <time.h>
#include <cstdarg>

#define LINE "------------------------------------------------------------"
#define THICK_LINE "############################################################"

class logger {
    public:
        /** Create a new logger with title run _run_title. If _log_enabled is set to true, it will write (append mode) on the file found at the path _log_name (created a new one if none found) */
        logger(const std::string& _run_title, bool _log_enabled, const std::string& _log_name) : log_file(_log_name), log_enabled(_log_enabled) {
            if (log_enabled)  {
                FILE* log_file = fopen(this->log_file.c_str(), "a");
                fprintf(log_file, "\n%s\nRUN_NAME: %s\n%s\n", THICK_LINE, _run_title.c_str(), THICK_LINE);
                fclose(log_file);
            }

        }
        /** Writes a simple non-formatted message */
        inline void print(const char* _str, ...) const {
            #if HPLUS_VERBOSE == 0
            return;
            #endif
            FILE* log_file = nullptr;
            if (this->log_enabled) log_file = fopen(this->log_file.c_str(), "a");
            va_list ptr;
            va_start(ptr, _str);
            // Use one va_list for each vfprintf call
            if (this->log_enabled) {
                va_list ptr_copy;
                va_copy(ptr_copy, ptr);
                vfprintf(log_file, _str, ptr_copy);
                va_end(ptr_copy);
            }
            vfprintf(stdout, _str, ptr);
            fprintf(stdout, "\n");
            if (this->log_enabled) {
                fprintf(log_file, "\n");
                fclose(log_file);
            }
            va_end(ptr);
        }
        /** Writes a info-type message */
        inline void print_info(const char* _str, ...) const {
            FILE* log_file = nullptr;
            if (this->log_enabled) log_file = fopen(this->log_file.c_str(), "a");
            va_list ptr;
            va_start(ptr, _str);
            // Print prefix
            fprintf(stdout, "\033[92m\033[1m[ INFO ]\033[0m -- ");
            if (this->log_enabled) fprintf(log_file, "[ INFO ] -- ");
            time_t now = time(0);
            struct tm tstruct;
            char str_time[80];
            tstruct = *localtime(&now);
            strftime(str_time, sizeof(str_time), "%Y-%m-%d.%X", &tstruct);
            fprintf(stdout, "%s : ", str_time);
            if (this->log_enabled) fprintf(log_file, "%s : ", str_time);
            // Use one va_list for each vfprintf call
            if (this->log_enabled) {
                va_list ptr_copy;
                va_copy(ptr_copy, ptr);
                vfprintf(log_file, _str, ptr_copy);
                va_end(ptr_copy);
            }
            vfprintf(stdout, _str, ptr);
            fprintf(stdout, "\n");
            if (this->log_enabled) {
                fprintf(log_file, "\n");
                fclose(log_file);
            }
            va_end(ptr);
        }
        /** Writes a warning-type message */
        inline void print_warn(const char* _str, ...) const {
            FILE* log_file = nullptr;
            if (this->log_enabled) log_file = fopen(this->log_file.c_str(), "a");
            va_list ptr;
            va_start(ptr, _str);
            // Print prefix
            fprintf(stdout, "\033[93m\033[1m[ WARN ]\033[0m -- ");
            if (this->log_enabled) fprintf(log_file, "[ WARN ] -- ");
            time_t now = time(0);
            struct tm tstruct;
            char str_time[80];
            tstruct = *localtime(&now);
            strftime(str_time, sizeof(str_time), "%Y-%m-%d.%X", &tstruct);
            fprintf(stdout, "%s : ", str_time);
            if (this->log_enabled) fprintf(log_file, "%s : ", str_time);
            // Use one va_list for each vfprintf call
            if (this->log_enabled) {
                va_list ptr_copy;
                va_copy(ptr_copy, ptr);
                vfprintf(log_file, _str, ptr_copy);
                va_end(ptr_copy);
            }
            vfprintf(stdout, _str, ptr);
            fprintf(stdout, "\n");
            if (this->log_enabled) {
                fprintf(log_file, "\n");
                fclose(log_file);
            }
            va_end(ptr);
        }
        /** Writes an error message and terminates the execution with status code 1 */
        inline void raise_error(const char* _str, ...) const {
            FILE* log_file = nullptr;
            if (this->log_enabled) log_file = fopen(this->log_file.c_str(), "a");
            va_list ptr;
            va_start(ptr, _str);
            // Print prefix to stderr for errors
            fprintf(stderr, "\033[91m\033[1m[ ERROR ]\033[0m -- ");
            if (this->log_enabled) fprintf(log_file, "[ ERROR ] -- ");
            // Print timestamp
            time_t now = time(0);
            struct tm tstruct;
            char str_time[80];
            tstruct = *localtime(&now);
            strftime(str_time, sizeof(str_time), "%Y-%m-%d.%X", &tstruct);
            fprintf(stderr, "%s : ", str_time);
            if (this->log_enabled) fprintf(log_file, "%s : ", str_time);
            // Use one va_list for each vfprintf call
            if (this->log_enabled) {
                va_list ptr_copy;
                va_copy(ptr_copy, ptr);
                vfprintf(log_file, _str, ptr_copy);
                va_end(ptr_copy);
            }
            vfprintf(stderr, _str, ptr);
            fprintf(stderr, "\n");
            if (this->log_enabled) {
                fprintf(log_file, "\n");
                fclose(log_file);
            }
            va_end(ptr);
            exit(1);
        }
    protected:
        std::string log_file;
        bool log_enabled;
        inline void format(const char* _str, va_list _ptr, FILE* _log_file, bool _print_log, bool _show_time = true, bool _error = false) const {
            if (_show_time) {
                time_t now = time(0);
                struct tm tstruct;
                char str_time[80];
                tstruct = *localtime(&now);
                strftime(str_time, sizeof(str_time), "%Y-%m-%d.%X", &tstruct);
                fprintf(_error ? stderr : stdout, "%s : ", str_time);
                if (_print_log) fprintf(_log_file, "%s : ", str_time);
            }
            // vfprintf to handle all formatting
            vfprintf(_error ? stderr : stdout, _str, _ptr);
            if (_print_log) {
                // Need to make a copy of args since vfprintf consumes them
                va_list ptr_copy;
                va_copy(ptr_copy, _ptr);
                vfprintf(_log_file, _str, ptr_copy);
                va_end(ptr_copy);
            }
        }


};
                    
#endif /* LOG_H */