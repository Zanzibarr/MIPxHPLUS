/**
 * @file log.hpp
 * @brief Logger with formatted output
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef LOG_H
#define LOG_H

#include <cstdarg>
#include <stdexcept>
#include <string>
#include <time.h>

#define LINE "------------------------------------------------------------"
#define THICK_LINE "############################################################"

static inline void printf_logger(va_list& _valist, const char* _msg, bool _time, FILE* _out, FILE* _logfile) {
	if (_time) {
		time_t	  now = time(0);
		struct tm tstruct;
		char	  str_time[80];
		tstruct = *localtime(&now);
		strftime(str_time, sizeof(str_time), "%Y-%m-%d.%X", &tstruct);
		fprintf(_out, "%s : ", str_time);
		if (_logfile)
			fprintf(_logfile, "%s : ", str_time);
	}
	if (_logfile) {
		// Use one va_list for each vfprintf call
		va_list ptr_copy;
		va_copy(ptr_copy, _valist);
		vfprintf(_logfile, _msg, ptr_copy);
		va_end(ptr_copy);
	}
	vfprintf(_out, _msg, _valist);
	fprintf(_out, "\n");
	if (_logfile) {
		fprintf(_logfile, "\n");
		fclose(_logfile);
	}
	va_end(_valist);
}

/**
 * @brief logger class for logging formatted output both on terminal and on log file
 *
 */
class logger {
public:
	/**
	 * @brief Construct a new logger object
	 *
	 * @param _log_enabled Specify wether to log on a file too
	 * @param _log_name Specify the path of the log file (writing on append mode: 'a')
	 * @param _run_title Title of the run to be logged on the file (optional)
	 *
	 * @throw std::invalid_argument If any problem with opening the log file occurrs
	 */
	explicit inline logger(bool _log_enabled, const std::string& _log_name, const std::string& _run_title = "")
		: log_file(_log_name), log_enabled(_log_enabled) {
		if (this->log_enabled) {
			FILE* file = fopen(this->log_file.c_str(), "a");
			if (!file)
				throw std::invalid_argument("An error occurred while opening the file.");
			if (_run_title != "")
				fprintf(file, "\n%s\nRUN_NAME: %s\n%s\n", THICK_LINE, _run_title.c_str(), THICK_LINE);
			fclose(file);
		}
	}

	/**
	 * @brief Writes a simple output (consider this as a printf function)
	 *
	 * @param _str Formatted string
	 * @param ... Elements to be added to the formatted string
	 *
	 * @throw std::invalid_argument If any problem with opening the log file occurrs
	 */
	inline void print(const char* _str, ...) const {
		FILE* file = nullptr;
		if (this->log_enabled) {
			file = fopen(this->log_file.c_str(), "a");
			if (!file)
				throw std::invalid_argument("An error occurred while opening the file.");
		}
		va_list ptr;
		va_start(ptr, _str);
		printf_logger(ptr, _str, false, stdout, file);
	}

	/**
	 * @brief Writes an info message (consider this as a printf function)
	 *
	 * @param _str Formatted string
	 * @param ... Elements to be added to the formatted string
	 *
	 * @throw std::invalid_argument If any problem with opening the log file occurrs
	 */

	inline void print_info(const char* _str, ...) const {
		FILE* file = nullptr;
		if (this->log_enabled) {
			file = fopen(this->log_file.c_str(), "a");
			if (!file)
				throw std::invalid_argument("An error occurred while opening the file.");
		}
		va_list ptr;
		va_start(ptr, _str);
		// Print prefix
		fprintf(stdout, "\033[38;5;68m\033[1m[ INFO ] \033[22m-- ");
		if (file)
			fprintf(file, "[ INFO ] -- ");
		printf_logger(ptr, _str, true, stdout, file);
		fprintf(stdout, "\033[0m");
	}

	/**
	 * @brief Writes a warning message (consider this as a printf function)
	 *
	 * @param _str Formatted string
	 * @param ... Elements to be added to the formatted string
	 *
	 * @throw std::invalid_argument If any problem with opening the log file occurrs
	 */
	inline void print_warn(const char* _str, ...) const {
		FILE* file = nullptr;
		if (this->log_enabled) {
			file = fopen(this->log_file.c_str(), "a");
			if (!file)
				throw std::invalid_argument("An error occurred while opening the file.");
		}
		va_list ptr;
		va_start(ptr, _str);
		// Print prefix
		fprintf(stderr, "\033[38;5;215m\033[1m[ WARN ] \033[22m-- ");
		if (file)
			fprintf(file, "[ WARN ] -- ");
		printf_logger(ptr, _str, true, stderr, file);
		fprintf(stdout, "\033[0m");
	}

	/**
	 * @brief Writes an error message (consider this as a printf function) and terminates the execution of the code with exit(1)
	 *
	 * @param _str Formatted string
	 * @param ... Elements to be added to the formatted string
	 *
	 * @throw std::invalid_argument If any problem with opening the log file occurrs
	 */
	inline void raise_error(const char* _str, ...) const {
		FILE* file = nullptr;
		if (this->log_enabled) {
			file = fopen(this->log_file.c_str(), "a");
			if (!file)
				throw std::invalid_argument("An error occurred while opening the file.");
		}
		va_list ptr;
		va_start(ptr, _str);
		// Print prefix to stderr for errors
		fprintf(stderr, "\033[91m\033[1m[ ERROR ] \033[22m-- ");
		if (file)
			fprintf(file, "[ ERROR ] -- ");
		// Print timestamp
		printf_logger(ptr, _str, true, stderr, file);
		fprintf(stdout, "\033[0m");
		exit(1);
	}

private:
	std::string log_file;
	bool		log_enabled;
};

#endif /* LOG_H */