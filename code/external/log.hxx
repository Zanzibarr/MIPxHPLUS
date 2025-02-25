/**
 * @file log.hpp
 * @brief logger with formatted output using modern C++ features
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef LOG_HXX
#define LOG_HXX

#include <chrono>
#include <cstdarg>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#define LINE "------------------------------------------------------------"
#define THICK_LINE "############################################################"

namespace {

	// Enum for log level types
	enum class log_level {
		print,
		info,
		warning,
		error
	};

	// Get formatted current time
	inline std::string get_current_time() {
		const auto		  now = std::chrono::system_clock::now();
		const auto		  time_t = std::chrono::system_clock::to_time_t(now);
		std::stringstream ss;
		ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d.%X");
		return ss.str();
	}

	// ANSI color codes
	namespace colors {
		constexpr std::string_view reset = "\033[0m";
		constexpr std::string_view info = "\033[38;5;68m\033[1m";
		constexpr std::string_view warn = "\033[38;5;215m\033[1m";
		constexpr std::string_view error = "\033[91m\033[1m";
		constexpr std::string_view reset_bold = "\033[22m";
	} // namespace colors

	inline std::pair<std::string, std::string> format_prefix(log_level level, bool include_time) {
		std::string terminal_prefix, file_prefix;
		switch (level) {
			case log_level::info:
				terminal_prefix = std::string(colors::info) + "[ INFO ] " +
					std::string(colors::reset_bold) + "-- ";
				file_prefix = "[ INFO ] -- ";
				break;
			case log_level::warning:
				terminal_prefix = std::string(colors::warn) + "[ WARN ] " +
					std::string(colors::reset_bold) + "-- ";
				file_prefix = "[ WARN ] -- ";
				break;
			case log_level::error:
				terminal_prefix = std::string(colors::error) + "[ ERROR ] " +
					std::string(colors::reset_bold) + "-- ";
				file_prefix = "[ ERROR ] -- ";
				break;
			default:
				// No prefix for regular print
				break;
		}
		if (include_time) {
			const std::string time_str = get_current_time() + " : ";
			if (!terminal_prefix.empty()) {
				terminal_prefix = terminal_prefix + time_str;
				file_prefix = file_prefix + time_str;
			} else {
				terminal_prefix = time_str;
				file_prefix = time_str;
			}
		}
		return { terminal_prefix, file_prefix };
	}

	std::string format_string(const char* format, ...) {
		va_list arg_list;
		va_start(arg_list, format);
		int size_s = std::vsnprintf(nullptr, 0, format, arg_list) + 1;
		if (size_s <= 0) {
			va_end(arg_list);
			throw std::runtime_error("error during formatting");
		}
		auto size = static_cast<size_t>(size_s);
		auto buf = std::make_unique<char[]>(size);
		std::vsnprintf(buf.get(), size, format, arg_list);
		va_end(arg_list);
		return { buf.get(), buf.get() + size - 1 };
	}
} // anonymous namespace

/**
 * @brief Modern logger class for formatted output on terminal and log file
 */
class logger {
public:
	/**
	 * @brief Construct a new logger object
	 *
	 * @param log_enabled Specify whether to log to a file
	 * @param log_name Path of the log file (writing in append mode)
	 * @param run_title Optional title for the current logging session
	 *
	 * @throw std::runtime_error If any problem with opening the log file occurs
	 */
	explicit logger(bool log_enabled, std::string_view log_name, std::string_view run_title = "")
		: log_file_path_(log_name), log_enabled_(log_enabled) {
		if (log_enabled_) {
			try {
				std::ofstream file(log_file_path_, std::ios::app);
				if (!file)
					throw std::runtime_error("Failed to open log file: " + std::string(log_name));
				if (!run_title.empty()) {
					file << '\n'
						 << THICK_LINE << "\nRUN_NAME: " << run_title << '\n'
						 << THICK_LINE << '\n';
				}
			} catch (const std::exception& e) {
				throw std::runtime_error(std::string("logger initialization error: ") + e.what());
			}
		}
	}

	/**
	 * @brief Writes a simple output (like printf)
	 *
	 * @param format Formatted string
	 * @param args Elements to be added to the formatted string
	 *
	 * @throw std::runtime_error If any problem with the log file occurs
	 */
	template <typename... Args>
	void print(const char* format, Args&&... args) const {
		log_message(log_level::print, false, format, std::forward<Args>(args)...);
	}

	/**
	 * @brief Writes an info message with timestamp
	 *
	 * @param format Formatted string
	 * @param args Elements to be added to the formatted string
	 *
	 * @throw std::runtime_error If any problem with the log file occurs
	 */
	template <typename... Args>
	void print_info(const char* format, Args&&... args) const {
		log_message(log_level::info, true, format, std::forward<Args>(args)...);
	}

	/**
	 * @brief Writes a warning message with timestamp
	 *
	 * @param format Formatted string
	 * @param args Elements to be added to the formatted string
	 *
	 * @throw std::runtime_error If any problem with the log file occurs
	 */
	template <typename... Args>
	void print_warn(const char* format, Args&&... args) const {
		log_message(log_level::warning, true, format, std::forward<Args>(args)...);
	}

	/**
	 * @brief Writes an error message with timestamp and terminates execution
	 *
	 * @param format Formatted string
	 * @param args Elements to be added to the formatted string
	 *
	 * @throw std::runtime_error If any problem with the log file occurs
	 */
	template <typename... Args>
	[[noreturn]]
	void raise_error(const char* format, Args&&... args) const {
		log_message(log_level::error, true, format, std::forward<Args>(args)...);
		std::cerr << colors::reset;
		std::exit(1);
	}

private:
	std::string log_file_path_;
	bool		log_enabled_;

	/**
	 * @brief Internal implementation for logging messages
	 */
	template <typename... Args>
	void log_message(log_level level, bool include_time, const char* format, Args&&... args) const {
		try {
			std::string message = format_string(format, std::forward<Args>(args)...);
			auto [terminal_prefix, file_prefix] = format_prefix(level, include_time);
			std::ostream& out_stream = (level == log_level::warning || level == log_level::error)
				? std::cerr
				: std::cout;
			out_stream << terminal_prefix << message << "\n";
			if (log_enabled_) {
				std::ofstream file(log_file_path_, std::ios::app);
				if (!file)
					throw std::runtime_error("Failed to open log file for writing");
				file << file_prefix << message << "\n";
			}
			if (level != log_level::print)
				std::cout << colors::reset;
		} catch (const std::exception& e) {
			std::cerr << "logger error: " << e.what() << "\n";
			if (level == log_level::error)
				std::exit(1);
		}
	}
};

#endif /* LOG_HXX */