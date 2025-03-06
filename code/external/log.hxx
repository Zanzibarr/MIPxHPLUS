/**
 * @file log.hxx
 * @brief logger with formatted output using modern C++ features
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef LOG_HXX
#define LOG_HXX

#include <cstdarg>      // For va_list and related functions
#include <ctime>        // For time functions
#include <fstream>      // For file operations
#include <iostream>     // For console output
#include <memory>       // For std::make_unique
#include <stdexcept>    // For std::runtime_error
#include <string>       // For std::string
#include <string_view>  // For std::string_view

#define LINE "------------------------------------------------------------"
#define THICK_LINE \
    "############################################################"

// Enum for log level types
enum class log_level { print, info, warning, error };

// Get formatted current time using C time functions
inline std::string get_current_time() {
    const time_t time_now{std::time(nullptr)};

    char buffer[26];
    struct tm timeinfo{};
    localtime_r(&time_now, &timeinfo);

    strftime(buffer, sizeof(buffer), "%Y-%m-%d.%X", &timeinfo);
    return {buffer};
}

// ANSI color codes
namespace colors {
constexpr std::string_view reset{"\033[0m"};
constexpr std::string_view info{"\033[38;5;68m\033[1m"};
constexpr std::string_view warn{"\033[38;5;215m\033[1m"};
constexpr std::string_view error{"\033[91m\033[1m"};
constexpr std::string_view reset_bold{"\033[22m"};
}  // namespace colors

inline std::pair<std::string, std::string> format_prefix(
    const log_level level, const bool include_time) {
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
        const std::string& time_str{get_current_time() + " : "};
        if (!terminal_prefix.empty()) {
            terminal_prefix = terminal_prefix + time_str;
            file_prefix = file_prefix + time_str;
        } else {
            terminal_prefix = time_str;
            file_prefix = time_str;
        }
    }
    return {terminal_prefix, file_prefix};
}

inline std::string format_string(const char* format, va_list args) {
    // Make a copy of the va_list to avoid potential corruption
    va_list args_copy;
    va_copy(args_copy, args);

    // Get required buffer size
    const int size_s{std::vsnprintf(nullptr, 0, format, args_copy) + 1};
    va_end(args_copy);

    if (size_s <= 0) throw std::runtime_error("Error during formatting");

    const size_t size{static_cast<size_t>(size_s)};
    const auto buf{std::make_unique<char[]>(size)};
    std::vsnprintf(buf.get(), size, format, args);
    return {buf.get(), size - 1};
}

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
    explicit logger(const bool log_enabled, const std::string_view log_name,
                    const std::string_view run_title = "")
        : log_file_path_(log_name), log_enabled_(log_enabled) {
        if (log_enabled_) {
            try {
                std::ofstream file(log_file_path_.c_str(), std::ios::app);
                if (!file.is_open())
                    throw std::runtime_error("Failed to open log file: " +
                                             std::string(log_name));

                if (!run_title.empty()) {
                    file << '\n'
                         << THICK_LINE << "\nRUN_NAME: " << run_title << '\n'
                         << THICK_LINE << '\n';
                }
                file.close();
            } catch (const std::exception& e) {
                throw std::runtime_error(
                    std::string("logger initialization error: ") + e.what());
            }
        }
    }

    /**
     * @brief Writes a simple output (like printf)
     *
     * @param msg Formatted string
     * @param ... Elements to be added to the formatted string
     *
     * @throw std::runtime_error If any problem with the log file occurs
     */
    void print(const char* msg, ...) const {
        va_list args;
        va_start(args, msg);
        log_message(log_level::print, false, msg, args);
        va_end(args);
    }

    /**
     * @brief Writes an info message with timestamp
     *
     * @param msg Formatted string
     * @param ... Elements to be added to the formatted string
     *
     * @throw std::runtime_error If any problem with the log file occurs
     */
    void print_info(const char* msg, ...) const {
        va_list args;
        va_start(args, msg);
        log_message(log_level::info, true, msg, args);
        va_end(args);
    }

    /**
     * @brief Writes a warning message with timestamp
     *
     * @param msg Formatted string
     * @param ... Elements to be added to the formatted string
     *
     * @throw std::runtime_error If any problem with the log file occurs
     */
    void print_warn(const char* msg, ...) const {
        va_list args;
        va_start(args, msg);
        log_message(log_level::warning, true, msg, args);
        va_end(args);
    }

    /**
     * @brief Writes an error message with timestamp and terminates execution
     *
     * @param msg Formatted string
     * @param ... Elements to be added to the formatted string
     *
     * @throw std::runtime_error If any problem with the log file occurs
     */
    [[noreturn]]
    void raise_error(const char* msg, ...) const {
        va_list args;
        va_start(args, msg);
        log_message(log_level::error, true, msg, args);
        va_end(args);
        std::cerr << colors::reset;
        std::exit(1);
    }

   private:
    std::string log_file_path_;
    bool log_enabled_;

    /**
     * @brief Internal implementation for logging messages
     */
    void log_message(const log_level level, const bool include_time,
                     const char* format, va_list args) const {
        try {
            std::string message{format_string(format, args)};
            auto [terminal_prefix,
                  file_prefix]{format_prefix(level, include_time)};

            std::ostream& out_stream =
                (level == log_level::warning || level == log_level::error)
                    ? std::cerr
                    : std::cout;
            out_stream << terminal_prefix << message << std::endl;

            if (log_enabled_) {
                std::ofstream file(log_file_path_.c_str(), std::ios::app);
                if (!file.is_open())
                    throw std::runtime_error(
                        "Failed to open log file for writing: " +
                        log_file_path_);

                file << file_prefix << message << std::endl;
                file.close();
            }

            if (level != log_level::print) out_stream << colors::reset;

        } catch (const std::exception& e) {
            std::cerr << "logger error: " << e.what() << std::endl;
            if (level == log_level::error) std::exit(1);
        }
    }
};

#endif /* LOG_HXX */