/**
 * @file logger.hxx
 * @brief Class for logging to stdout/file with time and threads info
 * @version 1.0.0
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 * @see https://github.com/Zanzibarr/logger
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LOGGER_HXX
#define LOGGER_HXX

#include <fstream>    // std::ofstream
#include <iomanip>    // std::setfill, std::setw
#include <iostream>   // std::cout, std::cerr, std::endl, std::ios
#include <mutex>      // std::lock_guard, std::mutex
#include <sstream>    // std::ostringstream
#include <stdexcept>  // std::runtime_error
#include <string>     // std::string
#include <thread>     // std::this_thread
#include <utility>    // std::move

#include "timer.hxx"

/**
 * @brief Provides ANSI escape codes for coloring console output.
 */
struct Colors {
    static constexpr const char *reset = "\033[0m";
    static constexpr const char *red = "\033[31m";
    static constexpr const char *green = "\033[32m";
    static constexpr const char *yellow = "\033[33m";
    static constexpr const char *blue = "\033[34m";
    static constexpr const char *magenta = "\033[35m";
    static constexpr const char *cyan = "\033[36m";
    static constexpr const char *white = "\033[37m";
    static constexpr const char *bright_red = "\033[91m";
    static constexpr const char *bright_green = "\033[92m";
    static constexpr const char *bright_yellow = "\033[93m";
    static constexpr const char *bright_blue = "\033[94m";
    static constexpr const char *bright_magenta = "\033[95m";
    static constexpr const char *bright_cyan = "\033[96m";
    static constexpr const char *bright_white = "\033[97m";
};

/**
 * @brief Singleton Logger class for logging messages with various severity levels,
 * timestamps, and thread IDs to stdout and/or a file.
 *
 * This class ensures thread-safe logging operations.
 */
class logger {
   public:
    /**
     * @brief Log level enumeration.
     */
    enum class level { LOG, INFO, DEBUG, WARNING, ERROR, SUCCESS };

    /**
     * @brief Stream-like logger class that supports `operator<<` for building log messages.
     * This class uses RAII (Resource Acquisition Is Initialization) to ensure the log message
     * is processed upon destruction of the `log_stream` object.
     */
    class log_stream {
       private:
        logger &logger_ref_;
        level log_level_;
        std::ostringstream stream_;
        bool should_exit_;

       public:
        /**
         * @brief Constructs a log_stream.
         * @param logger_ref Reference to the parent logger instance.
         * @param log_level The severity level of this log message.
         * @param should_exit If true, the program will terminate immediately after logging an ERROR.
         */
        log_stream(logger &logger_ref, level log_level, bool should_exit = false)
            : logger_ref_(logger_ref), log_level_(log_level), should_exit_(should_exit) {}

        /**
         * @brief Move constructor for log_stream.
         * Enables efficient transfer of ownership for temporary `log_stream` objects.
         * @param other The log_stream object to move from.
         */
        log_stream(log_stream &&other) noexcept
            : logger_ref_(other.logger_ref_), log_level_(other.log_level_), stream_(std::move(other.stream_)), should_exit_(other.should_exit_) {}

        /**
         * @brief Deleted copy constructor to prevent copying.
         * The `log_stream` is not designed to be copied, only moved.
         */
        log_stream(const log_stream &) = delete;
        /**
         * @brief Deleted copy assignment operator to prevent copying.
         * The `log_stream` is not designed to be copied, only moved.
         */
        log_stream &operator=(const log_stream &) = delete;
        /**
         * @brief Deleted move assignment operator to prevent move assignment.
         * This prevents unintended reassignment of `log_stream` objects.
         */
        log_stream &operator=(log_stream &&) = delete;

        /**
         * @brief Overloads the stream insertion operator to allow chaining of values.
         * This enables a natural, stream-like syntax for building log messages.
         * @tparam T Type of the value to log.
         * @param value The value to log.
         * @return Reference to the log_stream for chaining further insertions.
         */
        template <typename T>
        log_stream &operator<<(const T &value) {
            stream_ << value;
            return *this;
        }

        /**
         * @brief Destructor for log_stream.
         * Automatically flushes the collected message to the logger with the appropriate level
         * when the `log_stream` object goes out of scope.
         * If `should_exit_` is true and the level is ERROR, the program terminates immediately using `_Exit`.
         */
        ~log_stream() {
            std::string message = stream_.str();
            if (!message.empty()) {
                switch (log_level_) {
                    case level::LOG:
                        logger_ref_.log_with_level(message, "", Colors::white);
                        break;
                    case level::INFO:
                        logger_ref_.log_with_level(message, " INFO  ", Colors::bright_green);
                        break;
                    case level::DEBUG:
                        logger_ref_.log_with_level(message, " DEBUG ", Colors::bright_blue);
                        break;
                    case level::WARNING:
                        logger_ref_.log_with_level(message, "WARNING", Colors::bright_yellow);
                        break;
                    case level::ERROR:
                        logger_ref_.log_with_level(message, " ERROR ", Colors::bright_red);
                        if (should_exit_) {
                            _Exit(EXIT_FAILURE);
                        }
                        break;
                    case level::SUCCESS:
                        logger_ref_.log_with_level(message, "SUCCESS", Colors::bright_green);
                        break;
                }
            }
        }
    };

    /**
     * @brief Returns the single instance of the logger (Singleton pattern).
     * This ensures that only one logger object exists throughout the application.
     * @return Reference to the logger instance.
     */
    static logger &get_instance() {
        static logger instance;
        return instance;
    }

    /**
     * @brief Initializes the logger with desired settings.
     * This method must be called once at program startup before any logging operations are performed.
     * Calling it multiple times, or if it fails to open the log file, will result in a `std::runtime_error`.
     * @param write_on_file Whether to write logs to a file. Defaults to `false`.
     * @param log_file_path The path to the log file (ignored if `write_on_file` is `false`). Defaults to an empty string.
     * @param use_colors Whether to use ANSI color codes for console output. Defaults to `true`.
     * @param show_thread Whether to include the thread ID in log messages. Defaults to `true`.
     * @throws std::runtime_error if the logger is already initialized or if the log file cannot be opened.
     */
    void initialize(bool write_on_file = false, const std::string &log_file_path = "", bool use_colors = true, bool show_thread = true) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            throw std::runtime_error("logger already initialized!");
        }

        write_on_file_ = write_on_file;
        log_file_path_ = log_file_path;
        use_colors_ = use_colors;
        show_thread_ = show_thread;

        if (write_on_file_ && !log_file_path_.empty()) {
            log_file_.open(log_file_path_, std::ios::app);
            if (!log_file_.is_open()) {
                throw std::runtime_error("Failed to open log file: " + log_file_path_);
            }
        }

        initialized_ = true;
    }

    /**
     * @brief Logs a message at the LOG level using a string.
     * This method is thread-safe.
     * @param message The message to log.
     */
    void log(const std::string &message) { log_with_level(message, "", Colors::white); }

    /**
     * @brief Logs a message at the INFO level using a string.
     * This method is thread-safe.
     * @param message The message to log.
     */
    void info(const std::string &message) { log_with_level(message, " INFO  ", Colors::bright_green); }

    /**
     * @brief Logs a message at the DEBUG level using a string.
     * This method is thread-safe.
     * @param message The message to log.
     */
    void debug(const std::string &message) { log_with_level(message, " DEBUG ", Colors::bright_blue); }

    /**
     * @brief Logs a message at the WARNING level using a string.
     * This method is thread-safe. Console output is directed to `std::cerr`.
     * @param message The message to log.
     */
    void warning(const std::string &message) { log_with_level(message, "WARNING", Colors::bright_yellow); }

    /**
     * @brief Logs a message at the ERROR level using a string and terminates the program immediately.
     * This method is thread-safe. Console output is directed to `std::cerr`.
     * It uses `_Exit(EXIT_FAILURE)` for immediate termination without calling destructors,
     * which is crucial for critical error handling where normal shutdown is not guaranteed.
     * @param message The message to log.
     */
    [[noreturn]]
    void error(const std::string &message) {
        log_with_level(message, " ERROR ", Colors::bright_red);
        _Exit(EXIT_FAILURE);  // Intentional immediate exit without calling destructors
    }

    /**
     * @brief Logs a message at the SUCCESS level using a string.
     * This method is thread-safe.
     * @param message The message to log.
     */
    void success(const std::string &message) { log_with_level(message, "SUCCESS", Colors::bright_green); }

    /**
     * @brief Returns a log_stream for stream-based logging at the LOG level.
     * @return A temporary `log_stream` object.
     */
    log_stream log() { return log_stream(*this, level::LOG); }

    /**
     * @brief Returns a log_stream for stream-based logging at the INFO level.
     * @return A temporary `log_stream` object.
     */
    log_stream info() { return log_stream(*this, level::INFO); }

    /**
     * @brief Returns a log_stream for stream-based logging at the DEBUG level.
     * @return A temporary `log_stream` object.
     */
    log_stream debug() { return log_stream(*this, level::DEBUG); }

    /**
     * @brief Returns a log_stream for stream-based logging at the WARNING level.
     * Console output will be directed to `std::cerr`.
     * @return A temporary `log_stream` object.
     */
    log_stream warning() { return log_stream(*this, level::WARNING); }

    /**
     * @brief Returns a log_stream for stream-based logging at the ERROR level,
     * which will terminate the program immediately after logging using `_Exit`.
     * Console output will be directed to `std::cerr`.
     * @return A temporary `log_stream` object.
     */
    log_stream error() { return log_stream(*this, level::ERROR, true); }

    /**
     * @brief Returns a log_stream for stream-based logging at the SUCCESS level.
     * @return A temporary `log_stream` object.
     */
    log_stream success() { return log_stream(*this, level::SUCCESS); }

    /**
     * @brief Deleted copy constructor to prevent copying and enforce the singleton pattern.
     */
    logger(const logger &) = delete;
    /**
     * @brief Deleted copy assignment operator to prevent copying and enforce the singleton pattern.
     */
    logger &operator=(const logger &) = delete;

    /**
     * @brief Destructor for the logger.
     * This method is thread-safe and ensures that the log file is properly closed
     * when the logger instance is destroyed.
     */
    ~logger() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    // Make log_stream a friend so it can access log_with_level
    friend class log_stream;

   private:
    logger() = default;

    void log_with_level(const std::string &message, const std::string &level_str, const char *color) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) {
            throw std::runtime_error("logger not initialized!");
        }

        // Get current time from timer
        double elapsed_time = GET_TIME();

        std::string time_part, thread_part, level_part;
        if (!level_str.empty()) {
            // Format time as hh:mm:ss.ms
            std::ostringstream time_stream;

            // Convert elapsed_time to total milliseconds
            int total_ms = static_cast<int>(elapsed_time * 1000);

            // Extract components
            int hours = total_ms / 3600000;
            int minutes = (total_ms % 3600000) / 60000;
            int seconds = (total_ms % 60000) / 1000;
            int milliseconds = total_ms % 1000;

            // Format as hh:mm:ss.ms
            time_stream << std::setfill('0') << std::setw(2) << hours << ":" << std::setw(2) << minutes << ":" << std::setw(2) << seconds << "."
                        << std::setw(3) << milliseconds;

            // Create formatted message parts
            time_part = "[" + time_stream.str() + "] ";
            if (show_thread_) {  // Get thread ID for multi-threaded logging
                std::ostringstream thread_stream;
                thread_stream << std::this_thread::get_id();
                std::string thread_id = thread_stream.str();
                // Truncate thread ID to last 4 characters for readability
                if (thread_id.length() > 4) {
                    thread_id = thread_id.substr(thread_id.length() - 4);
                }
                thread_part = "[T:" + thread_id + "] ";
            }
            level_part = "[" + level_str + "] ";
        }

        // Determine output stream for console: std::cerr for WARNING/ERROR, std::cout for others.
        std::ostream &os = (level_str == "WARNING" || level_str == " ERROR ") ? std::cerr : std::cout;

        // Write to file if enabled and file is open (without colors)
        if (write_on_file_ && log_file_.is_open()) {
            log_file_ << time_part << thread_part << level_part << message << std::endl;
            log_file_.flush();  // Ensure log is written immediately
        } else {
            // Write to console with or without colors based on configuration
            if (use_colors_) {
                os << Colors::cyan << time_part << Colors::reset << Colors::magenta << thread_part << Colors::reset << color << level_part
                   << Colors::reset << message << std::endl;
            } else {
                os << time_part << thread_part << level_part << message << std::endl;
            }
        }
    }

    mutable std::mutex mutex_;
    bool initialized_ = false;
    bool write_on_file_ = false;
    bool use_colors_ = true;
    bool show_thread_ = true;
    std::string log_file_path_;
    std::ofstream log_file_;
};

// Stream-based macros for convenient logging. These provide a shorthand for accessing the logger instance and its stream-based logging methods.
#define LOG logger::get_instance().log()
#define LOG_INFO logger::get_instance().info()
#define LOG_DEBUG logger::get_instance().debug()
#define LOG_WARNING logger::get_instance().warning()
#define LOG_ERROR logger::get_instance().error()
#define LOG_SUCCESS logger::get_instance().success()

/**
 * @brief Macro for logging unimplemented functionality at ERROR level, terminating the program.
 * Includes the function name, file name, and line number for easy identification.
 */
#define LOG_TODO LOG_ERROR << __func__ << "(): " << __FILE__ << ":" << __LINE__ << " : unimplemented "
/**
 * @brief Macro for logging unimplemented functionality at WARNING level.
 * Includes the function name, file name, and line number for easy identification.
 */
#define LOG_TODO_WARN LOG_WARNING << __func__ << "(): " << __FILE__ << ":" << __LINE__ << " : unimplemented "

#endif