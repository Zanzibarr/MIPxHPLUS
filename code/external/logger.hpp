#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>   // std::ofstream
#include <iomanip>   // std::setfill, std::setw
#include <iostream>  // std::cout, std::cerr, std::endl, std::ios
#include <mutex>     // std::lock_guard, std::mutex
#include <sstream>   // std::ostringstream
#include <string>    // std::string
#include <thread>    // std::this_thread
#include <utility>   // std::move

#include "timer.hpp"

// ANSI color codes - create a static instance
struct Colors {
    static constexpr const char* reset = "\033[0m";
    static constexpr const char* red = "\033[31m";
    static constexpr const char* green = "\033[32m";
    static constexpr const char* yellow = "\033[33m";
    static constexpr const char* blue = "\033[34m";
    static constexpr const char* magenta = "\033[35m";
    static constexpr const char* cyan = "\033[36m";
    static constexpr const char* white = "\033[37m";
    static constexpr const char* bright_red = "\033[91m";
    static constexpr const char* bright_green = "\033[92m";
    static constexpr const char* bright_yellow = "\033[93m";
    static constexpr const char* bright_blue = "\033[94m";
    static constexpr const char* bright_magenta = "\033[95m";
    static constexpr const char* bright_cyan = "\033[96m";
    static constexpr const char* bright_white = "\033[97m";
};

class logger {
   public:
    // Log level enumeration
    enum class level { LOG, INFO, DEBUG, WARNING, ERROR, SUCCESS };

    // Stream-like logger class for operator<< support
    class log_stream {
       private:
        logger& logger_ref_;
        level log_level_;
        std::ostringstream stream_;
        bool should_exit_;

       public:
        log_stream(logger& logger_ref, level log_level, bool should_exit = false)
            : logger_ref_(logger_ref), log_level_(log_level), should_exit_(should_exit) {}

        // Move constructor
        log_stream(log_stream&& other) noexcept
            : logger_ref_(other.logger_ref_), log_level_(other.log_level_), stream_(std::move(other.stream_)), should_exit_(other.should_exit_) {}

        // Delete copy operations
        log_stream(const log_stream&) = delete;
        log_stream& operator=(const log_stream&) = delete;
        log_stream& operator=(log_stream&&) = delete;

        template <typename T>
        log_stream& operator<<(const T& value) {
            stream_ << value;
            return *this;
        }

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

    // Get the single instance
    static logger& get_instance() {
        static logger instance;
        return instance;
    }

    // Initialize the logger (call once at program start)
    void initialize(bool write_on_file = false, const std::string& log_file_path = "", bool use_colors = true, bool show_thread = true) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            std::cerr << "logger already initialized!" << std::endl;
            _Exit(EXIT_FAILURE);
        }

        write_on_file_ = write_on_file;
        log_file_path_ = log_file_path;
        use_colors_ = use_colors;
        show_thread_ = show_thread;

        if (write_on_file_ && !log_file_path_.empty()) {
            log_file_.open(log_file_path_, std::ios::app);
            if (!log_file_.is_open()) {
                std::cerr << "Failed to open log file: " << log_file_path_ << std::endl;
                write_on_file_ = false;
            }
        }

        initialized_ = true;
    }

    // String-based logging methods (thread-safe)
    void log(const std::string& message) { log_with_level(message, "", Colors::white); }

    void info(const std::string& message) { log_with_level(message, " INFO  ", Colors::bright_green); }

    void debug(const std::string& message) { log_with_level(message, " DEBUG ", Colors::bright_blue); }

    void warning(const std::string& message) { log_with_level(message, "WARNING", Colors::bright_yellow); }

    [[noreturn]]
    void error(const std::string& message) {
        log_with_level(message, " ERROR ", Colors::bright_red);
        _Exit(EXIT_FAILURE);
    }

    void success(const std::string& message) { log_with_level(message, "SUCCESS", Colors::bright_green); }

    // Stream-based logging methods (thread-safe)
    log_stream log() { return log_stream(*this, level::LOG); }

    log_stream info() { return log_stream(*this, level::INFO); }

    log_stream debug() { return log_stream(*this, level::DEBUG); }

    log_stream warning() { return log_stream(*this, level::WARNING); }

    log_stream error() { return log_stream(*this, level::ERROR, true); }

    log_stream success() { return log_stream(*this, level::SUCCESS); }

    // Delete copy constructor and assignment operator
    logger(const logger&) = delete;
    logger& operator=(const logger&) = delete;

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

    void log_with_level(const std::string& message, const std::string& level, const char* color) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) {
            std::cerr << "logger not initialized!" << std::endl;
            _Exit(EXIT_FAILURE);
        }

        // Get current time from timer
        double elapsed_time = GET_TIME();

        std::string time_part, thread_part, level_part;
        if (!level.empty()) {
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
            level_part = "[" + level + "] ";
        }

        // File output without colors
        if (write_on_file_ && log_file_.is_open()) {
            log_file_ << time_part << thread_part << level_part << message << std::endl;
            log_file_.flush();
        } else {
            // Console output with colors
            if (use_colors_) {
                std::cout << Colors::cyan << time_part << Colors::reset << Colors::magenta << thread_part << Colors::reset << color << level_part
                          << Colors::reset << message << std::endl;
            } else {
                std::cout << time_part << thread_part << level_part << message << std::endl;
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

// Stream-based macros
#define LOG logger::get_instance().log()
#define LOG_INFO logger::get_instance().info()
#define LOG_DEBUG logger::get_instance().debug()
#define LOG_WARNING logger::get_instance().warning()
#define LOG_ERROR logger::get_instance().error()
#define LOG_SUCCESS logger::get_instance().success()

#endif  // LOGGER_HPP