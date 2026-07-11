#pragma once

/**
 * @file logger.hxx
 * @brief Thread-safe logger with synchronous and asynchronous output modes.
 * @version 2.2.2
 *
 * @details
 * `Logger` supports two operating modes selected at construction:
 *   - **Sync** — every `log()` call writes immediately on the calling thread
 *     (protected by a mutex).
 *   - **Async** — log records are pushed onto a lock-free queue and drained
 *     by a dedicated background thread, minimising latency on the hot path.
 *
 * Seven severity levels: `DEBUG`, `INFO`, `WARNING`, `RAW`, `SUCCESS`, `ERROR`, `FATAL`.
 *
 * Output to stdout/stderr and to a file are independently controlled —
 * both can be active simultaneously.  ANSI colors apply only to console output.
 *
 * Construction starts the internal timer automatically.  An externally-created
 * `steady_clock::time_point` may be passed so that elapsed times are measured
 * from a point prior to the Logger's own construction.
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * Repository: https://github.com/Zanzibarr/cpp_utils
 *
 * SPDX-License-Identifier: MIT
 */

#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#ifdef __APPLE__
#include <mach/mach.h>
#else
#include <unistd.h>  // sysconf(_SC_PAGESIZE)
#endif

#include "ansi_colors.hxx"

namespace utilz {
// ── LoggerLevel ───────────────────────────────────────────────────────────────

/**
 * @brief Severity / verbosity levels ordered by printing priority.
 *
 * The numeric ordering reflects how "chatty" a level is, not how critical it
 * is.  The filter rule is simply `level >= min_level`:
 *
 *   min_level = DEBUG   → everything printed
 *   min_level = INFO    → INFO and above  (default)
 *   min_level = ERROR   → only ERROR and FATAL
 *
 * Ordering:  DEBUG(0) < INFO(1) < WARNING(2) < RAW(3) < SUCCESS(4) < ERROR(5) < FATAL(6)
 */
enum class LoggerLevel : int { DEBUG = 0, INFO = 1, WARNING = 2, RAW = 3, SUCCESS = 4, ERROR = 5, FATAL = 6 };

// ── LoggerConfig ─────────────────────────────────────────────────────────────

/**
 * @brief Construction-time configuration for Logger, all options are optional.
 *
 * Example:
 * @code
 *   Logger lg(LoggerConfig{}.with_file("run.log").with_async());
 * @endcode
 */
struct LoggerConfig {
    bool to_stdout = true;                      ///< Write to stdout/stderr.
    bool use_colors = true;                     ///< Emit ANSI escape codes on console.
    bool show_thread = true;                    ///< Prefix each line with a short thread ID.
    bool async = false;                         ///< Dispatch writes to a background thread.
    std::string file_path;                      ///< Non-empty → open this file for plain-text output.
    LoggerLevel min_level = LoggerLevel::INFO;  ///< Discard messages below this level.
    bool show_memory = false;                   ///< Prefix each line with current RSS in KB.

    auto with_stdout(bool enabled = true) -> auto& {
        to_stdout = enabled;
        return *this;
    }
    auto with_file(std::string path) -> auto& {
        file_path = std::move(path);
        return *this;
    }
    auto with_colors(bool enabled = true) -> auto& {
        use_colors = enabled;
        return *this;
    }
    auto with_thread(bool enabled = true) -> auto& {
        show_thread = enabled;
        return *this;
    }
    auto with_async(bool enabled = true) -> auto& {
        async = enabled;
        return *this;
    }
    auto with_min_level(LoggerLevel lvl) -> auto& {
        min_level = lvl;
        return *this;
    }
    auto with_memory(bool enabled = true) -> auto& {
        show_memory = enabled;
        return *this;
    }
};

// ── Logger ───────────────────────────────────────────────────────────────────

/**
 * @brief Thread-safe Logger.
 *
 * Supports:
 *  - Zero-overhead construction — no separate initialize() call required.
 *  - Synchronous (default) and asynchronous (background-thread) modes.
 *  - Independent stdout and file output — both may be active at once.
 *  - ANSI color codes on console, plain text in files.
 *  - Optional thread-ID stamping.
 *  - Stream-style `log_stream` objects (RAII flush on destruction).
 *  - Runtime reconfiguration of all output options.
 */
class Logger {
    // ── Private stream helpers ────────────────────────────────────────────────
    // Defined before log_stream so it can reference them.

    /// Custom streambuf that appends directly into a `std::string&`.
    /// Declared `final` to enable devirtualisation where the concrete type is visible.
    class tl_log_buf final : public std::streambuf {
        std::string* str_;

       public:
        explicit tl_log_buf(std::string& str) : str_(&str) {}

        ~tl_log_buf() override = default;
        tl_log_buf(const tl_log_buf&) = delete;
        tl_log_buf(tl_log_buf&&) = delete;
        auto operator=(const tl_log_buf&) -> tl_log_buf& = delete;
        auto operator=(tl_log_buf&&) -> tl_log_buf& = delete;

       protected:
        auto xsputn(const char* data, std::streamsize n) -> std::streamsize override {
            str_->append(data, static_cast<std::size_t>(n));
            return n;
        }
        auto overflow(int chr) -> int override {
            if (chr != traits_type::eof()) {
                str_->push_back(static_cast<char>(chr));
            }
            return chr;
        }
    };

    /// Thread-local state reused by every `log_stream` on a given thread.
    /// Declaration order (buf → sbuf → stream) guarantees correct initialisation.
    struct tl_stream_state {
        std::string buf;
        tl_log_buf sbuf{buf};
        std::ostream stream{&sbuf};
    };

    static auto get_tl_stream() -> tl_stream_state& {
        thread_local tl_stream_state state;
        return state;
    }

   public:
    /// Alias so `Logger::level::DEBUG` etc. work at call sites.
    using level = LoggerLevel;
    /// Alias so `Logger::config` still works at call sites.
    using config = LoggerConfig;

    // ── log_stream ────────────────────────────────────────────────────────────

    /**
     * @brief RAII stream wrapper — accumulates tokens via `operator<<` and
     *        flushes the full message to the Logger on destruction.
     *
     * Internally backed by a thread-local buffer, so construction is allocation-free
     * and the completed message is moved (not copied) into the log record on flush.
     *
     * @code
     *   lg.info() << "Value = " << x;   // temporary log_stream, flushed at `;`
     * @endcode
     */
    class log_stream {
       public:
        log_stream(Logger* logger_ptr, level lvl, bool exit_on_error = false)
            : lg_(logger_ptr), level_(lvl), exit_on_error_(exit_on_error), active_(lvl >= logger_ptr->min_level_.load(std::memory_order_relaxed)) {
            if (active_) {
                state_ = &Logger::get_tl_stream();
                state_->buf.clear();
            }
        }

        log_stream(log_stream&& other) noexcept
            : lg_(other.lg_), level_(other.level_), state_(other.state_), exit_on_error_(other.exit_on_error_), active_(other.active_) {
            other.moved_ = true;
        }

        log_stream(const log_stream&) = delete;
        auto operator=(const log_stream&) = delete;
        auto operator=(log_stream&&) = delete;

        template <typename T>
        auto operator<<(const T& val) -> log_stream& {
            if (active_) {
                state_->stream << val;
            }
            return *this;
        }

        /// @warning If used with FATAL level, calls `_Exit()` after flushing this logger's streams.
        /// Any other open streams or RAII resources in the process will NOT be cleaned up — destructors are not run.  Use only for truly
        /// unrecoverable errors.
        ~log_stream() {
            if (moved_ || !active_) {
                return;
            }
            if (!state_->buf.empty()) {
                lg_->emit_owned(std::move(state_->buf), level_);
            }
            if (exit_on_error_ && level_ == level::FATAL) {
                lg_->flush();
                abort();
            }
        }

       private:
        Logger* lg_;
        level level_;
        tl_stream_state* state_ = nullptr;
        bool exit_on_error_ = false;
        bool moved_ = false;
        bool active_;
    };

    // ── Construction ──────────────────────────────────────────────────────────

    /**
     * @brief Construct and immediately ready the logger.
     *
     * @param cfg    Output configuration (all fields have sensible defaults).
     * @param start  Origin for elapsed-time stamps.  Pass a previously
     *               captured `steady_clock::now()` to track time from before
     *               this object was constructed.
     *
     * @throws std::runtime_error if a file path is given but cannot be opened.
     */
    explicit Logger(config cfg = {}, std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now());

    Logger(const Logger&) = delete;
    Logger(Logger&&) = delete;
    auto operator=(const Logger&) -> Logger& = delete;
    auto operator=(Logger&&) -> Logger& = delete;

    // ── Runtime controls ──────────────────────────────────────────────────────

    /// Reconfigure all output options at runtime.
    /// Note: async mode is constructor-only and is not changed by this method.
    void set_config(config cfg) {
        set_stdout(cfg.to_stdout);
        set_colors(cfg.use_colors);
        set_thread(cfg.show_thread);
        set_memory(cfg.show_memory);
        set_min_level(cfg.min_level);
        if (!cfg.file_path.empty()) {
            open_file(cfg.file_path);
        }
    }

    /// Enable or disable console (stdout/stderr) output.
    void set_stdout(bool enabled) {
        std::lock_guard lock(mutex_);
        to_stdout_ = enabled;
    }
    /// Enable or disable ANSI color codes on the console.
    void set_colors(bool enabled) {
        std::lock_guard lock(mutex_);
        use_colors_ = enabled;
    }
    /// Enable or disable thread-ID prefixes.
    void set_thread(bool enabled) {
        std::lock_guard lock(mutex_);
        show_thread_ = enabled;
    }
    /// Change the minimum level filter at runtime (lock-free).
    void set_min_level(level lvl) { min_level_.store(lvl, std::memory_order_relaxed); }
    /// Enable or disable memory-usage stamping (lock-free).
    void set_memory(bool enabled) { show_memory_.store(enabled, std::memory_order_relaxed); }

    /**
     * @brief Open (or switch to) a log file.  Can be called at any time.
     *        If a file was already open it is closed first.
     * @throws std::runtime_error if the file cannot be opened.
     */
    void open_file(const std::string& path) {
        std::lock_guard lock(mutex_);
        if (file_.is_open()) {
            file_.close();
        }
        file_.open(path, std::ios::app);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open log file: " + path);
        }
    }

    /// Close the current log file (stdout/stderr output is unaffected).
    void close_file() {
        std::lock_guard lock(mutex_);
        if (file_.is_open()) {
            file_.close();
        }
    }

    // ── Flush ─────────────────────────────────────────────────────────────────

    /// Flush all output streams.  In async mode, blocks until the background
    /// queue is fully drained so all enqueued records are guaranteed written.
    void flush() {
        if (async_mode_) {
            std::unique_lock lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return queue_.empty() && in_flight_.load(std::memory_order_acquire) == 0; });
        }
        std::lock_guard lock(mutex_);
        std::cout.flush();
        std::cerr.flush();
        if (file_.is_open()) {
            file_.flush();
        }
    }

    // ── String overloads ──────────────────────────────────────────────────────

    void log(std::string_view msg) { emit(msg, level::RAW); }
    void info(std::string_view msg) { emit(msg, level::INFO); }
    void success(std::string_view msg) { emit(msg, level::SUCCESS); }
    void warning(std::string_view msg) { emit(msg, level::WARNING); }
    void debug(std::string_view msg) { emit(msg, level::DEBUG); }
    void error(std::string_view msg) { emit(msg, level::ERROR); }
    /// @warning Calls `_Exit()` after flushing this logger's streams.
    /// Any other open streams or RAII resources in the process will NOT be cleaned up — destructors are not run.  Use only for truly unrecoverable
    /// errors.
    [[noreturn]] void fatal(std::string_view msg) {
        emit(msg, level::FATAL);
        flush();
        abort();
    }

    // ── Stream-style factory methods ──────────────────────────────────────────

    [[nodiscard]] auto log() -> log_stream { return {this, level::RAW}; }
    [[nodiscard]] auto info() -> log_stream { return {this, level::INFO}; }
    [[nodiscard]] auto success() -> log_stream { return {this, level::SUCCESS}; }
    [[nodiscard]] auto warning() -> log_stream { return {this, level::WARNING}; }
    [[nodiscard]] auto debug() -> log_stream { return {this, level::DEBUG}; }
    [[nodiscard]] auto error() -> log_stream { return {this, level::ERROR}; }
    /// @warning Calls `_Exit()` after flushing this logger's streams.
    /// Any other open streams or RAII resources in the process will NOT be cleaned up — destructors are not run.  Use only for truly
    /// unrecoverable errors.
    [[nodiscard]] auto fatal() -> log_stream { return {this, level::FATAL, true}; }

    /// Select a level via subscript: `lg[INFO] << "msg"`.
    [[nodiscard]] auto operator[](level lvl) -> log_stream { return {this, lvl, lvl == level::FATAL}; }

    /// Stream directly at RAW level: `lg << "msg"`.
    template <typename T>
    auto operator<<(T&& val) -> log_stream {
        log_stream stream{this, level::RAW};
        stream << std::forward<T>(val);
        return stream;
    }

    // ── Destructor ────────────────────────────────────────────────────────────

    ~Logger() {
        flush();
        if (async_mode_) {
            stop_worker();
        }
        std::lock_guard lock(mutex_);
        if (file_.is_open()) {
            file_.close();
        }
    }

    friend class log_stream;

   private:
    // ── Internal record type ──────────────────────────────────────────────────

    struct record {
        std::string message;
        level lvl;
        double elapsed;         // captured at emit() time
        std::string thread_id;  // captured at emit() time
        long memory_kb;         // captured at emit() time; -1 when not requested
    };

    // ── Level metadata ────────────────────────────────────────────────────────

    struct level_meta {
        const char* label;  // fixed-width, 7 chars
        const char* color;
        bool use_err;  // route to stderr?
    };

    static auto meta_of(level lvl) noexcept -> level_meta {
        switch (lvl) {
            case level::RAW:
                return {.label = "       ", .color = ansi::codes::white, .use_err = false};
            case level::DEBUG:
                return {.label = " DEBUG ", .color = ansi::codes::bright_magenta, .use_err = false};
            case level::INFO:
                return {.label = "  INFO ", .color = ansi::codes::bright_blue, .use_err = false};
            case level::SUCCESS:
                return {.label = "SUCCESS", .color = ansi::codes::bright_green, .use_err = false};
            case level::WARNING:
                return {.label = "WARNING", .color = ansi::codes::bright_yellow, .use_err = true};
            case level::ERROR:
                return {.label = " ERROR ", .color = ansi::codes::bright_red, .use_err = true};
            case level::FATAL:
                return {.label = " FATAL ", .color = ansi::codes::bright_red, .use_err = true};
        }
        return {.label = "       ", .color = ansi::codes::white, .use_err = false};
    }

    // ── Memory sampling ───────────────────────────────────────────────────────

    static auto current_memory_kb() -> long {
#ifdef __APPLE__
        task_vm_info_data_t info{};
        mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
        if (task_info(mach_task_self(), TASK_VM_INFO, reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS) {
            return static_cast<long>(info.phys_footprint / 1024);
        }
        return -1;
#else
        // /proc/self/statm: "total resident ..." — values in pages
        std::ifstream f("/proc/self/statm");
        long total = 0, resident = 0;
        f >> total >> resident;
        if (!f) return -1;
        return resident * (sysconf(_SC_PAGESIZE) / 1024);
#endif
    }

    // ── Memory formatting ─────────────────────────────────────────────────────

    static auto format_memory(long kb) -> std::string {
        // Auto-scale unit so the 7-digit field never overflows:
        //   < 1,000,000 KB  (~1 TB) → show KB
        //   < 1,000,000 MB  (~1 PB) → show MB
        //   else             (≥ 1 PB) → show GB
        // "[NNNNNNN XB] " — always 13 chars regardless of scale.
        constexpr long KB_PER_MB = 1'000L;
        constexpr long KB_PER_GB = 1'000'000L;
        constexpr long KB_PER_TB = 1'000'000'000L;

        long value = kb;
        char unit0 = 'K';
        char unit1 = 'B';
        if (kb >= KB_PER_TB) {
            value = kb / KB_PER_GB;
            unit0 = 'G';
        } else if (kb >= KB_PER_GB) {
            value = kb / KB_PER_MB;
            unit0 = 'M';
        }

        char num[8] = {' ', ' ', ' ', ' ', ' ', ' ', '0', '\0'};
        if (value > 0) {
            long v = value;
            for (int i = 6; i >= 0 && v > 0; --i, v /= 10) {
                num[i] = static_cast<char>('0' + (v % 10));
            }
        }
        char buf[14];
        buf[0] = '[';
        buf[1] = num[0];
        buf[2] = num[1];
        buf[3] = num[2];
        buf[4] = num[3];
        buf[5] = num[4];
        buf[6] = num[5];
        buf[7] = num[6];
        buf[8] = ' ';
        buf[9] = unit0;
        buf[10] = unit1;
        buf[11] = ']';
        buf[12] = ' ';
        buf[13] = '\0';
        return {buf, 13};
    }

    // ── Time formatting ───────────────────────────────────────────────────────

    static auto format_time(double elapsed) -> std::string {
        constexpr int MS_PER_SECOND = 1000;
        constexpr int MS_PER_MINUTE = 60000;
        constexpr int MS_PER_HOUR = 3600000;

        int total_ms = static_cast<int>(elapsed * MS_PER_SECOND);
        int hours = total_ms / MS_PER_HOUR;
        int minutes = (total_ms % MS_PER_HOUR) / MS_PER_MINUTE;
        int seconds = (total_ms % MS_PER_MINUTE) / MS_PER_SECOND;
        int millis = total_ms % MS_PER_SECOND;

        // "HH:MM:SS.mmm" = 12 chars + null
        char buf[13];
        buf[0] = '0' + (hours / 10);
        buf[1] = '0' + (hours % 10);
        buf[2] = ':';
        buf[3] = '0' + (minutes / 10);
        buf[4] = '0' + (minutes % 10);
        buf[5] = ':';
        buf[6] = '0' + (seconds / 10);
        buf[7] = '0' + (seconds % 10);
        buf[8] = '.';
        buf[9] = '0' + (millis / 100);
        buf[10] = '0' + (millis / 10 % 10);
        buf[11] = '0' + (millis % 10);
        buf[12] = '\0';
        return {buf, 12};  // construct from ptr+len, no strlen scan
    }

    // ── Thread ID ─────────────────────────────────────────────────────────────

    static auto current_thread_tag() -> const std::string& {
        thread_local std::string tag = "[T:" + []() -> std::string {
            std::ostringstream oss;
            oss << std::this_thread::get_id();
            std::string str = oss.str();
            if (str.size() > 4) {
                str = str.substr(str.size() - 4);
            }
            return str;
        }() + "] ";
        return tag;
    }

    // ── write_record (called with mutex_ held) ────────────────────────────────

    void write_record(const record& rec) {
        const auto [label, color, use_err] = meta_of(rec.lvl);
        const bool has_prefix = rec.lvl != level::RAW;
        const bool need_plain = file_.is_open() || !to_stdout_ || !use_colors_ || !has_prefix;
        const bool need_colored = to_stdout_ && use_colors_ && has_prefix;

        // Compute once, shared between all lines of a multi-line message.
        const std::string time_str = has_prefix ? format_time(rec.elapsed) : std::string{};
        const std::string mem_str = (has_prefix && rec.memory_kb >= 0) ? format_memory(rec.memory_kb) : std::string{};

        thread_local std::string plain;
        thread_local std::string colored;

        // Emits one line (no embedded newlines) with the full prefix.
        const auto write_line = [&](std::string_view line) {
            // Plain line (for file and/or uncolored console).
            if (need_plain) {
                plain.clear();
                if (has_prefix) {
                    plain += '[';
                    plain += time_str;
                    plain += "] ";
                    plain += mem_str;
                    if (show_thread_) {
                        plain += rec.thread_id;
                    }
                    plain += '[';
                    plain += label;
                    plain += "] ";
                }
                plain += line;
            }

            // Colored line — built into a second thread_local buffer, written in
            // one ostr.write() call to avoid per-segment virtual-dispatch overhead.
            if (need_colored) {
                colored.clear();
                colored += ansi::codes::bright_cyan;
                colored += '[';
                colored += time_str;
                colored += "] ";
                colored += ansi::codes::reset;
                if (!mem_str.empty()) {
                    colored += ansi::codes::green;
                    colored += mem_str;
                    colored += ansi::codes::reset;
                }
                if (show_thread_) {
                    colored += ansi::codes::magenta;
                    colored += rec.thread_id;
                    colored += ansi::codes::reset;
                }
                colored += color;
                colored += '[';
                colored += label;
                colored += "] ";
                colored += ansi::codes::reset;
                colored += line;
                colored += '\n';
            }

            if (file_.is_open()) {
                file_ << plain << '\n';
                file_.flush();
            }

            if (to_stdout_) {
                std::ostream& ostr = use_err ? std::cerr : std::cout;
                if (need_colored) {
                    ostr.write(colored.data(), static_cast<std::streamsize>(colored.size()));
                } else {
                    ostr << plain << '\n';
                }
            }
        };

        // Fast path: single-line message — one scan, no branching overhead.
        std::string_view rest = rec.message;
        if (rest.find('\n') == std::string_view::npos) {
            write_line(rest);
            return;
        }

        // Multi-line path: each segment gets its own full prefix.
        while (true) {
            const auto pos = rest.find('\n');
            if (pos == std::string_view::npos) {
                if (!rest.empty()) {
                    write_line(rest);
                }  // skip empty trailing segment from "msg\n"
                break;
            }
            write_line(rest.substr(0, pos));
            rest = rest.substr(pos + 1);
        }
    }

    // ── emit ─────────────────────────────────────────────────────────────────

    void emit(std::string_view message, level lvl) {
        if (lvl < min_level_.load(std::memory_order_relaxed)) {
            return;
        }
        double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
        long mem_kb = show_memory_ ? current_memory_kb() : -1L;
        record rec{.message = std::string(message), .lvl = lvl, .elapsed = elapsed_s, .thread_id = current_thread_tag(), .memory_kb = mem_kb};

        if (async_mode_) {
            {
                std::lock_guard lock(queue_mutex_);
                queue_.push(std::move(rec));
            }
            queue_cv_.notify_one();
        } else {
            std::lock_guard lock(mutex_);
            write_record(rec);
        }
    }

    // ── emit_owned ────────────────────────────────────────────────────────────

    void emit_owned(std::string msg, level lvl) {
        if (lvl < min_level_.load(std::memory_order_relaxed)) {
            return;
        }
        double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
        long mem_kb = show_memory_ ? current_memory_kb() : -1L;
        record rec{.message = std::move(msg), .lvl = lvl, .elapsed = elapsed_s, .thread_id = current_thread_tag(), .memory_kb = mem_kb};

        if (async_mode_) {
            {
                std::lock_guard lock(queue_mutex_);
                queue_.push(std::move(rec));
            }
            queue_cv_.notify_one();
        } else {
            std::lock_guard lock(mutex_);
            write_record(rec);
        }
    }

    // ── Async worker ──────────────────────────────────────────────────────────

    void start_worker() {
        worker_running_ = true;
        worker_ = std::thread([this] {
            while (true) {
                std::unique_lock lock(queue_mutex_);
                queue_cv_.wait(lock, [this] { return !queue_.empty() || !worker_running_; });

                while (!queue_.empty()) {
                    record rec = std::move(queue_.front());
                    queue_.pop();
                    in_flight_.fetch_add(1, std::memory_order_relaxed);
                    lock.unlock();
                    {
                        std::lock_guard writelock(mutex_);
                        write_record(rec);
                    }
                    in_flight_.fetch_sub(1, std::memory_order_release);
                    queue_cv_.notify_all();
                    lock.lock();
                }

                // Signal flush() waiters that the queue is now empty.
                queue_cv_.notify_all();

                if (!worker_running_ && queue_.empty()) {
                    break;
                }
            }
        });
    }

    void stop_worker() {
        {
            std::lock_guard lock(queue_mutex_);
            worker_running_ = false;
        }
        queue_cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    // ── Data members ─────────────────────────────────────────────────────────

    mutable std::mutex mutex_;
    bool to_stdout_ = true;
    bool use_colors_ = true;
    bool show_thread_ = true;
    std::atomic<bool> show_memory_{false};
    bool async_mode_ = false;
    std::atomic<int> in_flight_{0};
    std::atomic<level> min_level_{level::INFO};

    std::ofstream file_;

    std::thread worker_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<record> queue_;
    std::atomic<bool> worker_running_{false};

    std::chrono::steady_clock::time_point start_;
};

// ── Constructor (defined outside class so config's default member initializers
//    are fully visible at the point where cfg = {} is evaluated) ───────────────

inline Logger::Logger(config cfg, std::chrono::steady_clock::time_point start)
    : to_stdout_(cfg.to_stdout),
      use_colors_(cfg.use_colors),
      show_thread_(cfg.show_thread),
      show_memory_(cfg.show_memory),
      async_mode_(cfg.async),
      min_level_(cfg.min_level),
      start_(start) {
    if (!cfg.file_path.empty()) {
        file_.open(cfg.file_path, std::ios::app);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open log file: " + cfg.file_path);
        }
    }
    if (async_mode_) {
        start_worker();
    }
}

}  // namespace utilz
