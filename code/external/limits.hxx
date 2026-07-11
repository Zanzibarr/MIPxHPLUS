#pragma once

/**
 * @file limits.hxx
 * @brief Cooperative time limits and memory limits.
 * @version 3.0.1
 *
 * @details
 * Two self-contained limiter classes — `TimeLimiter` and `MemoryLimiter` —
 * each with an independent background polling thread.
 *
 * **Global limits** are a special case: convenience free functions
 * (`set_time_limit`, `set_memory_limit`, …) operate on process-wide singleton
 * instances whose callbacks write `global_limits::time_flag` /
 * `global_limits::memory_flag`.  Poll those flags cheaply with
 * `global_limits::time_reached()` / `global_limits::memory_reached()` from anywhere.
 *
 * **Local limits** are plain stack objects.  Give them a callback if you want
 * a notification, or just poll `expired()` / `exceeded()`.
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * Repository: https://github.com/Zanzibarr/cpp_utils
 *
 * SPDX-License-Identifier: MIT
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>

#ifdef __APPLE__
#include <mach/mach.h>
#else
#include <unistd.h>

#include <fstream>

#endif

namespace utilz {

// ── TimeLimiter ───────────────────────────────────────────────────────────────

/**
 * Cooperative time limiter backed by a single background thread.
 *
 * `expired()` is sticky: once true it never reverts, even if `cancel()` is
 * called afterwards.
 *
 * Usage — local:
 * @code
 *   TimeLimiter lim;
 *   lim.set(3s, []{ std::cerr << "time's up\n"; });
 *   while (!lim.expired()) { ... }
 * @endcode
 *
 * Usage — global (via free functions):
 * @code
 *   set_time_limit(5);
 *   while (!LIMITS_CHECK_STOP()) { ... }
 *   cancel_time_limit();
 * @endcode
 */
class [[nodiscard]] TimeLimiter {
   public:
    using Clock = std::chrono::steady_clock;
    using Callback = std::function<void()>;

    TimeLimiter() = default;
    ~TimeLimiter() { cancel(); }

    TimeLimiter(const TimeLimiter&) = delete;
    auto operator=(const TimeLimiter&) -> TimeLimiter& = delete;
    TimeLimiter(TimeLimiter&&) = delete;
    auto operator=(TimeLimiter&&) -> TimeLimiter& = delete;

    /**
     * Arms the timer.  Cancels any previously running timer first.
     *
     * @param duration   How long until expiry.
     * @param on_expire  Called from the background thread on expiry (optional).
     */
    void set(std::chrono::seconds duration, Callback on_expire = nullptr) {
        cancel();
        on_expire_ = std::move(on_expire);

        thread_ = std::jthread{[this, deadline = Clock::now() + duration](std::stop_token stop) {
            std::unique_lock lock{cv_mutex_};
            cv_.wait_until(lock, stop, deadline, [] { return false; });

            if (stop.stop_requested()) {
                return;
            }

            expired_.store(true, std::memory_order_release);
            if (on_expire_) {
                on_expire_();
            }
        }};
    }

    /**
     * Disarms the timer.  Blocks until the background thread exits.
     * Does NOT reset `expired_` — if the timer fired naturally that state
     * is preserved and `expired()` will keep returning true.
     */
    void cancel() {
        if (thread_.joinable()) {
            thread_.request_stop();
            thread_.join();
        }
    }

    /**
     * Returns true if this limiter has expired.  Sticky: never reverts to
     * false after returning true.
     */
    [[nodiscard]] auto expired() const noexcept -> bool { return expired_.load(std::memory_order_acquire); }

   private:
    std::atomic<bool> expired_{false};
    std::jthread thread_;
    Callback on_expire_;
    std::condition_variable_any cv_;
    std::mutex cv_mutex_;
};

// ── Memory helpers ────────────────────────────────────────────────────────────

namespace memlim {

constexpr std::size_t BYTES_PER_MB = 1024ULL * 1024ULL;

/**
 * Returns the current RSS / physical footprint in bytes, or 0 on failure.
 * Callers must treat 0 as "unknown" and skip threshold comparisons.
 */
[[nodiscard]] inline auto current_memory_bytes() noexcept -> std::size_t {
#ifdef __APPLE__
    task_vm_info_data_t info{};
    mach_msg_type_number_t count = TASK_VM_INFO_COUNT;
    if (task_info(mach_task_self(), TASK_VM_INFO, reinterpret_cast<task_info_t>(&info), &count) == KERN_SUCCESS) {
        return static_cast<std::size_t>(info.phys_footprint);
    }
    return 0;
#else
    std::ifstream f{"/proc/self/statm"};
    long resident{};
    if (long total{}; f >> total >> resident) {
        return static_cast<std::size_t>(resident) * static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
    }
    return 0;
#endif
}

/**
 * Returns the current RSS in bytes, or -1 on failure.
 */
[[nodiscard]] inline auto current_memory_usage() noexcept -> std::ptrdiff_t {
    const auto bytes = current_memory_bytes();
    return bytes > 0 ? static_cast<std::ptrdiff_t>(bytes) : -1;
}

}  // namespace memlim

// ── MemoryLimiter ─────────────────────────────────────────────────────────────

/**
 * Cooperative RSS memory limiter backed by a single background thread.
 *
 * `exceeded()` is sticky: once true it never reverts.
 *
 * **Note on multi-threaded use:** the underlying measurement is process-wide
 * RSS.  There is no per-thread memory accounting at the OS level, so multiple
 * concurrent `MemoryLimiter` instances all observe the same number.
 *
 * Usage — local:
 * @code
 *   MemoryLimiter mlim;
 *   mlim.set(256, []{ std::cerr << "memory exceeded\n"; });
 *   while (!mlim.exceeded()) { ... }
 * @endcode
 *
 * Usage — global (via free functions):
 * @code
 *   set_memory_limit(512);
 *   while (!LIMITS_CHECK_MEMORY()) { ... }
 *   cancel_memory_limit();
 * @endcode
 */
class [[nodiscard]] MemoryLimiter {
   public:
    using Callback = std::function<void()>;
    static constexpr auto POLL_INTERVAL = std::chrono::milliseconds{100};

    MemoryLimiter() = default;
    ~MemoryLimiter() { cancel(); }

    MemoryLimiter(const MemoryLimiter&) = delete;
    auto operator=(const MemoryLimiter&) -> MemoryLimiter& = delete;
    MemoryLimiter(MemoryLimiter&&) = delete;
    auto operator=(MemoryLimiter&&) -> MemoryLimiter& = delete;

    /**
     * Arms the monitor.  Cancels any previously running monitor first.
     *
     * @param limit_mb   Threshold in megabytes.  Must be > 0.
     * @param on_exceed  Called from the polling thread on first breach (optional).
     */
    void set(std::size_t limit_mb, Callback on_exceed = nullptr) {
        cancel();
        on_exceed_ = std::move(on_exceed);
        const std::size_t limit_bytes = limit_mb * memlim::BYTES_PER_MB;

        thread_ = std::jthread{[this, limit_bytes](std::stop_token stop) {
            while (!stop.stop_requested()) {
                {
                    std::unique_lock lock{cv_mutex_};
                    cv_.wait_for(lock, stop, POLL_INTERVAL, [] { return false; });
                }

                if (stop.stop_requested()) {
                    return;
                }

                const auto bytes = memlim::current_memory_bytes();
                if (bytes == 0) {
                    continue;  // sampling failure — skip
                }

                if (bytes >= limit_bytes) {
                    exceeded_.store(true, std::memory_order_release);
                    if (on_exceed_) {
                        on_exceed_();
                    }
                    return;
                }
            }
        }};
    }

    /**
     * Disarms the monitor.  Blocks until the polling thread exits.
     * Does NOT reset `exceeded_` — naturally-fired state is preserved.
     */
    void cancel() {
        if (thread_.joinable()) {
            thread_.request_stop();
            thread_.join();
        }
    }

    /**
     * Returns true if the RSS threshold has been breached.  Sticky: never
     * reverts to false after returning true.
     */
    [[nodiscard]] auto exceeded() const noexcept -> bool { return exceeded_.load(std::memory_order_acquire); }

   private:
    std::atomic<bool> exceeded_{false};
    std::jthread thread_;
    Callback on_exceed_;
    std::condition_variable_any cv_;
    std::mutex cv_mutex_;
};

// ── Global singletons and free functions ──────────────────────────────────────

namespace global_limits {

/// Global flag atomics
inline std::atomic<bool> time_flag{false};
inline std::atomic<bool> memory_flag{false};

/// The global TimeLimiter: its callback writes global_limits::time_flag.
inline TimeLimiter global_time_limiter;

/// The global MemoryLimiter: its callback writes global_limits::memory_flag.
inline MemoryLimiter global_memory_limiter;

/// Cheap relaxed read of the global time flag.  Use inside tight loops.
inline auto time_reached() { return global_limits::time_flag.load(std::memory_order_relaxed); }
/// Cheap relaxed read of the global memory flag.  Use inside tight loops.
inline auto memory_reached() { return global_limits::memory_flag.load(std::memory_order_relaxed); }

}  // namespace global_limits

/**
 * Arms the process-wide time limit.
 * On expiry, sets `global_limits::time_flag` (readable via `global_limits::time_reached()`).
 *
 * @param seconds    Limit duration.
 * @param on_expire  Extra callback on expiry alongside the flag write (optional).
 */
inline void set_time_limit(unsigned int seconds, TimeLimiter::Callback on_expire = nullptr) {
    global_limits::global_time_limiter.set(std::chrono::seconds{seconds}, [cb = std::move(on_expire)] {
        global_limits::time_flag.store(true, std::memory_order_release);
        if (cb) {
            cb();
        }
    });
}

/**
 * Disarms the process-wide time limit.
 * Does NOT reset `global_limits::time_flag` if it already fired.
 */
inline void cancel_time_limit() { global_limits::global_time_limiter.cancel(); }

/**
 * Arms the process-wide memory limit.
 * On breach, sets `global_limits::memory_flag` (readable via `global_limits::memory_reached()`).
 *
 * @param limit_mb   Threshold in megabytes.
 * @param on_exceed  Extra callback on breach alongside the flag write (optional).
 */
inline void set_memory_limit(std::size_t limit_mb, MemoryLimiter::Callback on_exceed = nullptr) {
    global_limits::global_memory_limiter.set(limit_mb, [cb = std::move(on_exceed)] {
        global_limits::memory_flag.store(true, std::memory_order_release);
        if (cb) {
            cb();
        }
    });
}

/**
 * Disarms the process-wide memory limit.
 * Does NOT reset `global_limits::memory_flag` if it already fired.
 */
inline void cancel_memory_limit() { global_limits::global_memory_limiter.cancel(); }

}  // namespace utilz
