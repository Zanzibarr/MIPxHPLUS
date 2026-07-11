#pragma once

/**
 * @file timer.hxx
 * @brief High-resolution timer with per-name statistics accumulation.
 * @version 2.1.0
 *
 * @details
 * Core types:
 *   - `timer_detail::WelfordAccumulator` — shared base implementing Welford's
 *     online algorithm (count, total, min, max, running mean/M2, merge).
 *     Also reused by `stats_registry.hxx` as `GaugeStats`.
 *   - `TimerStats` — inherits `WelfordAccumulator` and adds `get_<stat><D>()`
 *     helpers that convert the stored nanosecond values to any `std::chrono`
 *     duration type.
 *   - `Timer` — start/stop timer.
 *   - `TimerRegistry` — process-wide registry keyed by compile-time names
 *     (`CTString` → FNV-1a hash → `CtSlotID`).  Uses thread-local slot
 *     arrays for contention-free recording; slots are merged into a shared
 *     table on `get_report()`.
 *   - `ScopedTimer` / `make_scoped_timer()` — RAII wrappers that start a
 *     named registry slot and stop it on destruction.
 *
 * The global registry is accessible via `TIMER_REG` (macro alias for
 * `global_timer_registry()`).  Per-thread data is lazily initialized and
 * auto-merged, so recording from multiple threads requires no locking on the
 * hot path.
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * Repository: https://github.com/Zanzibarr/cpp_utils
 *
 * SPDX-License-Identifier: MIT
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ct_string.hxx"

namespace utilz {
// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace timer_detail {

using clock = std::chrono::steady_clock;
using time_point = clock::time_point;

// Constraint shared by all public Duration template parameters.
// Accepts any std::chrono::duration specialization (not just the four named ones).
template <typename D>
concept ValidDuration =
    std::is_same_v<D, std::chrono::duration<typename D::rep, typename D::period>> && std::ratio_less_equal<typename D::period, std::ratio<60>>::value;

// Convert a raw nanosecond double to the requested Duration.
template <ValidDuration D>
auto ns_to(double nanos) -> double {
    using Target = std::chrono::duration<double, typename D::period>;
    return std::chrono::duration_cast<Target>(std::chrono::duration<double, std::nano>(nanos)).count();
}

// Convert a steady_clock duration to nanoseconds as double.
inline auto to_ns(clock::duration dur) -> double { return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(dur).count()); }

template <ValidDuration D>
consteval auto unit_name() -> const char* {
    if constexpr (std::is_same_v<D, std::chrono::nanoseconds>) {
        return "ns";
    } else if constexpr (std::is_same_v<D, std::chrono::microseconds>) {
        return "us";
    } else if constexpr (std::is_same_v<D, std::chrono::milliseconds>) {
        return "ms";
    } else if constexpr (std::is_same_v<D, std::chrono::seconds>) {
        return "s";
    } else if constexpr (std::is_same_v<D, std::chrono::minutes>) {
        return "min";
    } else {
        return "?";
    }
}

// ── Print helpers shared by both print_stats_report variants ─────────────────

struct UnitSpec {
    double divisor;
    const char* suffix;
};

inline auto pick_unit(double min_ns) -> UnitSpec {
    constexpr double NANOS_PER_SECOND = 1e9;
    constexpr double NANOS_PER_MILLI = 1e6;
    constexpr double NANOS_PER_MICRO = 1e3;

    if (min_ns >= NANOS_PER_SECOND) {
        return {.divisor = NANOS_PER_SECOND, .suffix = "s"};
    }
    if (min_ns >= NANOS_PER_MILLI) {
        return {.divisor = NANOS_PER_MILLI, .suffix = "ms"};
    }
    if (min_ns >= NANOS_PER_MICRO) {
        return {.divisor = NANOS_PER_MICRO, .suffix = "us"};
    }
    return {.divisor = 1.0, .suffix = "ns"};
}

template <typename Row>
auto col_min(const std::vector<Row>& rows, std::function<double(const Row&)> func) -> double {
    double best = std::numeric_limits<double>::max();
    for (const auto& row : rows) {
        double val = func(row);
        if (val > 0.0) {
            best = std::min(best, val);
        }
    }
    return (best == std::numeric_limits<double>::max()) ? 1.0 : best;
}

// ── Shared Welford statistics accumulator ─────────────────────────────────────
//
// Core fields and methods for online mean/variance computation (Welford 1962).
// Used by TimerStats (nanosecond timers) and stats_detail::GaugeStats
// (arbitrary double values).  Duration-conversion accessors live only in
// TimerStats, as they assume nanosecond-valued data.

struct WelfordAccumulator {
    std::size_t count = 0;
    double total = 0.0;
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    double mean = 0.0;  // running mean
    double M2 = 0.0;    // running sum of squared deviations

    /** Records one sample, updating all running statistics. */
    void record(double val) {
        ++count;
        total += val;
        min = std::min(min, val);
        max = std::max(max, val);
        double delta = val - mean;
        mean += delta / static_cast<double>(count);
        M2 += delta * (val - mean);
    }

    /** Resets all fields to their initial values. */
    void reset() { *this = WelfordAccumulator{}; }

    /** Population variance (M2 / n). Returns 0 if fewer than 2 samples. */
    [[nodiscard]] auto variance() const -> double { return count < 2 ? 0.0 : M2 / static_cast<double>(count); }
    /** Sample variance (M2 / (n-1), Bessel's correction). Returns 0 if fewer than 2 samples. */
    [[nodiscard]] auto sample_variance() const -> double { return count < 2 ? 0.0 : M2 / static_cast<double>(count - 1); }
    /** Population standard deviation. */
    [[nodiscard]] auto stddev() const -> double { return std::sqrt(variance()); }
    /** Sample standard deviation (Bessel's correction). */
    [[nodiscard]] auto sample_stddev() const -> double { return std::sqrt(sample_variance()); }

    /**
     * Parallel Welford merge — correctly combines means and variances from two
     * independent sets without needing access to the original samples.
     * Reference: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
     */
    void merge(const WelfordAccumulator& other) {
        if (other.count == 0) {
            return;
        }
        if (count == 0) {
            *this = other;
            return;
        }
        auto this_count = static_cast<double>(count);
        auto other_count = static_cast<double>(other.count);
        double combined = this_count + other_count;
        double delta = other.mean - mean;
        mean = mean + (delta * (other_count / combined));
        M2 += other.M2 + (delta * delta * (this_count * other_count / combined));
        count += other.count;
        total += other.total;
        min = std::min(min, other.min);
        max = std::max(max, other.max);
    }
};

}  // namespace timer_detail

template <std::size_t Hash>
struct CtSlotID {
    static const std::size_t value;
};

// ─────────────────────────────────────────────────────────────────────────────
// Timer
// ─────────────────────────────────────────────────────────────────────────────

/**
 * A simple, single-threaded wall-clock timer.
 * All internal state is stored in nanoseconds as doubles.
 *
 * Thread safety: not thread-safe. Each instance must be owned by one thread.
 * Use TimerRegistry for multi-threaded access.
 */
class Timer {
   public:
    /** @param start_immediately If `true`, calls `start()` immediately. */
    explicit Timer(bool start_immediately = false) {
        if (start_immediately) {
            start();
        }
    }

    /** Starts the timer.  No-op if already running. */
    void start() {
        if (running_) {
            return;
        }
        running_ = true;
        start_tp_ = timer_detail::clock::now();
    }

    /** Stops the timer and accumulates the elapsed lap into `elapsed_`. No-op if not running. */
    void stop() {
        if (!running_) {
            return;
        }
        running_ = false;
        last_lap_ = timer_detail::to_ns(timer_detail::clock::now() - start_tp_);
        elapsed_ += last_lap_;
    }

    /** Resets all state; the timer is stopped after this call. */
    void reset() {
        running_ = false;
        elapsed_ = 0.0;
        last_lap_ = 0.0;
        start_tp_ = {};
    }

    /** Returns `true` if the timer is currently running. */
    [[nodiscard]] auto is_running() const -> bool { return running_; }

    /** Elapsed time in the requested unit. Counts live time if still running. */
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto elapsed() const -> double {
        double total = elapsed_;
        if (running_) {
            total += timer_detail::to_ns(timer_detail::clock::now() - start_tp_);
        }
        return timer_detail::ns_to<D>(total);
    }
    [[nodiscard]] auto elapsed_ns() const -> double { return elapsed<std::chrono::nanoseconds>(); }
    [[nodiscard]] auto elapsed_us() const -> double { return elapsed<std::chrono::microseconds>(); }
    [[nodiscard]] auto elapsed_ms() const -> double { return elapsed<std::chrono::milliseconds>(); }
    [[nodiscard]] auto elapsed_s() const -> double { return elapsed<std::chrono::seconds>(); }

    /** Duration of the most recent start->stop pair in nanoseconds. Zero if never stopped. */
    [[nodiscard]] auto last_lap_ns() const -> double { return last_lap_; }

   private:
    bool running_ = false;
    double elapsed_ = 0.0;
    double last_lap_ = 0.0;
    timer_detail::time_point start_tp_;
};

// ─────────────────────────────────────────────────────────────────────────────
// TimerStats
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Accumulates statistics over multiple start/stop cycles using Welford's
 * online algorithm. All internal state is in nanoseconds.
 *
 * Inherits the core Welford fields and methods (record, reset, variance,
 * stddev, merge, etc.) from timer_detail::WelfordAccumulator.  The Duration
 * conversion accessors (get_total<D>, get_mean<D>, ...) are added here as
 * they are specific to nanosecond-valued data.
 *
 * Thread safety: not thread-safe on its own. Access is serialised externally.
 */
struct TimerStats : timer_detail::WelfordAccumulator {
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_total() const -> double {
        return timer_detail::ns_to<D>(total);
    }
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_mean() const -> double {
        return count == 0 ? 0.0 : timer_detail::ns_to<D>(mean);
    }
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_min() const -> double {
        return count == 0 ? 0.0 : timer_detail::ns_to<D>(min);
    }
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_max() const -> double {
        return count == 0 ? 0.0 : timer_detail::ns_to<D>(max);
    }
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_stddev() const -> double {
        return timer_detail::ns_to<D>(stddev());
    }
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_sample_stddev() const -> double {
        return timer_detail::ns_to<D>(sample_stddev());
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// TimerRegistry
// ─────────────────────────────────────────────────────────────────────────────

/**
 * A registry for managing multiple named timers across threads.
 *
 * All timer names are compile-time CTString template parameters.
 * Lookup is O(1) array indexing — no string hashing or map search at runtime.
 *
 * Performance design
 * ──────────────────
 * The hot path (start<n> / stop / elapsed<n> / is_running<n>) is entirely
 * lock-free after the first call per name per thread. The registry mutex is
 * only acquired when:
 *   - A thread uses a name for the very first time (registers the ct slot).
 *   - reset<n>() is called.
 *   - A report is requested.
 *   - A thread exits (snapshots its stats into the graveyard).
 *
 * Per-thread storage uses a thread_local array indexed by a per-registry
 * integer ID (assigned once at construction via an atomic counter). This
 * replaces the previous unordered_map lookup on every hot-path call with a
 * single array index — effectively one pointer load after the first access.
 * Up to MAX_REGISTRIES independent TimerRegistry instances are supported
 * (default 8). Increase the constant if you need more.
 *
 * Preferred usage
 * ───────────────
 *   One-liner RAII — name resolved at compile time, stop is a pointer deref:
 *   auto t = make_scoped_timer<"db_query">(reg);
 *
 *   Manual handle-based:
 *   auto* slot = reg.start<"db_query">();
 *   ... work ...
 *   reg.stop(slot);
 */
class TimerRegistry {
   public:
    using thread_id = std::thread::id;

    // ── Per-thread slot — public so make_scoped_timer can cache the pointer ──
    struct Slot {
        Timer timer;
        TimerStats stats;
    };

    // ── Compile-time timer limit ──────────────────────────────────────────────
    // Maximum number of distinct compile-time timer names across the whole
    // program. Raise if you hit the abort() in assign_id().
    // Keep in sync with MAX_CT_STATS (stats_registry/stats_registry.hxx)
    // and MAX_CT_PARAMS (parameters/parameters.hxx) — all default to 128.
    static constexpr std::size_t MAX_CT_TIMERS = 128;

    // Maximum number of independent TimerRegistry instances in the program.
    // Each instance consumes one slot in the thread_local array used by
    // thread_local_storage(). Raise if you need more than 256 registries.
    // 256 is a really high number for this class.. it's set to 256 only so
    // that tests compile (since there a lot of registries are created)
    static constexpr std::size_t MAX_REGISTRIES = 256;

    /**
     * Assigns a unique sequential slot index to a hash at static-init time.
     * Called once per unique CTString instantiation via CtSlotID<H>::value.
     * Thread-safe: uses a static atomic counter.
     */
    static auto assign_id(std::size_t /*hash*/) -> std::size_t {
        static std::atomic<std::size_t> next{0};
        std::size_t slot_id = next.fetch_add(1, std::memory_order_relaxed);
        if (slot_id >= MAX_CT_TIMERS) {
            std::cerr << "TimerRegistry: MAX_CT_TIMERS (" << MAX_CT_TIMERS << ") exceeded. Increase the limit.\n";
            std::abort();
        }
        return slot_id;
    }

    TimerRegistry() : registry_id_(registry_id_counter_.fetch_add(1, std::memory_order_relaxed)) {
        if (registry_id_ >= MAX_REGISTRIES) {
            std::cerr << "TimerRegistry: MAX_REGISTRIES (" << MAX_REGISTRIES << ") exceeded. Increase the limit.\n";
            std::abort();
        }
    }
    ~TimerRegistry() {
        std::lock_guard lock(mutex_);
        for (auto& [tid, local] : live_threads_) {
            local->registry = nullptr;  // prevent dangling pointer access
        }
    }
    TimerRegistry(const TimerRegistry&) = delete;
    TimerRegistry(TimerRegistry&&) = delete;
    auto operator=(const TimerRegistry&) -> TimerRegistry& = delete;
    auto operator=(TimerRegistry&&) -> TimerRegistry& = delete;

    // ── Hot path — compile-time API, O(1) array lookup ────────────────────

    /**
     * Starts the calling thread's timer for the compile-time name.
     * Returns a Slot* handle — pass to stop(Slot*) to skip even the array
     * lookup on the stop side.
     *
     *   auto* slot = reg.start<"db_query">();
     *   ... work ...
     *   reg.stop(slot);
     */
    template <CTString Name>
    auto start() -> Slot* {
        constexpr std::size_t hash = hash_name(Name);
        auto& slot = ct_get_or_create_slot<hash, Name>();
        slot.timer.start();
        return &slot;
    }

    /**
     * Stops via a Slot* handle returned by start<n>() — no lookup at all, O(1).
     * This is the preferred stop path and is used internally by make_scoped_timer.
     *
     * No-op if the timer is not currently running (mirrors Timer::stop() semantics
     * and prevents a second call from recording a stale last_lap_ value).
     */
    static void stop(Slot* slot) noexcept {
        if (!slot->timer.is_running()) {
            return;
        }
        slot->timer.stop();
        slot->stats.record(slot->timer.last_lap_ns());
    }

    /**
     * Stops a compile-time named timer by name — single array index lookup.
     * Prefer stop(Slot*) in tight loops; use this for readability elsewhere.
     */
    template <CTString Name>
    void stop() {
        const std::size_t slot_id = CtSlotID<hash_name(Name)>::value;
        stop(&thread_local_storage().ct_slots[slot_id]);
    }

    /**
     * Returns true if the calling thread's timer for Name is currently running.
     * O(1), lock-free.
     */
    template <CTString Name>
    [[nodiscard]] auto is_running() const -> bool {
        const std::size_t slot_id = CtSlotID<hash_name(Name)>::value;
        return thread_local_storage().ct_slots[slot_id].timer.is_running();
    }

    /**
     * Returns elapsed time for the calling thread's timer for Name.
     * O(1), lock-free.
     */
    template <CTString Name, timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto elapsed() const -> double {
        const std::size_t slot_id = CtSlotID<hash_name(Name)>::value;
        return thread_local_storage().ct_slots[slot_id].timer.elapsed<D>();
    }

    /**
     * Returns a copy of the calling thread's accumulated stats for Name.
     * O(1), lock-free.
     */
    template <CTString Name>
    [[nodiscard]] auto stats() const -> TimerStats {
        const std::size_t slot_id = CtSlotID<hash_name(Name)>::value;
        return thread_local_storage().ct_slots[slot_id].stats;
    }

    // ── Slow path — acquires mutex ────────────────────────────────────────

    /**
     * Resets all threads' timers and stats for Name.
     * The name remains registered; start<n>() can be called again immediately.
     */
    template <CTString Name>
    void reset() {
        const std::size_t slot_id = CtSlotID<hash_name(Name)>::value;
        std::lock_guard lock(mutex_);
        for (auto& [tid, local] : live_threads_) {
            if (!local->ct_active[slot_id]) {
                continue;
            }
            local->ct_slots[slot_id].timer.reset();
            local->ct_slots[slot_id].stats.reset();
        }
        const auto& name = ct_names_[slot_id];
        if (!name.empty() && graveyard_.contains(name)) {
            graveyard_[name].reset();
        }
        auto& thr_grv = thread_graveyard_;
        thr_grv.erase(std::remove_if(thr_grv.begin(), thr_grv.end(), [&](const ThreadStatsRow& row) { return row.name == name; }), thr_grv.end());
    }

    // ── Report types ──────────────────────────────────────────────────────

    /** One row in a merged (per-name) report. All time fields in the requested Duration. */
    struct StatsRow {
        std::string name;
        std::size_t thread_count;
        std::size_t call_count;
        double total;
        double mean;
        double min;
        double max;
        double stddev;
        double sample_stddev;
    };

    /** One row in a per-thread report. All time fields in the requested Duration. */
    struct ThreadStatsRow {
        std::string name;
        thread_id tid;
        std::size_t call_count;
        double total;
        double mean;
        double min;
        double max;
        double stddev;
        double sample_stddev;
        double M2;
    };

    // ── Report accessors ──────────────────────────────────────────────────
    /**
     * Returns a simple elapsed-time report — one value per name, summed across
     * all live threads and exited threads (graveyard).
     * Useful for a quick overview without the full Welford stats table.
     *
     * Returns a snapshot elapsed-time report — one value per name, summed across
     * all live threads and exited threads (graveyard).
     * Only completed laps (where stop() has been called) are included.
     * Any currently-running timer's in-flight time is excluded from this snapshot.
     * For a live view that includes in-flight time, read timer.elapsed() directly.
     */
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_totals_report() const -> std::vector<std::pair<std::string, double>> {
        std::vector<std::pair<std::string, double>> result;
        std::lock_guard lock(mutex_);

        for (const auto& name : known_names_order_) {
            double total = 0.0;

            // Exited threads — already summed into graveyard_
            auto iter = graveyard_.find(name);
            if (iter != graveyard_.end()) {
                total += timer_detail::ns_to<D>(iter->second.total);
            }

            // Live threads — find the slot index for this name, then sum
            for (std::size_t i = 0; i < MAX_CT_TIMERS; ++i) {
                if (ct_names_[i] != name) {
                    continue;
                }
                for (const auto& [tid, local] : live_threads_) {
                    if (!local->ct_active[i]) {
                        continue;
                    }
                    // Snapshot: only count completed laps (stop() has been called).
                    // In-flight time from a currently-running timer is excluded.
                    total += timer_detail::ns_to<D>(local->ct_slots[i].stats.total);
                }
                break;  // name is unique across slots, no need to keep scanning
            }

            result.emplace_back(name, total);
        }
        return result;
    }

    /**
     * Returns a merged report — one row per name, stats aggregated across all
     * threads (including exited ones) via parallel Welford.
     */
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_stats_report() const -> std::vector<StatsRow> {
        std::vector<StatsRow> result;
        std::lock_guard lock(mutex_);
        for (const auto& name : known_names_order_) {
            TimerStats merged;
            std::size_t thread_count = 0;
            // Count individual exited threads from the per-thread graveyard.
            for (const auto& row : thread_graveyard_) {
                if (row.name != name) {
                    continue;
                }
                TimerStats stats;
                stats.count = row.call_count;
                stats.total = row.total;
                stats.mean = row.mean;
                stats.min = row.min;
                stats.max = row.max;
                // Reconstruct M2 from stddev: M2 = stddev^2 * count
                stats.M2 = row.M2;
                merged.merge(stats);
                ++thread_count;
            }
            // Add live threads.
            for (const auto& [tid, local] : live_threads_) {
                for (std::size_t i = 0; i < MAX_CT_TIMERS; ++i) {
                    if (!local->ct_active[i] || ct_names_[i] != name) {
                        continue;
                    }
                    if (local->ct_slots[i].stats.count > 0) {
                        merged.merge(local->ct_slots[i].stats);
                        ++thread_count;
                    }
                }
            }
            if (merged.count == 0) {
                continue;
            }
            result.push_back({
                name,
                thread_count,
                merged.count,
                merged.get_total<D>(),
                merged.get_mean<D>(),
                merged.get_min<D>(),
                merged.get_max<D>(),
                merged.get_stddev<D>(),
                merged.get_sample_stddev<D>(),
            });
        }
        return result;
    }

    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto get_thread_report() const -> std::vector<ThreadStatsRow> {
        std::vector<ThreadStatsRow> result;
        std::lock_guard lock(mutex_);
        for (const auto& row : thread_graveyard_) {
            result.push_back({
                row.name,
                row.tid,
                row.call_count,
                timer_detail::ns_to<D>(row.total),
                timer_detail::ns_to<D>(row.mean),
                timer_detail::ns_to<D>(row.min),
                timer_detail::ns_to<D>(row.max),
                timer_detail::ns_to<D>(row.stddev),
                timer_detail::ns_to<D>(row.sample_stddev),
            });
        }
        for (const auto& [tid, local] : live_threads_) {
            for (std::size_t i = 0; i < MAX_CT_TIMERS; ++i) {
                if (!local->ct_active[i] || local->ct_slots[i].stats.count == 0) {
                    continue;
                }
                const auto& stats = local->ct_slots[i].stats;
                result.push_back({
                    ct_names_[i],
                    tid,
                    stats.count,
                    stats.get_total<D>(),
                    stats.get_mean<D>(),
                    stats.get_min<D>(),
                    stats.get_max<D>(),
                    stats.get_stddev<D>(),
                    stats.get_sample_stddev<D>(),
                });
            }
        }
        return result;
    }

    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard]] auto totals_report_to_str() const -> std::string {
        std::stringstream ost;
        const auto rows = get_totals_report<D>();
        if (rows.empty()) {
            return ost.str();
        }

        // Find the longest name for alignment
        std::size_t name_width = 0;
        for (const auto& [name, _] : rows) {
            name_width = std::max(name_width, name.size());
        }

        for (const auto& [name, value] : rows) {
            ost << std::left << std::setw(static_cast<int>(name_width)) << name << std::right << std::setw(10) << std::fixed << std::setprecision(2)
                << value << "  " << timer_detail::unit_name<D>() << "\n";
        }

        return ost.str();
    }

    /**
     * Prints the simple elapsed-time report — one line per timer name.
     *
     * Example output:
     *   db_query:    42.31 ms
     *   render:     120.07 ms
     */
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    void print_totals_report() const {
        std::cout << totals_report_to_str<D>();
    }

    // ── Deprecated aliases (use the totals_* names above) ─────────────────
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard, deprecated("use get_totals_report()")]] auto get_report() const {
        return get_totals_report<D>();
    }
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[nodiscard, deprecated("use totals_report_to_str()")]] auto report_to_str() const {
        return totals_report_to_str<D>();
    }
    template <timer_detail::ValidDuration D = std::chrono::milliseconds>
    [[deprecated("use print_totals_report()")]] void print_report() const {
        print_totals_report<D>();
    }

    [[nodiscard]] auto stats_report_to_str() const -> std::string {
        std::stringstream ost;
        const auto rows = get_stats_report<std::chrono::nanoseconds>();
        if (rows.empty()) {
            return ost.str();
        }

        using namespace timer_detail;
        auto cmin = [&](auto func) -> auto { return col_min<StatsRow>(rows, func); };
        auto [dt, ut] = pick_unit(cmin([](const auto& row) { return row.total; }));
        auto [dm, um] = pick_unit(cmin([](const auto& row) { return row.mean; }));
        auto [di, ui] = pick_unit(cmin([](const auto& row) { return row.min; }));
        auto [dx, ux] = pick_unit(cmin([](const auto& row) { return row.max; }));

        constexpr int NAME_WIDTH = 24;
        constexpr int THREAD_WIDTH = 9;
        constexpr int VALUE_WIDTH = 14;
        constexpr int THREAD_COLUMNS = 2;
        constexpr int VALUE_COLUMNS = 5;

        auto hdr = [](const char* key, const char* unit) -> std::string { return std::string(key) + "(" + unit + ")"; };

        ost << std::left << std::setw(NAME_WIDTH) << "Timer" << std::right << std::setw(THREAD_WIDTH) << "Threads" << std::setw(THREAD_WIDTH)
            << "Calls" << std::setw(VALUE_WIDTH) << hdr("Total", ut) << std::setw(VALUE_WIDTH) << hdr("Mean", um) << std::setw(VALUE_WIDTH)
            << hdr("Min", ui) << std::setw(VALUE_WIDTH) << hdr("Max", ux) << std::setw(VALUE_WIDTH) << hdr("Stddev", um) << "\n"
            << std::string(NAME_WIDTH + (THREAD_WIDTH * THREAD_COLUMNS) + (VALUE_WIDTH * VALUE_COLUMNS), '-') << "\n";
        for (const auto& row : rows) {
            ost << std::left << std::setw(NAME_WIDTH) << row.name << std::right << std::setw(THREAD_WIDTH) << row.thread_count
                << std::setw(THREAD_WIDTH) << row.call_count << std::fixed << std::setprecision(2) << std::setw(VALUE_WIDTH) << row.total / dt
                << std::setw(VALUE_WIDTH) << row.mean / dm << std::setw(VALUE_WIDTH) << row.min / di << std::setw(VALUE_WIDTH) << row.max / dx
                << std::setw(VALUE_WIDTH) << row.stddev / dm << "\n";
        }

        return ost.str();
    }

    /**
     * Prints the merged statistics report.
     * Numbers are right-aligned, 2 decimal places. Each column independently
     * selects the most readable unit based on the smallest value in that column.
     */
    void print_stats_report() const { std::cout << stats_report_to_str(); }

    [[nodiscard]] auto thread_report_to_str() const -> std::string {
        std::stringstream ost;

        const auto rows = get_thread_report<std::chrono::nanoseconds>();
        if (rows.empty()) {
            return ost.str();
        }
        using namespace timer_detail;
        auto cmin = [&](auto func) -> auto { return col_min<ThreadStatsRow>(rows, func); };
        auto [dt, ut] = pick_unit(cmin([](const auto& row) { return row.total; }));
        auto [dm, um] = pick_unit(cmin([](const auto& row) { return row.mean; }));
        auto [di, ui] = pick_unit(cmin([](const auto& row) { return row.min; }));
        auto [dx, ux] = pick_unit(cmin([](const auto& row) { return row.max; }));
        auto [ds, us] = pick_unit(cmin([](const auto& row) { return row.stddev > 0 ? row.stddev : std::numeric_limits<double>::max(); }));

        constexpr int NAME_WIDTH = 24;
        constexpr int IDW = 24;
        constexpr int CALL_WIDTH = 8;
        constexpr int VALUE_WIDTH = 14;
        constexpr int VALUE_COLUMNS = 5;

        auto hdr = [](const char* key, const char* unit) -> std::string { return std::string(key) + "(" + unit + ")"; };

        ost << std::left << std::setw(NAME_WIDTH) << "Timer" << std::setw(IDW) << "Thread ID" << std::right << std::setw(CALL_WIDTH) << "Calls"
            << std::setw(VALUE_WIDTH) << hdr("Total", ut) << std::setw(VALUE_WIDTH) << hdr("Mean", um) << std::setw(VALUE_WIDTH) << hdr("Min", ui)
            << std::setw(VALUE_WIDTH) << hdr("Max", ux) << std::setw(VALUE_WIDTH) << hdr("Stddev", us) << "\n"
            << std::string(NAME_WIDTH + IDW + CALL_WIDTH + (VALUE_WIDTH * VALUE_COLUMNS), '-') << "\n";
        for (const auto& row : rows) {
            std::ostringstream ostr;
            ostr << row.tid;
            ost << std::left << std::setw(NAME_WIDTH) << row.name << std::setw(IDW) << ostr.str() << std::right << std::setw(CALL_WIDTH)
                << row.call_count << std::fixed << std::setprecision(2) << std::setw(VALUE_WIDTH) << row.total / dt << std::setw(VALUE_WIDTH)
                << row.mean / dm << std::setw(VALUE_WIDTH) << row.min / di << std::setw(VALUE_WIDTH) << row.max / dx << std::setw(VALUE_WIDTH)
                << row.stddev / ds << "\n";
        }
        return ost.str();
    }

    /**
     * Prints the per-thread statistics report.
     * Same formatting rules as print_stats_report().
     */
    void print_thread_report() const { std::cout << thread_report_to_str(); }

   private:
    struct ThreadLocal {
        thread_id tid;

        // Compile-time slots — fixed array, indexed by CtSlotID::value.
        std::array<Slot, TimerRegistry::MAX_CT_TIMERS> ct_slots;
        std::array<bool, TimerRegistry::MAX_CT_TIMERS> ct_active{};

        TimerRegistry* registry = nullptr;

        ~ThreadLocal() {
            if (registry == nullptr) {
                return;
            }
            std::lock_guard lock(registry->mutex_);
            for (std::size_t i = 0; i < ct_active.size(); ++i) {
                if (!ct_active[i] || ct_slots[i].stats.count == 0) {
                    continue;
                }
                const auto& name = registry->ct_names_[i];
                registry->graveyard_[name].merge(ct_slots[i].stats);
                const auto& s = ct_slots[i].stats;
                registry->thread_graveyard_.push_back({name, tid, s.count, s.total, s.mean, s.min, s.max, s.stddev(), s.sample_stddev(), s.M2});
            }
            registry->live_threads_.erase(tid);
        }
    };

    // ── Compile-time slot creation ────────────────────────────────────────

    template <std::size_t Hash, CTString Name>
    auto ct_get_or_create_slot() -> Slot& {
        const std::size_t id = CtSlotID<Hash>::value;
        auto& tloc = thread_local_storage();

        if (!tloc.ct_active[id]) {
            std::lock_guard lock(mutex_);
            if (ct_names_[id].empty()) {
                ct_names_[id] = std::string(Name.view());
                if (!known_names_.contains(ct_names_[id])) {
                    known_names_.emplace(ct_names_[id]);
                    known_names_order_.push_back(ct_names_[id]);
                }
            } else if (ct_names_[id] != Name.view()) {
                throw std::logic_error(std::string("TimerRegistry: FNV-1a hash collision between '") + ct_names_[id] + "' and '" +
                                       std::string(Name.view()) + "'");
            }
            if (tloc.registry == nullptr) {
                tloc.tid = std::this_thread::get_id();
                tloc.registry = this;
                live_threads_[tloc.tid] = &tloc;
            }
            tloc.ct_active[id] = true;
        }
        return tloc.ct_slots[id];
    }

    // Each registry instance has a unique registry_id_ (assigned at construction).
    // The thread_local array is indexed by that ID — one pointer load on the hot path instead of an unordered_map hash + probe on every
    // start()/stop() call. The problem here is that thread_local is static under the same thread: different TimerRegistry on the same thread won't be
    // independent
    auto thread_local_storage() const -> ThreadLocal& {
        thread_local std::array<std::unique_ptr<ThreadLocal>, MAX_REGISTRIES> tl_ptrs{};
        auto& ptr = tl_ptrs[registry_id_];
        if (!ptr) [[unlikely]] {
            ptr = std::make_unique<ThreadLocal>();
        }
        return *ptr;
    }

    // ── State — all guarded by mutex_ ─────────────────────────────────────

    mutable std::mutex mutex_;
    std::unordered_map<thread_id, ThreadLocal*> live_threads_;
    std::unordered_map<std::string, TimerStats> graveyard_;
    std::vector<ThreadStatsRow> thread_graveyard_;
    std::unordered_set<std::string> known_names_;
    std::vector<std::string> known_names_order_;

    // Maps compile-time slot IDs back to their string names (for reporting).
    // Written once at first use, read-only after that.
    std::array<std::string, MAX_CT_TIMERS> ct_names_;

    // ── Per-registry identity — drives the thread_local_storage() index ───
    // Assigned once at construction; stable for the lifetime of the registry.
    inline static std::atomic<std::size_t> registry_id_counter_{0};
    const std::size_t registry_id_;

   protected:
    /** Returns the unique per-instance registry ID used to index thread-local storage.
     *  Subclasses (e.g. StatsRegistry) can use this to share the same index space. */
    auto get_registry_id() const noexcept -> std::size_t { return registry_id_; }
};

// ─────────────────────────────────────────────────────────────────────────────
// CtSlotID<Hash> — maps a compile-time hash to a unique sequential slot index.
//
// The static member `value` is initialised exactly once at program startup
// via TimerRegistry::assign_id(). Every translation unit that uses
// start<"name">() will instantiate this template for that name's hash,
// ensuring a consistent ID across the whole program.
// ─────────────────────────────────────────────────────────────────────────────

template <std::size_t Hash>
const std::size_t CtSlotID<Hash>::value = TimerRegistry::assign_id(Hash);

// ─────────────────────────────────────────────────────────────────────────────
// make_scoped_timer — primary RAII entry point for registry-backed timers
//
// Construction calls start<n>() — O(1) array lookup after the first call per
// name per thread. Destruction calls stop(Slot*) — a single pointer
// dereference, no lookup of any kind.
//
// Usage:
//   auto t = make_scoped_timer<"db_query">(reg);
// ─────────────────────────────────────────────────────────────────────────────

template <CTString Name, timer_detail::ValidDuration D = std::chrono::milliseconds>
[[nodiscard]] auto make_scoped_timer(TimerRegistry& reg) {
    struct CtScopedTimer {
        TimerRegistry* registry_;
        TimerRegistry::Slot* slot_;

        explicit CtScopedTimer(TimerRegistry& r) : registry_(&r), slot_(r.template start<Name>()) {}

        ~CtScopedTimer() noexcept { registry_->stop(slot_); }

        CtScopedTimer(const CtScopedTimer&) = delete;
        CtScopedTimer(CtScopedTimer&&) = delete;
        auto operator=(const CtScopedTimer&) -> CtScopedTimer& = delete;
        auto operator=(CtScopedTimer&&) -> CtScopedTimer& = delete;
    };
    return CtScopedTimer{reg};
}

// ─────────────────────────────────────────────────────────────────────────────
// ScopedTimer — standalone (no-registry) RAII timer
//
// Prints "name: elapsed unit\n" to stdout on destruction.
// For registry-backed timing use make_scoped_timer<"name">(reg) instead.
// ─────────────────────────────────────────────────────────────────────────────

template <timer_detail::ValidDuration D = std::chrono::milliseconds>
class ScopedTimer {
   public:
    explicit ScopedTimer(std::string name) : name_(std::move(name)) { timer_.start(); }

    ~ScopedTimer() noexcept {
        timer_.stop();
        std::cout << name_ << ": " << timer_.elapsed<D>() << " " << timer_detail::unit_name<D>() << "\n";
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer(ScopedTimer&&) = delete;
    auto operator=(const ScopedTimer&) -> ScopedTimer& = delete;
    auto operator=(ScopedTimer&&) -> ScopedTimer& = delete;

   private:
    std::string name_;
    Timer timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Global convenience
// ─────────────────────────────────────────────────────────────────────────────

inline auto global_timers() -> TimerRegistry& {
    static TimerRegistry inst;
    return inst;
}

#define TIMER_REG ::utilz::global_timers()

}  // namespace utilz
