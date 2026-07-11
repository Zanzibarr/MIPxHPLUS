#pragma once

/**
 * @file stats_registry.hxx
 * @brief Counters, gauges, and histograms registry built on top of TimerRegistry.
 * @version 2.2.0
 *
 * @details
 * `StatsRegistry` extends `TimerRegistry` (from `timer.hxx`) with three
 * additional measurement primitives, all indexed by compile-time names:
 *
 *   - **Counters** — `counter_inc<Name>()` / `counter_add<Name>(n)` use a
 *     `std::atomic<int64_t>` per slot; the hot path is a single lock-free
 *     fetch_add with no branching.
 *   - **Gauges** — `gauge_record<Name>(v)` records a `double` sample using
 *     Welford's online algorithm (`GaugeStats` is a type alias for
 *     `timer_detail::WelfordAccumulator` from `timer.hxx`).
 *   - **Histograms** — `histogram_record<Name>(v)` slots a `double` into one
 *     of up to 16 user-defined bins; bins are specified as upper-bound edges
 *     via `set_histogram_bounds<Name>()`.
 *
 * Reports (`get_report()` / `print_report()`) return a `StatsReport` that
 * bundles all four metric types in insertion order.
 *
 * The global registry is accessible via `STATS_REG` (macro alias for
 * `global_stats_registry()`).
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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "timer.hxx"

namespace utilz {
// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace stats_detail {

// Welford accumulator for gauge values (doubles).
// Reuses the shared WelfordAccumulator from timer_detail (see timer.hxx).
using GaugeStats = timer_detail::WelfordAccumulator;

// Fixed-width histogram (equal-width buckets).
struct Histogram {
    double low;
    double high;
    std::vector<std::size_t> buckets;
    std::size_t underflow = 0;
    std::size_t overflow = 0;
    std::size_t total = 0;

    explicit Histogram(double low, double high, std::size_t n_buckets) : low(low), high(high), buckets(n_buckets, 0) {}

    void record(double val) {
        ++total;
        if (val < low) {
            ++underflow;
            return;
        }
        if (val >= high) {
            ++overflow;
            return;
        }
        double width = (high - low) / static_cast<double>(buckets.size());
        auto idx = static_cast<std::size_t>((val - low) / width);
        if (idx >= buckets.size()) {
            idx = buckets.size() - 1;
        }
        ++buckets[idx];
    }

    void reset() {
        std::ranges::fill(buckets, 0);
        underflow = overflow = total = 0;
    }
};

enum class SlotKind : uint8_t { Unset, Counter, Gauge, Histogram, Series };

// One timestamped sample in a Series.
struct SeriesPoint {
    int64_t timestamp_ns;  ///< steady_clock nanoseconds since epoch
    double value;
};

}  // namespace stats_detail

// ─────────────────────────────────────────────────────────────────────────────
// CtStatID — forward declaration; defined after StatsRegistry
// ─────────────────────────────────────────────────────────────────────────────

template <std::size_t Hash>
struct CtStatID {
    static const std::size_t value;
};

// ─────────────────────────────────────────────────────────────────────────────
// StatsRegistry
// ─────────────────────────────────────────────────────────────────────────────

/**
 * StatsRegistry — extends TimerRegistry with:
 *   - Counters   : increment / decrement / get occurrence counts
 *   - Gauges     : record fractional values, accumulate Welford statistics
 *   - Histograms : bucket sampled values, print distributions with ASCII bars
 *
 * All names are compile-time CTString template parameters.
 * Lookup is O(1) array indexing on the hot path.
 *
 * Thread safety
 * ─────────────
 * Counter hot path   : atomic, fully lock-free.
 * Gauge hot path     : per-entry mutex (fine-grained).
 * Histogram hot path : per-entry mutex (fine-grained).
 * All report / reset paths take stats_mutex_.
 */
class StatsRegistry : public TimerRegistry {
   public:
    StatsRegistry() = default;

    ~StatsRegistry() {
        // Prevent use-after-free: if live threads outlive this registry, their
        // SeriesThreadLocal destructor must not attempt to lock stats_mutex_.
        std::lock_guard lock(stats_mutex_);
        for (auto& [tid, thr_local] : series_live_threads_) {
            thr_local->registry = nullptr;
        }
    }
    StatsRegistry(const StatsRegistry&) = delete;
    StatsRegistry(StatsRegistry&&) = delete;
    auto operator=(const StatsRegistry&) -> StatsRegistry& = delete;
    auto operator=(StatsRegistry&&) -> StatsRegistry& = delete;

    // Maximum number of distinct compile-time stat names across the whole
    // program. Shared across counters, gauges, and histograms — each name
    // occupies one slot regardless of which primitive uses it.
    // Raise if you hit the abort() in assign_stat_id().
    // Keep in sync with MAX_CT_TIMERS (timer/timer.hxx)
    // and MAX_CT_PARAMS (parameters/parameters.hxx) — all default to 128.
    static constexpr std::size_t MAX_CT_STATS = 128;

    // Default number of histogram buckets
    static constexpr std::size_t DEFAULT_HISTOGRAM_BUCKETS = 10;
    // Default width for histogram bar chart (in characters)
    static constexpr int DEFAULT_HISTOGRAM_BAR_WIDTH = 40;
    // Default per-(thread, series) pre-reserved capacity (number of SeriesPoints).
    // Each SeriesPoint is 16 bytes, so the default reserves 16 KiB per slot per thread.
    // Override per-series at startup with series_create<Name>(capacity).
    static constexpr std::size_t DEFAULT_SERIES_CAPACITY = 1024;
    // Width for percentage column in histogram output
    static constexpr int HISTOGRAM_PERCENTAGE_WIDTH = 6;

    /**
     * Assigns a unique sequential index to a compile-time stat hash.
     * Called once per unique CTString at static-init time via CtStatID<H>::value.
     * Thread-safe via a static atomic counter.
     */
    static auto assign_stat_id(std::size_t /*hash*/) -> std::size_t {
        static std::atomic<std::size_t> next{0};
        std::size_t idx = next.fetch_add(1, std::memory_order_relaxed);
        if (idx >= MAX_CT_STATS) {
            std::cerr << "StatsRegistry: MAX_CT_STATS (" << MAX_CT_STATS << ") exceeded. Increase the limit.\n";
            std::abort();
        }
        return idx;
    }

    // ═════════════════════════════════════════════════════════════════════
    // COUNTERS
    // Counts discrete occurrences. Backed by std::atomic<int64_t>, fully
    // lock-free on the hot path.
    // ═════════════════════════════════════════════════════════════════════

    /** Increments counter Name by delta (default 1). */
    template <CTString Name>
    void counter_inc(int64_t delta = 1) {
        ct_counters_[ct_stat_id<Name>()].fetch_add(delta, std::memory_order_relaxed);
        ct_ensure_counter_name<Name>();
    }

    /** Decrements counter Name by delta (default 1). */
    template <CTString Name>
    void counter_dec(int64_t delta = 1) {
        ct_counters_[ct_stat_id<Name>()].fetch_sub(delta, std::memory_order_relaxed);
        ct_ensure_counter_name<Name>();
    }

    /** Sets counter Name to an explicit value. */
    template <CTString Name>
    void counter_set(int64_t value) {
        ct_counters_[ct_stat_id<Name>()].store(value, std::memory_order_relaxed);
        ct_ensure_counter_name<Name>();  // register so counter appears in reports
    }

    /** Returns the current value of counter Name.
     * Also registers the counter so it appears in get_counter_report().
     */
    template <CTString Name>
    [[nodiscard]] auto counter_get() -> int64_t {
        ct_ensure_counter_name<Name>();
        return ct_counters_[ct_stat_id<Name>()].load(std::memory_order_relaxed);
    }

    /**
     * Returns a stable pointer to the underlying atomic<int64_t> for Name.
     * Cache this in tight loops to bypass even the array lookup:
     *
     *   auto* ctr = reg.counter_ref<"hits">();
     *   for (...) ctr->fetch_add(1, std::memory_order_relaxed);
     *
     * The pointer remains valid for the lifetime of the registry.
     */
    template <CTString Name>
    [[nodiscard]] auto counter_ref() -> std::atomic<int64_t>* {
        ct_ensure_counter_name<Name>();
        return &ct_counters_[ct_stat_id<Name>()];
    }

    /** Resets counter Name to 0. */
    template <CTString Name>
    void counter_reset() noexcept {
        ct_counters_[ct_stat_id<Name>()].store(0, std::memory_order_relaxed);
    }

    // ── Counter reporting ─────────────────────────────────────────────────

    struct CounterRow {
        std::string name;
        int64_t value;
    };

    [[nodiscard]] auto get_counter_report() const -> std::vector<CounterRow> {
        std::lock_guard lock(stats_mutex_);
        std::vector<CounterRow> result;
        for (std::size_t i = 0; i < MAX_CT_STATS; ++i) {
            if (!ct_counter_active_[i]) {
                continue;
            }
            result.push_back({ct_stat_names_[i], ct_counters_[i].load(std::memory_order_relaxed)});
        }
        return result;
    }

    [[nodiscard]] auto counter_report_to_str() const -> std::string {
        std::stringstream ost;
        auto rows = get_counter_report();
        if (rows.empty()) {
            return ost.str();
        }
        constexpr std::size_t DEFAULT_NAME_WIDTH = 8;
        std::size_t name_width = DEFAULT_NAME_WIDTH;
        for (const auto& row : rows) {
            name_width = std::max(name_width, row.name.size() + 2);
        }
        constexpr int VAL_WIDTH = 14;
        ost << std::left << std::setw(static_cast<int>(name_width)) << "Counter" << std::right << std::setw(VAL_WIDTH) << "Value" << "\n"
            << std::string(name_width + VAL_WIDTH, '-') << "\n";
        for (const auto& row : rows) {
            ost << std::left << std::setw(static_cast<int>(name_width)) << row.name << std::right << std::setw(VAL_WIDTH) << row.value << "\n";
        }
        return ost.str();
    }

    void print_counter_report() const { std::cout << counter_report_to_str(); }

    /** Returns the CounterRow for Name, registering it if not yet seen. */
    template <CTString Name>
    [[nodiscard]] auto counter_get_row() -> CounterRow {
        ct_ensure_counter_name<Name>();
        const std::size_t idx = ct_stat_id<Name>();
        return {.name = std::string(Name.view()), .value = ct_counters_[idx].load(std::memory_order_relaxed)};
    }

    // ═════════════════════════════════════════════════════════════════════
    // GAUGES
    // Accumulates fractional (double) samples; reports Welford statistics
    // identical in style to TimerRegistry's stats report.
    // ═════════════════════════════════════════════════════════════════════

    /** Records a new sample for gauge Name. */
    template <CTString Name>
    void gauge_record(double value) {
        // Register name first (acquires stats_mutex_) — must happen BEFORE
        // entry.mtx is taken to preserve lock order: stats_mutex_ → entry.mtx.
        // get_gauge_report() takes stats_mutex_ then entry.mtx; inverting the
        // order here would cause a deadlock on the first concurrent call.
        ct_ensure_gauge_name<Name>();
        auto& entry = ct_gauges_[ct_stat_id<Name>()];
        std::lock_guard lock(entry.mtx);
        entry.stats.record(value);
    }

    /** Resets all samples for gauge Name. */
    template <CTString Name>
    void gauge_reset() {
        auto& entry = ct_gauges_[ct_stat_id<Name>()];
        std::lock_guard lock(entry.mtx);
        entry.stats.reset();
    }

    // ── Gauge reporting ───────────────────────────────────────────────────

    struct GaugeRow {
        std::string name;
        std::size_t count;
        double total;
        double mean;
        double min;
        double max;
        double stddev;
        double sample_stddev;
    };

    [[nodiscard]] auto get_gauge_report() const -> std::vector<GaugeRow> {
        std::lock_guard lock(stats_mutex_);
        std::vector<GaugeRow> result;
        for (std::size_t i = 0; i < MAX_CT_STATS; ++i) {
            if (!ct_gauge_active_[i]) {
                continue;
            }
            std::lock_guard glock(ct_gauges_[i].mtx);
            if (ct_gauges_[i].stats.count == 0) {
                continue;
            }
            const auto& stats = ct_gauges_[i].stats;
            result.push_back({ct_stat_names_[i], stats.count, stats.total, stats.mean, stats.min, stats.max, stats.stddev(), stats.sample_stddev()});
        }
        return result;
    }

    [[nodiscard]] auto gauge_report_to_str() const -> std::string {
        std::stringstream ost;
        const auto rows = get_gauge_report();
        if (rows.empty()) {
            return ost.str();
        }
        constexpr std::size_t DEFAULT_GAUGE_NAME_WIDTH = 8;
        std::size_t name_width = DEFAULT_GAUGE_NAME_WIDTH;
        for (const auto& row : rows) {
            name_width = std::max(name_width, row.name.size() + 2);
        }
        constexpr int TAG_WIDTH = 9;
        constexpr int VAL_WIDTH = 14;
        constexpr int NUM_VALUE_COLUMNS = 5;
        ost << std::left << std::setw(static_cast<int>(name_width)) << "Gauge" << std::right << std::setw(TAG_WIDTH) << "Samples"
            << std::setw(VAL_WIDTH) << "Total" << std::setw(VAL_WIDTH) << "Mean" << std::setw(VAL_WIDTH) << "Min" << std::setw(VAL_WIDTH) << "Max"
            << std::setw(VAL_WIDTH) << "Stddev" << "\n"
            << std::string(name_width + TAG_WIDTH + (VAL_WIDTH * NUM_VALUE_COLUMNS), '-') << "\n";
        for (const auto& row : rows) {
            ost << std::left << std::setw(static_cast<int>(name_width)) << row.name << std::right << std::setw(TAG_WIDTH) << row.count << std::fixed
                << std::setprecision(4) << std::setw(VAL_WIDTH) << row.total << std::setw(VAL_WIDTH) << row.mean << std::setw(VAL_WIDTH) << row.min
                << std::setw(VAL_WIDTH) << row.max << std::setw(VAL_WIDTH) << row.stddev << "\n";
        }
        return ost.str();
    }

    void print_gauge_report() const { std::cout << gauge_report_to_str(); }

    /** Returns the GaugeRow for Name (empty-count row if never recorded). */
    template <CTString Name>
    [[nodiscard]] auto gauge_get() -> GaugeRow {
        ct_ensure_gauge_name<Name>();
        const std::size_t idx = ct_stat_id<Name>();
        std::lock_guard lock(ct_gauges_[idx].mtx);
        const auto& stats = ct_gauges_[idx].stats;
        if (stats.count == 0) {
            return {std::string(Name.view()), 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        return {std::string(Name.view()), stats.count, stats.total, stats.mean, stats.min, stats.max, stats.stddev(), stats.sample_stddev()};
    }

    // ═════════════════════════════════════════════════════════════════════
    // HISTOGRAMS
    // Buckets sampled values into equal-width bins over [low, high).
    // histogram_create<n>() must be called once before histogram_record<n>().
    // ═════════════════════════════════════════════════════════════════════

    /**
     * Optional startup override — sets histogram bounds before the first record call.
     * If called again with the same bounds after creation, it is a no-op.
     * @throws std::logic_error if called with different bounds after already created.
     */
    template <CTString Name>
    void histogram_create(double low, double high, std::size_t n_buckets = DEFAULT_HISTOGRAM_BUCKETS) {
        if (low >= high) {
            throw std::invalid_argument("Histogram low must be < high.");
        }
        if (n_buckets == 0) {
            throw std::invalid_argument("n_buckets must be > 0.");
        }
        // Register name first (acquires stats_mutex_) — must happen BEFORE
        // entry.mtx is taken to preserve lock order: stats_mutex_ → entry.mtx.
        ct_ensure_hist_name<Name>();
        auto& entry = ct_hist_entries_[ct_stat_id<Name>()];
        std::lock_guard lock(entry.mtx);
        if (entry.created) {
            if (entry.hist.low != low || entry.hist.high != high || entry.hist.buckets.size() != n_buckets) {
                throw std::logic_error("Histogram already created with different bounds for '" + std::string(Name.view()) + "'.");
            }
            return;  // same params — no-op
        }
        entry.hist = stats_detail::Histogram{low, high, n_buckets};
        entry.created = true;
    }

    /**
     * Records value into histogram Name, lazily initialising with [low, high) if
     * histogram_create<Name>() has not been called.
     */
    template <CTString Name>
    void histogram_record(double val, double low = 0.0, double high = 1.0, std::size_t n_buckets = DEFAULT_HISTOGRAM_BUCKETS) {
        ct_ensure_hist_name<Name>();
        auto& entry = ct_hist_entries_[ct_stat_id<Name>()];
        std::lock_guard lock(entry.mtx);
        if (!entry.created) {
            entry.hist = stats_detail::Histogram{low, high, n_buckets};
            entry.created = true;
        }
        entry.hist.record(val);
    }

    /** Resets all bucket counts for histogram Name. */
    template <CTString Name>
    void histogram_reset() {
        auto& entry = ct_hist_entries_[ct_stat_id<Name>()];
        std::lock_guard lock(entry.mtx);
        entry.hist.reset();
    }

    // ── Histogram reporting ───────────────────────────────────────────────

    struct HistogramBucket {
        double low;
        double high;
        std::size_t count;
        double pct;
    };

    struct HistogramRow {
        std::string name;
        std::size_t total{};
        std::size_t underflow{};
        std::size_t overflow{};
        std::vector<HistogramBucket> buckets;
    };

    [[nodiscard]] auto get_histogram_report() const -> std::vector<HistogramRow> {
        std::lock_guard lock(stats_mutex_);
        std::vector<HistogramRow> result;
        for (std::size_t i = 0; i < MAX_CT_STATS; ++i) {
            if (!ct_hist_active_[i]) {
                continue;
            }
            std::lock_guard hlock(ct_hist_entries_[i].mtx);
            const auto& hist = ct_hist_entries_[i].hist;
            std::size_t in_range = hist.total - hist.underflow - hist.overflow;
            double width = (hist.high - hist.low) / static_cast<double>(hist.buckets.size());
            HistogramRow row;
            row.name = ct_stat_names_[i];
            row.total = hist.total;
            row.underflow = hist.underflow;
            row.overflow = hist.overflow;
            for (std::size_t buck_idx = 0; buck_idx < hist.buckets.size(); ++buck_idx) {
                double blo = hist.low + (static_cast<double>(buck_idx) * width);
                double bhi = blo + width;
                double pct = in_range > 0 ? 100.0 * static_cast<double>(hist.buckets[buck_idx]) / static_cast<double>(in_range) : 0.0;
                row.buckets.push_back({blo, bhi, hist.buckets[buck_idx], pct});
            }
            result.push_back(std::move(row));
        }
        return result;
    }

    [[nodiscard]] auto histogram_report_to_str(int bar_width = DEFAULT_HISTOGRAM_BAR_WIDTH) const -> std::string {
        std::stringstream ost;
        const auto rows = get_histogram_report();
        if (rows.empty()) {
            return ost.str();
        }
        for (const auto& row : rows) {
            ost << "── Histogram: " << row.name << "  [total=" << row.total << "  underflow=" << row.underflow << "  overflow=" << row.overflow
                << "] ──\n";
            std::size_t in_range = row.total - row.underflow - row.overflow;
            int label_width = 0;
            for (const auto& bucket : row.buckets) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << "[" << bucket.low << ", " << bucket.high << ")";
                label_width = std::max(label_width, static_cast<int>(oss.str().size()));
            }
            for (const auto& bucket : row.buckets) {
                std::ostringstream label;
                label << std::fixed << std::setprecision(2) << "[" << bucket.low << ", " << bucket.high << ")";
                int filled = in_range > 0 ? static_cast<int>(std::round(bucket.pct / 100.0 * bar_width)) : 0;
                ost << std::left << std::setw(label_width) << label.str() << " | " << std::string(static_cast<std::size_t>(filled), '#')
                    << std::string(static_cast<std::size_t>(std::max(0, bar_width - filled)), ' ') << " " << std::right
                    << std::setw(HISTOGRAM_PERCENTAGE_WIDTH) << std::fixed << std::setprecision(1) << bucket.pct << "%"
                    << "  (" << bucket.count << ")\n";
            }
            ost << "\n";
        }
        return ost.str();
    }

    /**
     * Prints histograms with ASCII bar charts.
     * @param bar_width Maximum number of '#' characters for 100%.
     */
    void print_histogram_report(int bar_width = DEFAULT_HISTOGRAM_BAR_WIDTH) const { std::cout << histogram_report_to_str(bar_width); }

    /** Returns the HistogramRow for Name (empty row if never recorded). */
    template <CTString Name>
    [[nodiscard]] auto histogram_get() -> HistogramRow {
        ct_ensure_hist_name<Name>();
        const std::size_t idx = ct_stat_id<Name>();
        std::lock_guard lock(ct_hist_entries_[idx].mtx);
        const auto& entry = ct_hist_entries_[idx];
        if (!entry.created) {
            return {.name = std::string(Name.view()), .total = 0, .underflow = 0, .overflow = 0, .buckets = {}};
        }
        const auto& hist = entry.hist;
        std::size_t in_range = hist.total - hist.underflow - hist.overflow;
        double width = (hist.high - hist.low) / static_cast<double>(hist.buckets.size());
        HistogramRow row;
        row.name = std::string(Name.view());
        row.total = hist.total;
        row.underflow = hist.underflow;
        row.overflow = hist.overflow;
        for (std::size_t i = 0; i < hist.buckets.size(); ++i) {
            double blo = hist.low + (static_cast<double>(i) * width);
            double bhi = blo + width;
            double pct = in_range > 0 ? 100.0 * static_cast<double>(hist.buckets[i]) / static_cast<double>(in_range) : 0.0;
            row.buckets.push_back({blo, bhi, hist.buckets[i], pct});
        }
        return row;
    }

    // ═════════════════════════════════════════════════════════════════════
    // SERIES
    // Records an ordered sequence of (timestamp, double) samples.
    // Push is lock-free on the hot path (after first call per thread/slot).
    // Merge is done on read; concurrent push+read for the same series is
    // not supported — call get_series_report() only after all pushes are done.
    // ═════════════════════════════════════════════════════════════════════

    /**
     * Appends {steady_clock::now(), value} to this thread's local buffer for
     * series Name. Lock-free after the first call per (thread, Name) pair.
     */
    template <CTString Name>
    void series_push(double value) {
        const std::size_t idx = ct_stat_id<Name>();
        auto& thr_local = series_thread_local_storage();

        // Register name + thread on first use (takes stats_mutex_ once).
        ct_ensure_series_name<Name>();
        if (!thr_local.active[idx]) {
            std::lock_guard lock(stats_mutex_);
            series_register_thread(idx, thr_local);
        }

        // Lazy reset: if a reset happened since the last push, clear local buffer.
        const uint64_t cur_epoch = ct_series_entries_[idx].reset_epoch.load(std::memory_order_acquire);
        if (cur_epoch != thr_local.last_epoch[idx]) {
            thr_local.series[idx].clear();
            thr_local.last_epoch[idx] = cur_epoch;
        }

        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        thr_local.series[idx].push_back({now_ns, value});
    }

    /**
     * Clears all stored points for series Name across all threads and the
     * archived buffer. Each thread's local buffer is cleared lazily on its
     * next series_push<Name>() call via the reset_epoch mechanism.
     */
    template <CTString Name>
    void series_reset() {
        const std::size_t idx = ct_stat_id<Name>();
        std::lock_guard lock(stats_mutex_);
        auto& entry = ct_series_entries_[idx];
        entry.archived.clear();
        // Also eagerly clear all live thread-local vectors (safe because
        // concurrent push+reset for the same series is not supported).
        for (auto* ptr : entry.live_ptrs) {
            ptr->clear();
        }
        entry.reset_epoch.fetch_add(1, std::memory_order_release);
    }

    /**
     * Pre-reserves per-thread storage for series Name.
     *
     * Call once at startup (before the first series_push<Name>()) to eliminate
     * reallocation spikes on the hot path. Every thread that subsequently pushes
     * to this series will have its local buffer reserved to @p capacity points
     * upfront. If not called, DEFAULT_SERIES_CAPACITY (1024) is used.
     *
     * @param capacity  Number of SeriesPoints to reserve per thread (16 bytes each).
     */
    template <CTString Name>
    void series_create(std::size_t capacity = DEFAULT_SERIES_CAPACITY) {
        ct_ensure_series_name<Name>();
        const std::size_t idx = ct_stat_id<Name>();
        std::lock_guard lock(stats_mutex_);
        ct_series_entries_[idx].capacity = capacity;
    }

    /**
     * Returns a sorted (by timestamp) copy of all points pushed to series Name
     * across all threads (including threads that have since exited).
     * Do not call concurrently with series_push<Name>().
     */
    template <CTString Name>
    [[nodiscard]] auto series_get() const -> std::vector<stats_detail::SeriesPoint> {
        const std::size_t idx = ct_stat_id<Name>();
        std::lock_guard lock(stats_mutex_);
        std::vector<stats_detail::SeriesPoint> merged = ct_series_entries_[idx].archived;
        for (auto* ptr : ct_series_entries_[idx].live_ptrs) {
            merged.insert(merged.end(), ptr->begin(), ptr->end());
        }
        std::sort(merged.begin(), merged.end(), [](const stats_detail::SeriesPoint& point_a, const stats_detail::SeriesPoint& point_b) {
            return point_a.timestamp_ns < point_b.timestamp_ns;
        });
        return merged;
    }

    // ── Series reporting ──────────────────────────────────────────────────

    struct SeriesRow {
        std::string name;
        std::vector<stats_detail::SeriesPoint> points;  ///< sorted by timestamp_ns
    };

    /**
     * Returns one SeriesRow per registered series, each with points sorted by
     * wall-clock timestamp. Do not call concurrently with series_push().
     */
    [[nodiscard]] auto get_series_report() const -> std::vector<SeriesRow> {
        std::lock_guard lock(stats_mutex_);
        std::vector<SeriesRow> result;
        for (std::size_t i = 0; i < MAX_CT_STATS; ++i) {
            if (!ct_series_active_[i]) {
                continue;
            }
            std::vector<stats_detail::SeriesPoint> merged = ct_series_entries_[i].archived;
            for (auto* ptr : ct_series_entries_[i].live_ptrs) {
                merged.insert(merged.end(), ptr->begin(), ptr->end());
            }
            if (merged.empty()) {
                continue;
            }
            std::ranges::sort(merged, [](const stats_detail::SeriesPoint& point_a, const stats_detail::SeriesPoint& point_b) {
                return point_a.timestamp_ns < point_b.timestamp_ns;
            });
            result.push_back({ct_stat_names_[i], std::move(merged)});
        }
        return result;
    }

    [[nodiscard]] auto series_report_to_str(std::size_t max_preview = 3) const -> std::string {
        std::stringstream ost;
        const auto rows = get_series_report();
        if (rows.empty()) {
            return ost.str();
        }
        for (const auto& row : rows) {
            const auto& pts = row.points;
            const int64_t timestamp_0 = pts.front().timestamp_ns;
            ost << "── Series: " << row.name << "  [n=" << pts.size() << "] ──\n  ";
            std::size_t shown = std::min(pts.size(), max_preview);
            for (std::size_t k = 0; k < shown; ++k) {
                if (k > 0) {
                    ost << ",  ";
                }
                double rel_ms = static_cast<double>(pts[k].timestamp_ns - timestamp_0) / 1e6;
                ost << std::fixed << std::setprecision(1) << rel_ms << "ms → " << pts[k].value;
            }
            if (pts.size() > max_preview) {
                const std::size_t remaining = pts.size() - max_preview;
                ost << "  … (+" << remaining << " more)";
            }
            if (pts.size() > 1) {
                ost << "  last: " << pts.back().value;
            }
            ost << "\n";
        }
        return ost.str();
    }

    void print_series_report(std::size_t max_preview = 3) const { std::cout << series_report_to_str(max_preview); }

    // ═════════════════════════════════════════════════════════════════════
    // AGGREGATE REPORT
    // ═════════════════════════════════════════════════════════════════════

    struct StatsReport {
        std::vector<StatsRow> timers;
        std::vector<CounterRow> counters;
        std::vector<GaugeRow> gauges;
        std::vector<HistogramRow> histograms;
        std::vector<SeriesRow> series;
    };

    [[nodiscard]] auto get_all_report() const -> StatsReport {
        return {
            .timers = get_stats_report(),
            .counters = get_counter_report(),
            .gauges = get_gauge_report(),
            .histograms = get_histogram_report(),
            .series = get_series_report(),
        };
    }

    // ═════════════════════════════════════════════════════════════════════
    // CONVENIENCE — print everything at once
    // ═════════════════════════════════════════════════════════════════════

    void print_all_reports() const {
        std::cout << "╔══ Timer Report ══════════════════════════════════╗\n";
        print_stats_report();
        std::cout << "\n╔══ Counter Report ════════════════════════════════╗\n";
        print_counter_report();
        std::cout << "\n╔══ Gauge Report ══════════════════════════════════╗\n";
        print_gauge_report();
        std::cout << "\n╔══ Histogram Report ══════════════════════════════╗\n";
        print_histogram_report();
        std::cout << "\n╔══ Series Report ═════════════════════════════════╗\n";
        print_series_report();
    }

   private:
    // ── Compile-time ID helper ────────────────────────────────────────────

    template <CTString Name>
    static auto ct_stat_id() -> std::size_t {
        return CtStatID<hash_name(Name)>::value;
    }

    // Shared DCLP helper — registers Name with Kind, sets active_flag.
    // Fast path: single branch on active_flag (already true after first call).
    template <CTString Name, stats_detail::SlotKind Kind>
    void ct_ensure_name(bool& active_flag) {
        if (active_flag) return;
        const std::size_t idx = ct_stat_id<Name>();
        std::lock_guard lock(stats_mutex_);
        if (active_flag) return;
        if (!ct_stat_names_[idx].empty() && ct_stat_names_[idx] != Name.view()) {
            throw std::logic_error(std::string("StatsRegistry: FNV-1a hash collision between '") + ct_stat_names_[idx] + "' and '" +
                                   std::string(Name.view()) + "'");
        }
        if (ct_stat_kinds_[idx] != stats_detail::SlotKind::Unset && ct_stat_kinds_[idx] != Kind) {
            throw std::logic_error("StatsRegistry: name '" + std::string(Name.view()) + "' already registered as a different primitive type");
        }
        ct_stat_names_[idx] = std::string(Name.view());
        ct_stat_kinds_[idx] = Kind;
        active_flag = true;
    }

    template <CTString Name>
    void ct_ensure_counter_name() {
        ct_ensure_name<Name, stats_detail::SlotKind::Counter>(ct_counter_active_[ct_stat_id<Name>()]);
    }

    template <CTString Name>
    void ct_ensure_gauge_name() {
        ct_ensure_name<Name, stats_detail::SlotKind::Gauge>(ct_gauge_active_[ct_stat_id<Name>()]);
    }

    template <CTString Name>
    void ct_ensure_hist_name() {
        ct_ensure_name<Name, stats_detail::SlotKind::Histogram>(ct_hist_active_[ct_stat_id<Name>()]);
    }

    // ── Storage ───────────────────────────────────────────────────────────

    // Lock ordering: if both locks are ever needed in the same code path,
    // always acquire mutex_ (TimerRegistry) before stats_mutex_ (StatsRegistry).
    // Currently no code path takes both simultaneously.
    mutable std::mutex stats_mutex_;

    // ── Counters — one atomic per slot, lock-free on the hot path ─────────
    std::array<std::atomic<int64_t>, MAX_CT_STATS> ct_counters_{};
    std::array<bool, MAX_CT_STATS> ct_counter_active_{};

    // ── Gauges — per-entry mutex + Welford stats ──────────────────────────
    struct CtGaugeEntry {
        mutable std::mutex mtx;
        stats_detail::GaugeStats stats;
    };
    std::array<CtGaugeEntry, MAX_CT_STATS> ct_gauges_;
    std::array<bool, MAX_CT_STATS> ct_gauge_active_{};

    // ── Histograms — per-entry mutex + optional Histogram ─────────────────
    // histogram_create<n>() initialises hist and sets created = true.
    // The placeholder Histogram in the default constructor is never recorded
    // into; histogram_record<n>() requires created == true.
    struct CtHistEntry {
        mutable std::mutex mtx;
        stats_detail::Histogram hist{0.0, 1.0, 1};  // placeholder
        bool created = false;
    };
    std::array<CtHistEntry, MAX_CT_STATS> ct_hist_entries_;
    std::array<bool, MAX_CT_STATS> ct_hist_active_{};

    // Name table — written once per name on first use, read-only thereafter.
    std::array<std::string, MAX_CT_STATS> ct_stat_names_;

    // Kind table — guards against the same name being registered as two different primitives.
    std::array<stats_detail::SlotKind, MAX_CT_STATS> ct_stat_kinds_{};

    // ── Series — per-slot archived + live thread-local vector pointers ────

    struct CtSeriesEntry {
        // Points flushed from threads that have since exited (protected by stats_mutex_).
        std::vector<stats_detail::SeriesPoint> archived;
        // Pointers to each live thread's thread-local vector for this slot.
        // Written under stats_mutex_; read under stats_mutex_ at report time.
        std::vector<std::vector<stats_detail::SeriesPoint>*> live_ptrs;
        // Monotonically increasing epoch; incremented by series_reset<Name>().
        // Each thread checks this on first push after a reset and clears its local buffer.
        std::atomic<uint64_t> reset_epoch{0};
        // Per-(thread, series) pre-reserved capacity. Set via series_create<Name>().
        // Applied in series_register_thread() so every thread that joins later also benefits.
        std::size_t capacity{DEFAULT_SERIES_CAPACITY};
    };

    std::array<CtSeriesEntry, MAX_CT_STATS> ct_series_entries_;
    std::array<bool, MAX_CT_STATS> ct_series_active_{};

    // Forward declaration so the map type is complete before SeriesThreadLocal is defined.
    struct SeriesThreadLocal;

    // Live-thread map for series (parallel to TimerRegistry::live_threads_).
    // Protected by stats_mutex_.
    std::unordered_map<std::thread::id, SeriesThreadLocal*> series_live_threads_;

    // Per-thread state for series data.
    struct SeriesThreadLocal {
        std::array<std::vector<stats_detail::SeriesPoint>, MAX_CT_STATS> series;
        std::array<bool, MAX_CT_STATS> active{};
        std::array<uint64_t, MAX_CT_STATS> last_epoch{};
        StatsRegistry* registry = nullptr;

        ~SeriesThreadLocal() {
            if (registry == nullptr) {
                return;
            }
            std::lock_guard lock(registry->stats_mutex_);
            for (std::size_t i = 0; i < MAX_CT_STATS; ++i) {
                if (!active[i]) {
                    continue;
                }
                auto& entry = registry->ct_series_entries_[i];
                // Flush to archive.
                entry.archived.insert(entry.archived.end(), series[i].begin(), series[i].end());
                // Remove this thread's vector pointer from live_ptrs to prevent dangling access.
                auto& ptrs = entry.live_ptrs;
                ptrs.erase(std::remove(ptrs.begin(), ptrs.end(), &series[i]), ptrs.end());
            }
            registry->series_live_threads_.erase(std::this_thread::get_id());
        }
    };

    auto series_thread_local_storage() -> SeriesThreadLocal& {
        thread_local std::array<std::unique_ptr<SeriesThreadLocal>, MAX_REGISTRIES> tl_ptrs{};
        auto& ptr = tl_ptrs[get_registry_id()];
        if (!ptr) [[unlikely]] {
            ptr = std::make_unique<SeriesThreadLocal>();
        }
        return *ptr;
    }

    template <CTString Name>
    void ct_ensure_series_name() {
        ct_ensure_name<Name, stats_detail::SlotKind::Series>(ct_series_active_[ct_stat_id<Name>()]);
    }

    // Registers the calling thread's local vector pointer for slot idx.
    // Must be called under stats_mutex_.
    void series_register_thread(std::size_t idx, SeriesThreadLocal& thr_local) {
        if (!thr_local.active[idx]) {
            auto& entry = ct_series_entries_[idx];
            thr_local.registry = this;
            entry.live_ptrs.push_back(&thr_local.series[idx]);
            series_live_threads_[std::this_thread::get_id()] = &thr_local;
            thr_local.active[idx] = true;
            entry.live_ptrs.back()->reserve(entry.capacity);
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// CtStatID — maps a compile-time hash to a unique sequential stat index.
//
// Instantiated once per unique CTString across the whole program.
// The static value is assigned at program startup via StatsRegistry::assign_stat_id().
// ─────────────────────────────────────────────────────────────────────────────

template <std::size_t Hash>
const std::size_t CtStatID<Hash>::value = StatsRegistry::assign_stat_id(Hash);

// ─────────────────────────────────────────────────────────────────────────────
// make_scoped_counter — RAII inc/dec for a compile-time named counter
//
// Construction increments, destruction decrements. Both operations are
// lock-free atomic array lookups — no map, no string, no mutex.
//
// Usage:
//   auto c = make_scoped_counter<"active_requests">(reg);
// ─────────────────────────────────────────────────────────────────────────────

template <CTString Name>
[[nodiscard]] auto make_scoped_counter(StatsRegistry& reg) {
    struct CtScopedCounter {
        std::atomic<int64_t>* ptr_;

        explicit CtScopedCounter(StatsRegistry& reg) : ptr_(reg.template counter_ref<Name>()) { ptr_->fetch_add(1, std::memory_order_relaxed); }

        ~CtScopedCounter() noexcept { ptr_->fetch_sub(1, std::memory_order_relaxed); }

        CtScopedCounter(const CtScopedCounter&) = delete;
        CtScopedCounter(CtScopedCounter&&) = delete;
        auto operator=(const CtScopedCounter&) -> CtScopedCounter& = delete;
        auto operator=(CtScopedCounter&&) -> CtScopedCounter& = delete;
    };
    return CtScopedCounter{reg};
}

// ─────────────────────────────────────────────────────────────────────────────
// Global convenience
// ─────────────────────────────────────────────────────────────────────────────

inline auto global_stats() -> StatsRegistry& {
    static StatsRegistry inst;
    return inst;
}

#define STATS_REG ::utilz::global_stats()

}  // namespace utilz
