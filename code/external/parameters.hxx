#pragma once

/**
 * @file parameters.hxx
 * @brief O(1) compile-time named parameter registry with type-safe get/set.
 * @version 1.1.0
 *
 * @details
 * `ParameterRegistry` stores named values indexed by `CTString` literals.
 * Each unique name is mapped at static-init time to a sequential slot index
 * via `CtParamID<hash_name(Name)>::value`, so every `get<Name, T>()` call
 * is a plain array lookup with no hashing or branching at runtime.
 *
 * Supported value types: `int`, `double`, `bool`, `std::string`.
 * Automatic coercion is applied on `set()`:
 *   - `bool` literals → `bool` (checked before the generic integral path)
 *   - integral types  → `int`
 *   - floating-point  → `double`
 *   - char* / string_view → `std::string`
 *
 * An explicit `StoredAs` template parameter overrides the coercion, which
 * is useful for resolving ambiguous literals (e.g. storing `0` as `double`).
 *
 * The global registry is accessible via `PARAMS` (macro alias for
 * `global_params()`).
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * Repository: https://github.com/Zanzibarr/cpp_utils
 *
 * SPDX-License-Identifier: MIT
 */

#include <array>
#include <atomic>
#include <format>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "ct_string.hxx"

namespace utilz {
// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace param_detail {

// Concept covering all types accepted by ParameterRegistry::set().
// bool satisfies is_integral too — it is listed explicitly so the intent
// is clear.  The actual dispatch order (bool before int) is enforced inside
// set()'s if constexpr chain, not by the concept itself.
template <typename T>
concept SupportedParamType = std::is_same_v<std::decay_t<T>, bool> || std::is_integral_v<std::decay_t<T>> ||
                             std::is_floating_point_v<std::decay_t<T>> || std::is_constructible_v<std::string, std::decay_t<T>>;

// Concept for the optional explicit StoredAs parameter in set().
// void means "auto-coerce" (the default); otherwise must be one of the four variant types.
template <typename T>
concept ValidStoredType =
    std::is_void_v<T> || std::is_same_v<T, int> || std::is_same_v<T, double> || std::is_same_v<T, bool> || std::is_same_v<T, std::string>;

}  // namespace param_detail

// ─────────────────────────────────────────────────────────────────────────────
// CtParamID — forward declaration; defined after ParameterRegistry
// ─────────────────────────────────────────────────────────────────────────────

template <std::size_t Hash>
struct CtParamID {
    static const std::size_t value;
};

// ─────────────────────────────────────────────────────────────────────────────
// ParameterRegistry
// ─────────────────────────────────────────────────────────────────────────────

/**
 * ParameterRegistry — stores named parameters indexed by compile-time strings.
 *
 * Parameters are written during initialisation (single-threaded) and then
 * read-only.  Lookup is O(1) array indexing — no hash map at runtime.
 *
 * Supported value types: int, double, bool, std::string.
 * Type coercions applied automatically on set():
 *   - bool literals        → bool     (checked before integral to avoid promotion)
 *   - integral types       → int
 *   - floating-point types → double
 *   - char* / char[] / string_view → std::string
 *
 * Thread safety
 * ─────────────
 * Not thread-safe.  All set() calls must complete before any get() call is
 * made from another thread.  No locking is provided or needed.
 *
 * Note: clear/reset is not supported.  ParameterRegistry is designed for the
 * write-once, read-many pattern.  Repeated set() calls on the same name
 * overwrite the previous value but do not change insertion order.
 *
 * Usage
 * ─────
 *   ParameterRegistry params;
 *   params.set<"learning_rate">(0.001);
 *   params.set<"batch_size">(32);         // stored as int
 *   params.set<"shuffle">(true);
 *   params.set<"optimizer">("adam");      // stored as std::string
 *
 *   double      lr  = params.get<"learning_rate", double>();
 *   int         bs  = params.get<"batch_size",    int>();
 *   bool        sh  = params.get<"shuffle",       bool>();
 *   std::string opt = params.get<"optimizer",     std::string>();
 *
 *   if (params.has<"dropout">()) { ... }
 *   std::cout << "set: " << params.size() << " params\n";
 *
 *   params.print_report();
 */
class ParameterRegistry {
   public:
    // Variant holding all supported parameter types.
    using Value = std::variant<int, double, bool, std::string>;

    // Maximum number of distinct compile-time parameter names across the whole
    // program.  Raise if assign_param_id() throws std::overflow_error.
    // Keep in sync with MAX_CT_TIMERS (timer/timer.hxx)
    // and MAX_CT_STATS (stats_registry/stats_registry.hxx) — all default to 128.
    static constexpr std::size_t MAX_CT_PARAMS = 128;

    /**
     * Assigns a unique sequential index to a compile-time parameter hash.
     * Called once per unique CTString at first-use time via CtParamID<H>::value.
     * Thread-safe via a static atomic counter.
     *
     * @throws std::overflow_error if more than MAX_CT_PARAMS unique names are used.
     */
    static auto assign_param_id(std::size_t /*hash*/) -> std::size_t {
        static std::atomic<std::size_t> next{0};
        const std::size_t idx = next.fetch_add(1, std::memory_order_relaxed);
        if (idx >= MAX_CT_PARAMS) {
            throw std::overflow_error(std::format("ParameterRegistry: MAX_CT_PARAMS ({}) exceeded — increase the limit.", MAX_CT_PARAMS));
        }
        return idx;
    }

    ParameterRegistry() = default;
    ~ParameterRegistry() = default;
    ParameterRegistry(const ParameterRegistry&) = default;
    ParameterRegistry(ParameterRegistry&&) = default;
    auto operator=(const ParameterRegistry&) -> ParameterRegistry& = default;
    auto operator=(ParameterRegistry&&) -> ParameterRegistry& = default;

    // ── Write ─────────────────────────────────────────────────────────────

    /**
     * Stores a value for compile-time name Name.
     * Calling set() again for the same name overwrites the previous value.
     *
     * @tparam Name      Compile-time parameter name (CTString).
     * @tparam StoredAs  Optional explicit target type (int, double, bool, std::string).
     *                   When omitted (default void), the stored type is auto-coerced from T.
     *                   Use this to resolve ambiguous literals, e.g.:
     *                     params.set<"threshold", double>(0);  // 0 is int, but stored as double
     * @tparam T         Deduced value type — must satisfy param_detail::SupportedParamType.
     */
    template <CTString Name, param_detail::ValidStoredType StoredAs = void, param_detail::SupportedParamType T>
    void set(T&& value) {
        using D = std::decay_t<T>;
        Value stored;
        if constexpr (!std::is_void_v<StoredAs>) {
            if constexpr (std::is_same_v<StoredAs, std::string>) {
                stored = std::string(std::forward<T>(value));
            } else {
                stored = static_cast<StoredAs>(value);
            }
        } else if constexpr (std::is_same_v<D, bool>) {
            stored = value;
        } else if constexpr (std::is_integral_v<D>) {
            stored = static_cast<int>(value);
        } else if constexpr (std::is_floating_point_v<D>) {
            stored = static_cast<double>(value);
        } else {
            stored = std::string(std::forward<T>(value));
        }

        const std::size_t idx = CtParamID<hash_name(Name)>::value;
        auto& slot = slots_[idx];

        if (slot.active && slot.name != Name.view()) {
            // Two different names mapped to the same slot — data corruption would occur silently.
            throw std::logic_error(
                std::format("ParameterRegistry: FNV-1a hash collision between '{}' and '{}' (same slot index)", slot.name, std::string(Name.view())));
        }

        slot.value = std::move(stored);
        if (!slot.active) {
            slot.name = std::string(Name.view());
            slot.active = true;
            active_indices_.push_back(idx);
        }
    }

    // ── Read ──────────────────────────────────────────────────────────────

    /**
     * Returns the stored value for Name.
     * Primitives (int, double, bool) are returned by value; std::string is
     * returned by const reference to avoid a copy on every call.
     * The reference is valid for the lifetime of the registry, provided
     * set() is not called again for the same name after the reference is taken.
     *
     * T must exactly match the type coerced during set<Name>():
     *   - integers  → int
     *   - floats    → double
     *   - bool      → bool
     *   - strings   → std::string
     *
     * @throws std::out_of_range   if Name has not been set.
     * @throws std::runtime_error  if T does not match the stored type.
     */
    template <CTString Name, typename T>
    [[nodiscard]] auto get() const -> std::conditional_t<std::is_same_v<T, std::string>, const T&, T> {
        const std::size_t idx = CtParamID<hash_name(Name)>::value;
        const auto& slot = slots_[idx];
        if (!slot.active) {
            throw std::out_of_range(std::format("ParameterRegistry: parameter not set: {}", std::string(Name.view())));
        }
        if (!std::holds_alternative<T>(slot.value)) {
            throw std::runtime_error(std::format("ParameterRegistry: type mismatch for {}", std::string(Name.view())));
        }
        return std::get<T>(slot.value);
    }

    /**
     * Returns true if a value has been stored for Name.
     */
    template <CTString Name>
    [[nodiscard]] auto has() const noexcept -> bool {
        return slots_[CtParamID<hash_name(Name)>::value].active;
    }

    /**
     * Returns the number of parameters currently set.
     */
    [[nodiscard]] auto size() const noexcept -> std::size_t { return active_indices_.size(); }

    // ── Report ────────────────────────────────────────────────────────────

    /** One row in the parameter report. */
    struct ParamRow {
        std::string name;
        std::string type;
        std::string value_str;
    };

    /** Returns all set parameters in insertion order. */
    [[nodiscard]] auto get_report() const -> std::vector<ParamRow> {
        std::vector<ParamRow> result;
        result.reserve(active_indices_.size());
        for (const std::size_t idx : active_indices_) {
            const auto& slot = slots_[idx];
            result.push_back({slot.name, type_name_(slot.value), format_value_(slot.value)});
        }
        return result;
    }

    /**
     * Prints all set parameters in a formatted table.
     *
     * Example output:
     *   Parameter          Type      Value
     *   ──────────────────────────────────────────
     *   learning_rate      double    0.001
     *   batch_size         int       32
     *   shuffle            bool      true
     *   optimizer          string    "adam"
     */
    void print_report() const { std::cout << *this; }

    explicit operator std::string() const {
        const auto rows = get_report();
        if (rows.empty()) {
            return {};
        }
        constexpr std::size_t MIN_NAME_WIDTH = 12;
        constexpr std::size_t MIN_TYPE_WIDTH = 8;
        std::size_t name_width = MIN_NAME_WIDTH;
        std::size_t type_width = MIN_TYPE_WIDTH;
        for (const auto& row : rows) {
            name_width = std::max(name_width, row.name.size() + 2);
            type_width = std::max(type_width, row.type.size() + 2);
        }
        std::string result;
        result += std::format("{:<{}}{:<{}}Value\n", "Parameter", name_width, "Type", type_width);
        result += box_line(name_width + type_width + 20) + '\n';
        for (const auto& row : rows) {
            result += std::format("{:<{}}{:<{}}{}\n", row.name, name_width, row.type, type_width, row.value_str);
        }
        return result;
    }

    friend auto operator<<(std::ostream& ostr, const ParameterRegistry& reg) -> std::ostream& { return ostr << static_cast<std::string>(reg); }

   private:
    struct Slot {
        Value value;
        std::string name;
        bool active = false;
    };

    std::array<Slot, MAX_CT_PARAMS> slots_;
    std::vector<std::size_t> active_indices_;  // insertion order for reporting

    // ── Formatting helpers ────────────────────────────────────────────────

    static auto box_line(std::size_t char_count) -> std::string {
        std::string line;
        line.reserve(char_count * 3);  // "─" (U+2500) is 3 UTF-8 bytes
        for (std::size_t i = 0; i < char_count; ++i) {
            line += "─";
        }
        return line;
    }

    static auto type_name_(const Value& val) -> std::string {
        return std::visit(
            [](const auto& v) -> std::string {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, int>) {
                    return "int";
                }
                if constexpr (std::is_same_v<T, double>) {
                    return "double";
                }
                if constexpr (std::is_same_v<T, bool>) {
                    return "bool";
                }
                return "string";
            },
            val);
    }

    static auto format_value_(const Value& val) -> std::string {
        return std::visit(
            [](const auto& v) -> std::string {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, bool>) {
                    return v ? "true" : "false";
                } else if constexpr (std::is_same_v<T, std::string>) {
                    return '"' + v + '"';
                } else {
                    return std::format("{}", v);
                }
            },
            val);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// CtParamID — maps a compile-time hash to a unique sequential parameter index.
//
// Instantiated once per unique CTString across the whole program.
// The static value is assigned on first use via ParameterRegistry::assign_param_id().
// ─────────────────────────────────────────────────────────────────────────────

template <std::size_t Hash>
const std::size_t CtParamID<Hash>::value = ParameterRegistry::assign_param_id(Hash);

// ─────────────────────────────────────────────────────────────────────────────
// Global convenience
// ─────────────────────────────────────────────────────────────────────────────

inline auto global_params() -> ParameterRegistry& {
    static ParameterRegistry inst;
    return inst;
}

#define PARAMS ::utilz::global_params()

}  // namespace utilz
