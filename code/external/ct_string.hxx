#pragma once

/**
 * @file ct_string.hxx
 * @brief Compile-time string literal usable as a non-type template parameter (C++20).
 * @version 1.0.0
 *
 * @details
 * `CTString<N>` wraps a string literal so it can appear as a template
 * argument.  The compiler deduplicates instantiations for the same literal
 * via the `consteval operator==`.
 *
 * `hash_name(string_view)` computes a 64-bit FNV-1a hash at compile time.
 * It is used by `CtParamID`, `CtSlotID`, and `CtStatID` in
 * `parameters.hxx`, `argparser.hxx`, `timer.hxx`, and `stats_registry.hxx` to map a
 * `CTString` to a unique sequential array index without any runtime hash
 * table.
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * Repository: https://github.com/Zanzibarr/cpp_utils
 *
 * SPDX-License-Identifier: MIT
 */

#include <algorithm>
#include <cstddef>
#include <string_view>

namespace utilz {
/**
 * String literal type usable as a non-type template parameter (C++20).
 *
 * @tparam N Length of the string including the null terminator.
 */
template <std::size_t N>
struct CTString {
    char data[N]{};

    /** Constructs from a string literal; `N` is deduced by the compiler. */
    consteval CTString(const char (&str)[N]) noexcept { std::copy_n(str, N, data); }

    /** Returns a `string_view` over the stored characters (excluding null terminator). */
    [[nodiscard]] consteval auto view() const noexcept -> std::string_view {
        return {data, N - 1};  // exclude null terminator
    }

    /** Implicit conversion to `string_view`. */
    consteval operator std::string_view() const noexcept { return view(); }

    /**
     * Equality comparison — used by the compiler to deduplicate template
     * instantiations for the same literal.
     */
    template <std::size_t M>
    consteval auto operator==(const CTString<M>& other) const noexcept -> bool {
        if constexpr (N != M) {
            return false;
        }
        for (std::size_t i = 0; i < N; ++i) {
            if (data[i] != other.data[i]) {
                return false;
            }
        }
        return true;
    }
};

/**
 * Computes a 64-bit FNV-1a hash of `s` at compile time.
 *
 * Used to produce a unique compile-time integer from a `CTString` so it can
 * drive template specialisations and slot-array indexing in `parameters.hxx`,
 * `timer.hxx`, and `stats_registry.hxx`.  Collision probability over 64 bits
 * is negligible for any realistic number of names.
 */
consteval auto hash_name(std::string_view str) noexcept -> std::size_t {
    std::size_t hash = 14695981039346656037ULL;
    for (char chr : str) {
        hash ^= static_cast<std::size_t>(static_cast<unsigned char>(chr));
        hash *= 1099511628211ULL;
    }
    return hash;
}

}  // namespace utilz
