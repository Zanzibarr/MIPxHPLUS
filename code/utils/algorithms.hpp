/**
 * Utility functions used in various parts of this project
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#pragma once

// ##################################################################### //
// ##################### VECTOR SORTING AND SEARCH ##################### //
// ##################################################################### //

#include <algorithm>
#include <unordered_set>
#include <vector>

template <typename T>
static inline void insert_sorted(std::vector<T>& vec, T value) {
    auto iter = std::lower_bound(vec.begin(), vec.end(), value);
    if (iter == vec.end() || *iter != value) {
        vec.insert(iter, value);
    }
}

template <typename T>
[[nodiscard]]
static inline auto sorted_find(const std::vector<T>& vec, T value) -> unsigned int {
    auto iter = std::lower_bound(vec.begin(), vec.end(), value);
    if (iter != vec.end() && *iter == value) {
        return static_cast<unsigned int>(iter - vec.begin());
    }
    return static_cast<unsigned int>(-1);  // Not found
}

template <typename T>
[[nodiscard]]
static inline auto sorted_contains(const std::vector<T>& vec, T value) -> bool {
    return sorted_find(vec, value) < vec.size();
}

template <typename T>
[[nodiscard]]
static inline auto set_contains(const std::unordered_set<T>& set, const std::vector<T>& vec) -> bool {
    return std::ranges::all_of(vec, [&set](unsigned int val) { return set.contains(val); });
}