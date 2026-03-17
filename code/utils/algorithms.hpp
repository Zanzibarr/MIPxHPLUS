/**
 * Utility functions used in various parts of this project
 *
 * @author Zanella Matteo (matteozanella2@gmail.com)
 */

#ifndef HPLUS_UTILS_ALGORITHMS_HPP
#define HPLUS_UTILS_ALGORITHMS_HPP

// ##################################################################### //
// ##################### VECTOR SORTING AND SEARCH ##################### //
// ##################################################################### //

#include <algorithm>
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
static inline size_t sorted_find(const std::vector<T>& vec, T value) {
    auto iter = std::lower_bound(vec.begin(), vec.end(), value);
    if (iter != vec.end() && *iter == value) {
        return static_cast<size_t>(iter - vec.begin());
    }
    return static_cast<size_t>(-1);  // Not found
}

#endif