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

template <typename T>
static inline void insert_sorted(std::vector<T>& vec, T value) {
    auto it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end() || *it != value) {
        vec.insert(it, value);
    }
}

template <typename T>
[[nodiscard]]
static inline size_t sorted_find(const std::vector<T>& vec, T value) {
    auto it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it != vec.end() && *it == value) {
        return static_cast<size_t>(it - vec.begin());
    }
    return static_cast<size_t>(-1);  // Not found
}

#endif