#pragma once

#include <algorithm>
#include <string>
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

[[nodiscard]]
inline auto split_string(const std::string& str, const char del) -> std::vector<std::string> {
    std::vector<std::string> tokens;
    tokens.reserve(static_cast<size_t>(std::count(str.begin(), str.end(), del)) + 1);

    size_t start{0};
    size_t end{0};

    while ((end = str.find(del, start)) != std::string::npos) {
        if (end > start) {  // Avoid empty strings
            tokens.push_back(str.substr(start, end - start));
        }
        start = end + 1;
    }

    // Add the last token if it exists
    if (start < str.length()) {
        tokens.push_back(str.substr(start));
    }

    return tokens;
}

template <typename T>
[[nodiscard]]
static inline auto vtos(std::vector<T> v, int size = -1) -> std::string {
    if (size == -1) {
        size = static_cast<int>(v.size());
    }
    std::string s;
    if (v.size() <= static_cast<unsigned int>(size)) {
        for (const auto& x : v) {
            s.append(std::to_string(x)).append(";");
        }
    } else {
        for (unsigned int i = 0; i < static_cast<unsigned int>(size) / 2; i++) {
            s.append(std::to_string(v[i])).append(";");
        }
        s.append("...[").append(std::to_string(v.size() - static_cast<unsigned int>(size))).append("];");
        for (unsigned int i = static_cast<unsigned int>(size) / 2; i > 0; i--) {
            s.append(std::to_string(v[v.size() - i])).append(";");
        }
    }
    return s;
}
