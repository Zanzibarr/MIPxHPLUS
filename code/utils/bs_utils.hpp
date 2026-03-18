#pragma once

#include <algorithm>

#include "bs.hxx"

static inline auto operator|=(binary_set &bs, const std::vector<unsigned int> &vec) -> binary_set & {
    for (const auto &val : vec) {
        bs.add(val);
    }
    return bs;
}

static inline auto bs_contains(const binary_set &bs, const std::vector<unsigned int> &vec) -> bool {
    return std::ranges::all_of(vec, [&bs](unsigned int val) { return bs[val]; });
}

static inline auto bs_intersects(const binary_set &bs, const std::vector<unsigned int> &vec) -> bool {
    return std::ranges::any_of(vec, [&bs](unsigned int val) { return bs[val]; });
}