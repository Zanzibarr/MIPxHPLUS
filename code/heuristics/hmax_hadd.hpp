/**
 * @file hmax_hadd.hpp
 * @brief hmax and hadd heuristics
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef HMAX_HADD_HPP
#define HMAX_HADD_HPP

#include "hplus_instance.hpp"
#include "utils.hpp"

namespace hmax {

/**
 * Find an heuristic solution using the hmax algorithm
 */
void run(hplus::instance& inst, hplus::environment& env, const logger& log);
}  // namespace hmax

namespace hadd {

/**
 * Find an heuristic solution using the hadd algorithm
 */
void run(hplus::instance& inst, hplus::environment& env, const logger& log);
}  // namespace hadd

#endif /* HMAX_HADD_HPP */