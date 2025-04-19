/**
 * @file greedycost.hpp
 * @brief greedycost heuristic
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef GREEDYCOST_HPP
#define GREEDYCOST_HPP

#include "hplus_instance.hpp"
#include "utils.hpp"

namespace greedycost {

/**
 * Find an heuristic solution using the greedycost algorithm
 */
void run(hplus::instance& inst, hplus::environment& env, const logger& log);
}  // namespace greedycost

#endif /* GREEDYCOST_HPP */