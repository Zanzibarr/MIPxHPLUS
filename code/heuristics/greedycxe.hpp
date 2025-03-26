/**
 * @file greedycxe.hpp
 * @brief greedycxe heuristic
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 */

#ifndef GREEDYCXE_HPP
#define GREEDYCXE_HPP

#include "hplus_instance.hpp"
#include "log.hxx"
#include "utils.hpp"

namespace greedycxe {

/**
 * Find an heuristic solution using the greedycxe algorithm
 */
void run(hplus::instance& inst, hplus::environment& env, const logger& log);
}  // namespace greedycxe

#endif /* GREEDYCXE_HPP */