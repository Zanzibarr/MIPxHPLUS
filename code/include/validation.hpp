#pragma once

#include <string>
#include <vector>

#include "utils.hpp"

inline void validate(const std::string& file_path, const std::vector<std::string>& solution, Logger& logger) {
    logger[WARNING] << "Verifying the solution";

    // TODO

    logger[FATAL] << "Infeasible solution";
}