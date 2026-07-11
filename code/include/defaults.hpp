#pragma once

#include <string>

namespace defaults {
// General parameters
constexpr bool debug_enabled = false;

// Execution parameters
constexpr int time_limit_s = 60;
constexpr int threads = 16;
constexpr bool stdout = true;
constexpr std::string log = "0";
constexpr std::string cpxlog = "cplex.log";
constexpr int seed = 2122187;
constexpr std::string mode = "hplus";
constexpr std::string hplus_alg = "base";

// Preprocessing
constexpr std::string preprocess = "lafbd";

// LMCut
constexpr std::string lmcut_pcf = "aiv";
constexpr std::string lmcut_min = "c";

// Primal Heuristic
constexpr std::string primal_heur = "gha";

// Cut separators
constexpr std::string cand_cuts = "lmcut-c";
constexpr std::string relax_cuts = "lmcut-g";
};  // namespace defaults
