#pragma once

/**
 * @file ansi_colors.hxx
 * @brief Shared ANSI escape-code constants and TTY-aware colour wrappers.
 * @version 1.0.0
 *
 * @details
 * Provides two layers in namespace `ansi`:
 *   - `codes` — a struct of `constexpr const char*` ANSI escape sequences
 *     (reset, foreground colours, bright variants).  Use these directly when
 *     you manage the reset/colour lifecycle yourself (e.g. `Logger`).
 *   - Wrapper functions `green()`, `red()`, `yellow()`, `cyan()`, `bold()`,
 *     `dim()` — wrap a `string_view` in the appropriate escape codes and
 *     return `std::string`.  They are no-ops (pass-through) when `enabled()`
 *     returns false.
 *
 * `enabled()` calls `isatty(fileno(stdout))` once and caches the result.
 * On Windows it always returns `false` (ANSI not universally supported).
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * Repository: https://github.com/Zanzibarr/cpp_utils
 *
 * SPDX-License-Identifier: MIT
 */

#include <string>
#include <string_view>

#ifndef _WIN32
#include <unistd.h>

#endif

namespace utilz {

namespace ansi {

// ─────────────────────────────────────────────────────────────────────────────
// Raw escape-code constants — use these when you manage the reset/color
// yourself (e.g. in Logger::write_record).
// ─────────────────────────────────────────────────────────────────────────────

struct codes {
    static constexpr const char* reset = "\033[0m";
    static constexpr const char* red = "\033[31m";
    static constexpr const char* green = "\033[32m";
    static constexpr const char* yellow = "\033[33m";
    static constexpr const char* blue = "\033[34m";
    static constexpr const char* magenta = "\033[35m";
    static constexpr const char* cyan = "\033[36m";
    static constexpr const char* white = "\033[37m";
    static constexpr const char* bright_red = "\033[91m";
    static constexpr const char* bright_green = "\033[92m";
    static constexpr const char* bright_yellow = "\033[93m";
    static constexpr const char* bright_blue = "\033[94m";
    static constexpr const char* bright_magenta = "\033[95m";
    static constexpr const char* bright_cyan = "\033[96m";
    static constexpr const char* bright_white = "\033[97m";
};

// ─────────────────────────────────────────────────────────────────────────────
// Terminal detection — true when stdout is a real tty (not a pipe/file).
// Result is cached after the first call.
// ─────────────────────────────────────────────────────────────────────────────

inline auto enabled() noexcept -> bool {
#ifdef _WIN32
    return false;
#else
    static const bool val = (isatty(fileno(stdout)) != 0);
    return val;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Wrapper helpers — wrap str in color codes when enabled(), else return as-is.
// ─────────────────────────────────────────────────────────────────────────────

inline auto green(std::string_view str) -> std::string { return enabled() ? "\033[32m" + std::string(str) + "\033[0m" : std::string(str); }
inline auto red(std::string_view str) -> std::string { return enabled() ? "\033[31m" + std::string(str) + "\033[0m" : std::string(str); }
inline auto yellow(std::string_view str) -> std::string { return enabled() ? "\033[33m" + std::string(str) + "\033[0m" : std::string(str); }
inline auto cyan(std::string_view str) -> std::string { return enabled() ? "\033[36m" + std::string(str) + "\033[0m" : std::string(str); }
inline auto bold(std::string_view str) -> std::string { return enabled() ? "\033[1m" + std::string(str) + "\033[0m" : std::string(str); }
inline auto dim(std::string_view str) -> std::string { return enabled() ? "\033[2m" + std::string(str) + "\033[0m" : std::string(str); }

}  // namespace ansi

}  // namespace utilz
