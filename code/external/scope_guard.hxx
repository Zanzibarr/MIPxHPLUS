#pragma once

/**
 * @file scope_guard.hxx
 * @brief RAII scope-guard that executes a callable on scope exit.
 * @version 1.0.0
 *
 * @details
 * Provides `ScopeGuard<F>` and the `ScopeStrategy` enum to control when
 * the guarded action runs:
 *   - `EXIT`    — always, regardless of exceptions (default)
 *   - `FAIL`    — only when the scope exits via an exception
 *   - `SUCCESS` — only when the scope exits normally
 *
 * Exception detection is implemented with `std::uncaught_exceptions()`:
 * on destruction the guard compares the current count against the count
 * captured at construction, so nested exception scenarios are handled
 * correctly.
 *
 * Three convenience factory functions wrap the most common cases:
 *   `on_scope_exit(f)`, `on_scope_fail(f)`, `on_scope_success(f)`.
 *
 * The guard can be dismissed early via `dismiss()`, which prevents
 * the callable from running.  Copy and move are both deleted.
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2026 Matteo Zanella
 *
 * Repository: https://github.com/Zanzibarr/cpp_utils
 *
 * SPDX-License-Identifier: MIT
 */

#include <cstdio>
#include <exception>
#include <utility>

namespace utilz {
/** Controls when the guarded callable is executed on scope exit. */
enum class ScopeStrategy {
    EXIT,    // Always run
    FAIL,    // Run only if an exception is thrown
    SUCCESS  // Run only if no exception is thrown
};

/**
 * RAII guard that invokes a callable when the enclosing scope exits.
 *
 * The execution condition is governed by `ScopeStrategy` and detected via
 * `std::uncaught_exceptions()` captured at construction time.
 *
 * @tparam F Callable type (function, lambda, or functor).  Must be movable.
 */
template <typename F>
class ScopeGuard {
   public:
    /**
     * @param func      Callable to invoke on scope exit.
     * @param strategy  When to invoke `func` (default: always).
     * ATTENTION: If the function throws, std:terminate is called
     */
    ScopeGuard(F&& func, ScopeStrategy strategy = ScopeStrategy::EXIT)
        : strategy_(strategy), func_(std::move(func)), initial_exceptions_(std::uncaught_exceptions()), active_(true) {}

    ~ScopeGuard() noexcept {
        if (!active_) {
            return;
        }

        int current_exceptions = std::uncaught_exceptions();
        bool failed = current_exceptions > initial_exceptions_;

        if (strategy_ == ScopeStrategy::EXIT || (strategy_ == ScopeStrategy::FAIL && failed) || (strategy_ == ScopeStrategy::SUCCESS && !failed)) {
            try {
                func_();
            } catch (...) {
                std::fputs("ScopeGuard: callable threw, terminating\n", stderr);
                std::terminate();
            }
        }
    }

    // Boilerplate: Disable copy and move
    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard(ScopeGuard&&) = delete;
    auto operator=(const ScopeGuard&) -> ScopeGuard& = delete;
    auto operator=(ScopeGuard&&) -> ScopeGuard& = delete;

    /** Cancels the guard; the callable will not run on destruction. */
    void dismiss() noexcept { active_ = false; }

   private:
    ScopeStrategy strategy_;
    F func_;
    int initial_exceptions_;
    bool active_{};
};

/** Returns a `ScopeGuard` that always calls `func` on scope exit. */
template <typename F>
[[nodiscard]] auto on_scope_exit(F&& func) {
    return ScopeGuard<std::decay_t<F>>(std::forward<F>(func), ScopeStrategy::EXIT);
}

/** Returns a `ScopeGuard` that calls `func` only if the scope exits normally. */
template <typename F>
[[nodiscard]] auto on_scope_success(F&& func) {
    return ScopeGuard<std::decay_t<F>>(std::forward<F>(func), ScopeStrategy::SUCCESS);
}

/** Returns a `ScopeGuard` that calls `func` only if the scope exits via an exception. */
template <typename F>
[[nodiscard]] auto on_scope_fail(F&& func) {
    return ScopeGuard<std::decay_t<F>>(std::forward<F>(func), ScopeStrategy::FAIL);
}

}  // namespace utilz
