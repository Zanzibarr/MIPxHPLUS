/**
 * @file limits.hxx
 * @brief Utilities about setting memory/time limits
 * @version 1.0.2
 *
 * @author Matteo Zanella <matteozanella2@gmail.com>
 * Copyright 2025 Matteo Zanella
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LIM_HXX
#define LIM_HXX

#include <sys/resource.h>

#include <chrono>              // std::chrono
#include <condition_variable>  // std::condition_variable
#include <iostream>            // std::cerr
#include <mutex>               // std::mutex
#include <thread>              // std::thread

inline int GLOBAL_TERMINATE_CONDITION = 0;

#define CHECK_STOP() GLOBAL_TERMINATE_CONDITION

namespace timelim {

inline bool stop_flag = false;
inline std::thread time_thread;
inline std::mutex cv_mutex;
inline std::condition_variable stop_cv;

inline void set_time_limit(unsigned int seconds) {
    {
        std::lock_guard<std::mutex> lock(cv_mutex);
        stop_flag = false;
        GLOBAL_TERMINATE_CONDITION = 0;
    }
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);

    time_thread = std::thread([end]() {
        std::unique_lock<std::mutex> lock(cv_mutex);
        stop_cv.wait_until(lock, end, [] { return stop_flag; });
        if (!stop_flag) {
            GLOBAL_TERMINATE_CONDITION = 1;
        }
    });
}

inline void cancel_time_limit() {
    {
        std::lock_guard<std::mutex> lock(cv_mutex);
        stop_flag = true;
    }
    stop_cv.notify_all();
    if (time_thread.joinable()) {
        time_thread.join();
    }
}

}  // namespace timelim

namespace memlim {

[[nodiscard]]
inline bool set_memory_limit(int limit_mb) {
    struct rlimit rl;
    rlim_t limit_bytes = (rlim_t)limit_mb * 1024 * 1024;

    // Set new limits
    rl.rlim_cur = limit_bytes;
    rl.rlim_max = limit_bytes;

    if (setrlimit(RLIMIT_AS, &rl) != 0) {
        std::cerr << "[WARNING]: setrlimit failed: ";
        return false;
    }

    return true;
}
}  // namespace memlim

#endif