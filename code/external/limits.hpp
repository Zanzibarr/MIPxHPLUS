#ifndef LIM_HPP
#define LIM_HPP

#include <sys/resource.h>

#include <chrono>  // std::chrono
#include <thread>  // std::thread

#include "logger.hpp"

inline int GLOBAL_TERMINATE_CONDITION = 0;

#define CHECK_STOP() GLOBAL_TERMINATE_CONDITION

namespace timelim {

namespace {
bool stop_flag = false;
std::thread time_thread;
}  // namespace

inline void set_time_limit(unsigned int seconds) {
    stop_flag = false;
    GLOBAL_TERMINATE_CONDITION = 0;
    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::seconds(seconds);

    time_thread = std::thread([end]() {
        while (std::chrono::steady_clock::now() <= end) {
            if (stop_flag) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!stop_flag) {
            GLOBAL_TERMINATE_CONDITION = 1;
        }
    });
}

inline void cancel_time_limit() {
    stop_flag = true;
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
        LOG_ERROR << "setrlimit failed: ";
        return false;
    }

    return true;
}
}  // namespace memlim

#endif