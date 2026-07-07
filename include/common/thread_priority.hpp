#pragma once

#include <pthread.h>
#include <iostream>
#include <thread>

namespace kist {

// Best-effort SCHED_FIFO real-time priority for a control/publish thread.
// Requires CAP_SYS_NICE (run the container with --cap-add=SYS_NICE);
// logs a hint and continues with normal scheduling otherwise.
inline void try_realtime_priority(std::thread& t, int priority, const char* name) {
    sched_param sp{};
    sp.sched_priority = priority;
    if (pthread_setschedparam(t.native_handle(), SCHED_FIFO, &sp) != 0) {
        std::cerr << "[" << name << "] SCHED_FIFO unavailable — running with normal "
                     "scheduling (add --cap-add=SYS_NICE to docker run for RT priority)\n";
    } else {
        std::cout << "[" << name << "] SCHED_FIFO priority " << priority << " set\n";
    }
}

} // namespace kist
