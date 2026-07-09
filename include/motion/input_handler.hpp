#pragma once

#include "common/data_buffer.hpp"
#include "motion/movement_state.hpp"
#include <atomic>
#include <thread>

namespace kist {

class InputHandler {
public:
    // ── lifecycle ──────────────────────────────────────────────
    static InputHandler& instance();
    void                 start();
    void                 stop();

    // ── data buffer (read from any thread) ─────────────────────
    DataBuffer<MovementState> movement_buf;

    // Selected locomotion mode — what the stick will trigger. The
    // published MovementState stays IDLE while the stick is in the
    // deadzone, so displays should show this instead.
    int mode() const { return mode_index_; }

    // Emergency stop: latched once both grips are squeezed for 1s.
    // One-way — never clears until the process restarts. Deliberately an
    // atomic and not a DataBuffer: a stop request must not be clearable
    // or go stale.
    bool estop() const { return estop_; }

private:
    InputHandler() = default;

    void loop();

    struct Button {
        bool prev{false};
        bool on_press{false};
        void update(bool cur) { on_press = cur && !prev; prev = cur; }
    };

    double facing_angle_{0.0};
    double height_{-1.0};  // valid only in crouch modes; -1 = mode default
    // Arming a locomotion mode is a deliberate operator act: start in IDLE,
    // and drop back to IDLE whenever the VR link is lost (gear_sonic default).
    std::atomic<int> mode_index_{0};  // IDLE

    Button btn_a_, btn_x_, btn_y_;
    int    estop_hold_ticks_{0};
    std::atomic<bool> estop_{false};

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
