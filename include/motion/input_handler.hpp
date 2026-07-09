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

private:
    InputHandler() = default;

    void loop();

    struct Button {
        bool prev{false};
        bool on_press{false};
        void update(bool cur) { on_press = cur && !prev; prev = cur; }
    };

    double facing_angle_{0.0};
    // Arming a locomotion mode is a deliberate operator act: start in IDLE,
    // and drop back to IDLE whenever the VR link is lost (gear_sonic default).
    int    mode_index_{0};  // IDLE

    Button btn_a_, btn_b_, btn_x_, btn_y_;

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
