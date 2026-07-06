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

    struct TriggerButton {
        bool prev{false};
        bool on_press{false};
        void update(double val) {
            bool cur = val > 0.5;
            on_press = cur && !prev;
            prev = cur;
        }
    };

    double facing_angle_{0.0};
    double movement_speed_{0.4};  // default SLOW_WALK speed
    int    mode_index_{1};         // default SLOW_WALK

    Button        btn_a_, btn_b_, btn_x_, btn_y_;
    TriggerButton left_trigger_, right_trigger_;

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
