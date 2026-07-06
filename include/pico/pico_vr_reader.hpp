#pragma once

#include "common/data_buffer.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <thread>

namespace kist {

struct PicoVRBodyPose {
    // 24 SMPL joints, each [x, y, z, qx, qy, qz, qw]
    std::array<std::array<double, 7>, 24> joints{};
    int64_t timestamp_ns{0};
};

struct PicoVRController {
    std::array<double, 2> left_axis{0.0, 0.0};   // joystick [x, y]
    std::array<double, 2> right_axis{0.0, 0.0};  // joystick [x, y]
    bool btn_a{false};   // right primary
    bool btn_b{false};   // right secondary
    bool btn_x{false};   // left primary
    bool btn_y{false};   // left secondary
    double left_trigger{0.0};
    double right_trigger{0.0};
    double left_grip{0.0};
    double right_grip{0.0};
};

class PicoVRReader {
public:
    // ── lifecycle ──────────────────────────────────────────────
    static PicoVRReader& instance();
    void                 start();
    void                 stop();

    // ── state ──────────────────────────────────────────────────
    std::atomic<bool> connected{false};

    // ── data buffers (read from any thread) ────────────────────
    DataBuffer<PicoVRBodyPose>        body_buf;
    DataBuffer<PicoVRController> ctrl_buf;

    // ── internal: called from SDK callback ─────────────────────
    void on_body_update(const PicoVRBodyPose& pose);
    void on_controller_update(const PicoVRController& ctrl);

private:
    PicoVRReader() = default;

    void watchdog_loop();

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};
};

} // namespace kist
