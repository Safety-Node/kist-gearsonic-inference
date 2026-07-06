#pragma once

#include "common/data_buffer.hpp"
#include <array>
#include <atomic>
#include <cstdint>

namespace kist {

struct BodyPose {
    // 24 SMPL joints, each [x, y, z, qx, qy, qz, qw]
    std::array<std::array<double, 7>, 24> joints{};
    int64_t timestamp_ns{0};
};

struct ControllerState {
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
    static PicoVRReader& instance();

    bool init();
    void deinit();

    DataBuffer<BodyPose>       body_buf;
    DataBuffer<ControllerState> ctrl_buf;

    // Internal: called from SDK callback
    void on_body_update(const BodyPose& pose);
    void on_controller_update(const ControllerState& ctrl);

private:
    PicoVRReader() = default;
};

} // namespace kist
