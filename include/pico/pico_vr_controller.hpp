#pragma once

#include <array>

namespace kist {

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

} // namespace kist
