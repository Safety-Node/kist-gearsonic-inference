#pragma once

#include <array>

namespace kist {

struct IMU {
    std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};  // (w, x, y, z)
    std::array<double, 3> gyroscope{0.0, 0.0, 0.0};        // rad/s
    std::array<double, 3> accelerometer{0.0, 0.0, 0.0};    // m/s^2
};

} // namespace kist
