#pragma once

#include "unitree/imu.hpp"
#include <array>
#include <cstdint>

namespace kist {

constexpr int kNumMotors = 29;  // Unitree G1

struct MotorState {
    double q{0.0};    // position (rad)
    double dq{0.0};   // velocity (rad/s)
    double tau{0.0};  // estimated torque (Nm)
};

struct UnitreeState {
    std::array<MotorState, kNumMotors> motors{};
    IMU      imu_pelvis{}; // pelvis IMU (from LowState.imu_state)
    uint32_t tick{0};
};

} // namespace kist
