#pragma once

#include <array>

namespace kist {

// Forward kinematics of the G1 wrist key frames in the pelvis frame,
// from measured joints (MuJoCo/DDS order q[29]; uses waist 12-14 and the
// arm 15-21 / 22-28). Chain constants come from g1_29dof_with_hand.urdf;
// the returned pose includes the [0.18, ∓0.025, 0] wrist_yaw_link local
// offset (gear_sonic key-frame convention) — directly comparable with
// the VR3Point wrist targets and usable as the calibration reference.
struct G1WristPose {
    std::array<double, 3> position;
    std::array<double, 4> quaternion;  // wxyz
};

G1WristPose g1_wrist_fk(const std::array<double, 29>& q, bool left);

} // namespace kist
