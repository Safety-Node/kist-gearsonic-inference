#pragma once

#include <array>

namespace kist {

// Joint index mappings between the two orderings used across the codebase
// (same values as gear_sonic policy_parameters.hpp):
//  - MuJoCo/URDF order: planner qpos, DDS motor commands/state.
//  - IsaacLab order: RL policy observations/actions, MotionSequence50Hz joints.

// IsaacLab-order joint i lives at MuJoCo index mujoco_to_isaaclab[i].
inline constexpr std::array<int, 29> mujoco_to_isaaclab = {
    0,  6,  12, 1,  7,  13, 2,  8,  14, 3,  9,  15, 22, 4, 10,
    16, 23, 5,  11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28};

// MuJoCo-order joint i lives at IsaacLab index isaaclab_to_mujoco[i].
inline constexpr std::array<int, 29> isaaclab_to_mujoco = {
    0,  3,  6,  9,  13, 17, 1,  4,  7,  10, 14, 18, 2,  5, 8,
    11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28};

} // namespace kist
