#pragma once

#include <array>

namespace kist {

enum class LocomotionMode {
    IDLE                  = 0,
    SLOW_WALK             = 1,
    WALK                  = 2,
    RUN                   = 3,
    IDEL_SQUAT            = 4,
    IDEL_KNEEL_TWO_LEGS   = 5,
    IDEL_KNEEL            = 6,
    IDEL_LYING_FACE_DOWN  = 7,
    CRAWLING              = 8,
    IDEL_BOXING           = 9,
    WALK_BOXING           = 10,
    LEFT_PUNCH            = 11,
    RIGHT_PUNCH           = 12,
    RANDOM_PUNCH          = 13,
    ELBOW_CRAWLING        = 14,
    LEFT_HOOK             = 15,
    RIGHT_HOOK            = 16,
    FORWARD_JUMP          = 17,
    STEALTH_WALK          = 18,
    INJURED_WALK          = 19,
    LEDGE_WALKING         = 20,
    OBJECT_CARRYING       = 21,
    STEALTH_WALK_2        = 22,
    HAPPY_DANCE_WALK      = 23,
    ZOMBIE_WALK           = 24,
    GUN_WALK              = 25,
    SCARE_WALK            = 26,
};

// Static modes hold a pose in place; the planner never replans them on a
// timer, only on explicit command changes (matches gear_sonic).
inline constexpr bool is_static_motion_mode(LocomotionMode mode) {
    return mode == LocomotionMode::IDLE ||
           mode == LocomotionMode::IDEL_SQUAT ||
           mode == LocomotionMode::IDEL_KNEEL_TWO_LEGS ||
           mode == LocomotionMode::IDEL_KNEEL ||
           mode == LocomotionMode::IDEL_LYING_FACE_DOWN ||
           mode == LocomotionMode::IDEL_BOXING;
}

struct MovementState {
    int                   locomotion_mode{0};
    std::array<double, 3> movement_direction{0.0, 0.0, 0.0};
    std::array<double, 3> facing_direction{1.0, 0.0, 0.0};
    double                movement_speed{-1.0};
    double                height{-1.0};

    MovementState() = default;
    MovementState(int mode,
                  const std::array<double, 3>& movement,
                  const std::array<double, 3>& facing,
                  double speed,
                  double height)
        : locomotion_mode(mode), movement_direction(movement),
          facing_direction(facing), movement_speed(speed), height(height) {}
};

} // namespace kist
