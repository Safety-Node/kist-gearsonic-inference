#pragma once

namespace kist {

// Velocity command from an external navigation stack (e.g. the
// route-planner path follower), robot base frame.
//
// Producer contract (in-process wiring writes InputHandler::nav_buf):
//   - publish continuously (~20Hz) while following a path
//   - publish zeros when arrived / no path  -> "stop" command
//   - stop publishing when inactive/dead    -> buffer goes stale and the
//     arbitration falls back to manual (empty-buffer principle)
struct NavCommand {
    double vx{0.0};    // m/s, forward
    double vy{0.0};    // m/s, left
    double vyaw{0.0};  // rad/s, counter-clockwise
};

} // namespace kist
