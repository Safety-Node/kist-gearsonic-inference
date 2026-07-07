#pragma once

#include <algorithm>
#include <array>
#include <vector>

namespace kist {

constexpr int kMaxPlannerFrames = 64;   // planner outputs up to 64 frames @ 30Hz
constexpr int kQposDim          = 36;   // 3 pos + 4 quat (wxyz) + 29 joints

// 50Hz motion sequence, matches gear_sonic's planner_motion_50hz_ layout.
// Each frame: 3 position + 4 quaternion (wxyz) + 29 joint angles/velocities.
// Joints are stored in IsaacLab order (matches gear_sonic MotionSequence).
struct MotionSequence50Hz {
    static constexpr int kNumJoints = 29;
    static constexpr int kCapacity  = 1500;   // 30 s at 50Hz (matches gear_sonic)

    struct Frame {
        std::array<double, 3>          position{};
        std::array<double, 4>          quaternion{1.0, 0.0, 0.0, 0.0};
        std::array<double, kNumJoints> joints{};
        std::array<double, kNumJoints> joint_velocities{};
    };

    // frames.size() == timesteps, so publishing a copy only copies real frames
    std::vector<Frame> frames;
    int                timesteps{0};

    // 50Hz frame on the consumer's playback timeline (look-ahead included)
    // this sequence was generated at; blend it in starting at this frame.
    int gen_frame{0};

    void resize(int n) {
        timesteps = std::min(n, kCapacity);
        frames.resize(timesteps);
    }
};

} // namespace kist
