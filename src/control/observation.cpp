#include "control/observation.hpp"
#include "common/math_utils.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace kist {

// Encoder buffer offsets (observation_config.yaml order; see header)
static constexpr size_t kOffEncoderMode = 0;
static constexpr size_t kOffMotionQ     = 4;
static constexpr size_t kOffMotionDq    = 294;
static constexpr size_t kOffAnchorOri   = 601;

// Policy buffer offsets
static constexpr size_t kOffAngVel     = 64;
static constexpr size_t kOffHisQ       = 94;
static constexpr size_t kOffHisDq      = 384;
static constexpr size_t kOffHisAction  = 674;
static constexpr size_t kOffGravity    = 964;

static constexpr int kNumJoints   = MotionSequence50Hz::kNumJoints;
static constexpr int kHistFrames  = 10;   // *_10frame_step1
static constexpr int kFutFrames   = 10;   // *_10frame_step5
static constexpr int kFutStep     = 5;

// Future-frame sampling: hold current frame when not playing, otherwise
// advance by step and clamp to the last frame (hold final pose).
static int target_frame(int current_frame, int frame_idx, int step, int timesteps,
                        bool playing) {
    int f = current_frame;
    if (playing)
        f = std::min(f + frame_idx * step, timesteps - 1);
    return std::clamp(f, 0, timesteps - 1);
}

void ObservationAssembler::update_heading_state(const MotionSequence50Hz& motion,
                                                int current_frame,
                                                const StateLogger& logger) {
    if (motion.timesteps == 0)
        return;

    if (reinitialize_heading_) {
        init_base_quat_ = logger.GetLatest(1)[0].base_quat;
        delta_heading_  = 0.0;
        init_ref_root_rot_ = motion.frames[std::clamp(current_frame, 0, motion.timesteps - 1)].quaternion;
        reinitialize_heading_ = false;
        std::cout << "[Observation] heading state reset\n";
    }

    if (current_frame == 0)
        init_ref_root_rot_ = motion.frames[0].quaternion;
}

std::array<double, 4> ObservationAssembler::compute_apply_delta_heading() const {
    auto init_heading     = calc_heading_quat(init_base_quat_);
    auto data_heading_inv = calc_heading_quat_inv(init_ref_root_rot_);
    auto apply            = quat_mul(init_heading, data_heading_inv);
    if (delta_heading_ != 0.0)
        apply = quat_mul(euler_z_to_quat(delta_heading_), apply);
    return apply;
}

void ObservationAssembler::fill_encoder_obs(float* dst, const MotionSequence50Hz& motion,
                                            int current_frame, bool playing,
                                            const StateLogger& logger) const {
    std::memset(dst, 0, kEncoderDim * sizeof(float));
    if (motion.timesteps == 0)
        return;

    // encoder_mode_4: g1 mode id (0) then zeros — whole field stays 0
    dst[kOffEncoderMode] = 0.0f;

    // motion joint positions / velocities, 10 future frames, step 5
    for (int f = 0; f < kFutFrames; ++f) {
        int tf = target_frame(current_frame, f, kFutStep, motion.timesteps, playing);
        const auto& frame = motion.frames[tf];
        for (int j = 0; j < kNumJoints; ++j) {
            dst[kOffMotionQ  + f * kNumJoints + j] = static_cast<float>(frame.joints[j]);
            // gear_sonic zero-fills velocities while not playing
            dst[kOffMotionDq + f * kNumJoints + j] =
                playing ? static_cast<float>(frame.joint_velocities[j]) : 0.0f;
        }
    }

    // motion anchor orientation (mode 0: full robot base quat as left side)
    auto base_quat = logger.GetLatest(1)[0].base_quat;
    auto apply_delta_heading = compute_apply_delta_heading();

    for (int f = 0; f < kFutFrames; ++f) {
        int tf = target_frame(current_frame, f, kFutStep, motion.timesteps, playing);
        auto new_ref_root_rot = quat_mul(apply_delta_heading, motion.frames[tf].quaternion);
        auto base_to_ref      = quat_mul(quat_conjugate(base_quat), new_ref_root_rot);
        auto m                = quat_to_rotation_matrix(base_to_ref);

        // first two columns, flattened row-wise
        float* out = dst + kOffAnchorOri + f * 6;
        out[0] = static_cast<float>(m[0][0]); out[1] = static_cast<float>(m[0][1]);
        out[2] = static_cast<float>(m[1][0]); out[3] = static_cast<float>(m[1][1]);
        out[4] = static_cast<float>(m[2][0]); out[5] = static_cast<float>(m[2][1]);
    }
}

void ObservationAssembler::fill_policy_obs(float* dst, const StateLogger& logger) const {
    auto hist = logger.GetLatest(kHistFrames);  // oldest -> newest

    for (int f = 0; f < kHistFrames; ++f) {
        const auto& e = hist[f];

        for (int i = 0; i < 3; ++i)
            dst[kOffAngVel + f * 3 + i] = static_cast<float>(e.base_ang_vel[i]);

        for (int j = 0; j < kNumJoints; ++j) {
            dst[kOffHisQ      + f * kNumJoints + j] = static_cast<float>(e.body_q[j]);
            dst[kOffHisDq     + f * kNumJoints + j] = static_cast<float>(e.body_dq[j]);
            dst[kOffHisAction + f * kNumJoints + j] = static_cast<float>(e.last_action[j]);
        }

        auto g = gravity_in_body_frame(e.base_quat);
        for (int i = 0; i < 3; ++i)
            dst[kOffGravity + f * 3 + i] = static_cast<float>(g[i]);
    }
}

} // namespace kist
