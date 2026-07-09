#include "control/token_encoder.hpp"
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

static constexpr int kNumJoints  = MotionSequence50Hz::kNumJoints;
static constexpr int kFutFrames  = 10;   // *_10frame_step5
static constexpr int kFutStep    = 5;

// Future-frame sampling: hold current frame when not playing, otherwise
// advance by step and clamp to the last frame (hold final pose).
static int target_frame(int current_frame, int frame_idx, int step, int timesteps,
                        bool playing) {
    int f = current_frame;
    if (playing)
        f = std::min(f + frame_idx * step, timesteps - 1);
    return std::clamp(f, 0, timesteps - 1);
}

bool TokenEncoder::init(const std::string& onnx_path) {
    return model_.Initialize(onnx_path, "encoded_tokens", kInputDim, kTokenDim);
}

bool TokenEncoder::step(const MotionSequence50Hz& motion, int cursor, bool playing,
                        const StateLogger& logger, Token& token_out) {
    update_heading_state(motion, cursor, logger);
    fill_obs(model_.input(), motion, cursor, playing, logger);

    if (!model_.Infer())
        return false;

    std::memcpy(token_out.data(), model_.output(), kTokenDim * sizeof(float));
    return true;
}

void TokenEncoder::update_heading_state(const MotionSequence50Hz& motion,
                                        int cursor,
                                        const StateLogger& logger) {
    if (motion.timesteps == 0)
        return;

    if (reinitialize_heading_) {
        init_base_quat_ = logger.GetLatest(1)[0].base_quat;
        delta_heading_  = 0.0;
        init_ref_root_rot_ = motion.frames[std::clamp(cursor, 0, motion.timesteps - 1)].quaternion;
        reinitialize_heading_ = false;
        std::cout << "[TokenEncoder] heading state reset\n";
    }

    if (cursor == 0)
        init_ref_root_rot_ = motion.frames[0].quaternion;
}

std::array<double, 4> TokenEncoder::compute_apply_delta_heading() const {
    auto init_heading     = calc_heading_quat(init_base_quat_);
    auto data_heading_inv = calc_heading_quat_inv(init_ref_root_rot_);
    auto apply            = quat_mul(init_heading, data_heading_inv);
    if (delta_heading_ != 0.0)
        apply = quat_mul(euler_z_to_quat(delta_heading_), apply);
    return apply;
}

void TokenEncoder::fill_obs(float* dst, const MotionSequence50Hz& motion,
                            int cursor, bool playing,
                            const StateLogger& logger) const {
    std::memset(dst, 0, kInputDim * sizeof(float));
    if (motion.timesteps == 0)
        return;

    // encoder_mode_4: g1 mode id (0) then zeros — whole field stays 0
    dst[kOffEncoderMode] = 0.0f;

    // motion joint positions / velocities, 10 future frames, step 5
    for (int f = 0; f < kFutFrames; ++f) {
        int tf = target_frame(cursor, f, kFutStep, motion.timesteps, playing);
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
        int tf = target_frame(cursor, f, kFutStep, motion.timesteps, playing);
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

} // namespace kist
