#include "control/policy_decoder.hpp"
#include "common/joint_order.hpp"
#include "common/math_utils.hpp"
#include "common/robot_params.hpp"

#include <cstring>

namespace kist {

// Policy buffer offsets (observation_config.yaml order; see header)
static constexpr size_t kOffAngVel    = 64;
static constexpr size_t kOffHisQ      = 94;
static constexpr size_t kOffHisDq     = 384;
static constexpr size_t kOffHisAction = 674;
static constexpr size_t kOffGravity   = 964;

static constexpr int kNumJoints  = 29;
static constexpr int kHistFrames = 10;   // *_10frame_step1

bool PolicyDecoder::init(const std::string& onnx_path) {
    return model_.Initialize(onnx_path, "action", kInputDim, kNumJoints);
}

bool PolicyDecoder::step(const TokenEncoder::Token& token, const StateLogger& logger,
                         MotorCommand& cmd_out) {
    std::memcpy(model_.input(), token.data(), token.size() * sizeof(float));
    fill_obs(model_.input(), logger);

    if (!model_.Infer())
        return false;

    // action -> motor command (q = default + scale * remap(action))
    const float* action = model_.output();
    for (int i = 0; i < kNumJoints; ++i) {
        double a = action[isaaclab_to_mujoco[i]] * g1_action_scale[i];
        cmd_out.q_target[i] = static_cast<float>(g1_default_angles[i] + a);
        cmd_out.kp[i]       = g1_kps[i];
        cmd_out.kd[i]       = g1_kds[i];
    }
    for (int i = 0; i < kNumJoints; ++i)
        last_action_[i] = action[i];
    return true;
}

void PolicyDecoder::fill_obs(float* dst, const StateLogger& logger) const {
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
