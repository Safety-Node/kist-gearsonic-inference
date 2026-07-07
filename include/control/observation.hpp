#pragma once

#include "control/state_logger.hpp"
#include "planner/motion_sequence.hpp"

#include <array>
#include <cstddef>

namespace kist {

// Observation assembly for the SONIC encoder/decoder, ported from the
// gear_sonic observation registry (g1 encoder mode only — teleop/smpl
// slots stay zero).
//
// Encoder obs_dict [1762], g1 mode fills:
//   [0:4]     encoder_mode_4            (mode id, then zeros)
//   [4:294]   motion_joint_positions_10frame_step5   (IsaacLab order)
//   [294:584] motion_joint_velocities_10frame_step5
//   [601:661] motion_anchor_orientation_10frame_step5 (mode 0: full base quat)
//
// Policy obs_dict [994]:
//   [0:64]    token_state (filled by caller from encoder output)
//   [64:94]   his_base_angular_velocity_10frame_step1
//   [94:384]  his_body_joint_positions_10frame_step1
//   [384:674] his_body_joint_velocities_10frame_step1
//   [674:964] his_last_actions_10frame_step1
//   [964:994] his_gravity_dir_10frame_step1
// History frames are oldest -> newest.
class ObservationAssembler {
public:
    static constexpr size_t kEncoderDim = 1762;
    static constexpr size_t kPolicyDim  = 994;
    static constexpr size_t kTokenDim   = 64;

    // Must be called once per control tick BEFORE fill_encoder_obs
    // (gear_sonic UpdateHeadingState). Reinitializes the heading alignment
    // when requested (first planner motion / operator reset) and pins the
    // reference root rotation whenever playback is at frame 0.
    void update_heading_state(const MotionSequence50Hz& motion, int current_frame,
                              const StateLogger& logger);

    void request_heading_reinit() { reinitialize_heading_ = true; }

    // Fills all 1762 floats (unused slots zeroed).
    void fill_encoder_obs(float* dst, const MotionSequence50Hz& motion,
                          int current_frame, bool playing,
                          const StateLogger& logger) const;

    // Fills [64:994]; the token slot [0:64] is left untouched for the caller.
    void fill_policy_obs(float* dst, const StateLogger& logger) const;

private:
    std::array<double, 4> compute_apply_delta_heading() const;

    // Heading alignment state (gear_sonic HeadingState + init ref root rot)
    std::array<double, 4> init_base_quat_{1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> init_ref_root_rot_{1.0, 0.0, 0.0, 0.0};
    double                delta_heading_{0.0};
    bool                  reinitialize_heading_{true};
};

} // namespace kist
