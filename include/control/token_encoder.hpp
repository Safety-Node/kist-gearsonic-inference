#pragma once

#include "control/obs_dict_model.hpp"
#include "control/state_logger.hpp"
#include "planner/motion_sequence_50hz.hpp"

#include <array>
#include <string>

namespace kist {

// Encoder stage: reference motion -> token.
//
// Owns the encoder TRT model, the observation assembly for it, and the
// heading-alignment state. Pure computation: inputs are data only — the
// orchestrator (WholeBodyController) decides when to run it, and
// operational signals (e-stop, safety) never enter this class.
//
// Encoder obs_dict [1762], g1 mode fills (observation_config.yaml offsets):
//   [0:4]     encoder_mode_4            (mode id, then zeros)
//   [4:294]   motion_joint_positions_10frame_step5   (IsaacLab order)
//   [294:584] motion_joint_velocities_10frame_step5
//   [601:661] motion_anchor_orientation_10frame_step5 (mode 0: full base quat)
// Remaining (teleop/smpl) slots stay zero.
class TokenEncoder {
public:
    static constexpr size_t kInputDim = 1762;
    static constexpr size_t kTokenDim = 64;
    using Token = std::array<float, kTokenDim>;

    bool init(const std::string& onnx_path);

    // One control tick: heading-state update, observation fill, inference.
    // Fails only on inference errors.
    bool step(const MotionSequence50Hz& motion, int cursor, bool playing,
              const StateLogger& logger, Token& token_out);

    // Re-anchor the heading alignment (called when a fresh playback
    // timeline starts, e.g. the first planner motion is adopted).
    void request_heading_reinit() { reinitialize_heading_ = true; }

private:
    void update_heading_state(const MotionSequence50Hz& motion, int cursor,
                              const StateLogger& logger);
    void fill_obs(float* dst, const MotionSequence50Hz& motion, int cursor,
                  bool playing, const StateLogger& logger) const;
    std::array<double, 4> compute_apply_delta_heading() const;

    ObsDictModel model_;

    // Heading alignment state (gear_sonic HeadingState + init ref root rot)
    std::array<double, 4> init_base_quat_{1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> init_ref_root_rot_{1.0, 0.0, 0.0, 0.0};
    double                delta_heading_{0.0};
    bool                  reinitialize_heading_{true};
};

} // namespace kist
