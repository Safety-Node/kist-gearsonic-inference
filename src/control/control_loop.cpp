#include "control/control_loop.hpp"
#include "common/joint_order.hpp"
#include "common/math_utils.hpp"
#include "planner/planner_inference.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace kist {

static constexpr double kMaxJointVel      = 35.0;   // rad/s, safety abort
static constexpr double kStateStaleMs     = 500.0;  // LowState absence threshold

ControlLoop& ControlLoop::instance() {
    static ControlLoop inst;
    return inst;
}

bool ControlLoop::start(const std::string& encoder_path, const std::string& decoder_path,
                        bool auto_start_control) {
    auto_start_ = auto_start_control;

    if (!encoder_.Initialize(encoder_path, "encoded_tokens",
                             ObservationAssembler::kEncoderDim,
                             ObservationAssembler::kTokenDim))
        return false;
    if (!decoder_.Initialize(decoder_path, "action",
                             ObservationAssembler::kPolicyDim, 29))
        return false;

    stop_        = false;
    loop_thread_ = std::thread(&ControlLoop::loop, this);
    std::cout << "[ControlLoop] started (50 Hz)\n";
    return true;
}

void ControlLoop::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
}

bool ControlLoop::playback_snapshot(MotionSequence50Hz& motion, int& cursor) const {
    std::lock_guard<std::mutex> lock(playback_mutex_);
    if (playback_motion_.timesteps == 0)
        return false;
    motion = playback_motion_;
    cursor = cursor_;
    return true;
}

// ─── safety / robot state ─────────────────────────────────────────────────────

bool ControlLoop::check_safety() {
    auto st = UnitreeStateReader::instance().unitree_state_buf.GetDataWithTime();
    if (!st.HasData() || st.GetAgeMs() > kStateStaleMs) {
        std::cerr << "[ControlLoop] LowState missing/stale — stopping\n";
        return false;
    }
    return true;
}

bool ControlLoop::gather_robot_state() {
    auto st = UnitreeStateReader::instance().unitree_state_buf.GetData();
    if (!st)
        return false;

    StateLogger::Entry e;
    e.base_quat    = {st->imu_pelvis.quaternion[0], st->imu_pelvis.quaternion[1],
                      st->imu_pelvis.quaternion[2], st->imu_pelvis.quaternion[3]};
    e.base_ang_vel = {st->imu_pelvis.gyroscope[0], st->imu_pelvis.gyroscope[1],
                      st->imu_pelvis.gyroscope[2]};
    for (int i = 0; i < 29; ++i) {
        int m = mujoco_to_isaaclab[i];
        e.body_q[i]  = st->motors[m].q - g1_default_angles[m];
        e.body_dq[i] = st->motors[m].dq;
        if (std::abs(e.body_dq[i]) > kMaxJointVel) {
            std::cerr << "[ControlLoop] joint velocity " << e.body_dq[i]
                      << " rad/s over limit — stopping\n";
            return false;
        }
    }
    e.last_action = last_action_;
    logger_.Log(e);
    return true;
}

// ─── INIT: 3s linear ramp to the default standing pose ────────────────────────

void ControlLoop::tick_init() {
    auto st = UnitreeStateReader::instance().unitree_state_buf.GetData();
    if (!st)
        return;

    if (!init_captured_) {
        for (int i = 0; i < 29; ++i)
            init_start_q_[i] = st->motors[i].q;
        init_captured_ = true;
    }

    double ratio = std::clamp(double(init_ticks_) / kInitTicks, 0.0, 1.0);
    MotorCommand cmd;
    for (int i = 0; i < 29; ++i) {
        cmd.q_target[i] = static_cast<float>(init_start_q_[i] * (1.0 - ratio) +
                                             g1_default_angles[i] * ratio);
        cmd.kp[i] = g1_kps[i];
        cmd.kd[i] = g1_kds[i];
    }
    motor_command_buf.SetData(cmd);

    if (++init_ticks_ >= kInitTicks) {
        state_ = State::WAIT_FOR_CONTROL;
        std::cout << "[ControlLoop] INIT ramp done -> WAIT_FOR_CONTROL\n";
    }
}

// ─── playback advance + planner motion blending ───────────────────────────────
// gear_sonic CurrentFrameAdvancement: adopt/blend new planner sequences, then
// advance the cursor by one frame per tick, clamping at the end.

void ControlLoop::advance_playback() {
    std::lock_guard<std::mutex> lock(playback_mutex_);

    auto plan = PlannerInference::instance().motion_50hz_buf.GetDataWithTime();
    if (plan.HasData() && plan.timestamp != last_plan_stamp_) {
        last_plan_stamp_ = plan.timestamp;
        const auto& gen = *plan.data;

        if (playback_motion_.timesteps == 0) {
            // first planner motion: copy wholesale, restart playback
            playback_motion_ = gen;
            cursor_  = 0;
            playing_ = true;
            obs_.request_heading_reinit();
            std::cout << "[ControlLoop] first planner motion adopted ("
                      << gen.timesteps << " frames)\n";
        } else {
            int fgen = gen.gen_frame;
            int new_len = fgen - cursor_ + gen.timesteps;
            if (new_len > 0) {
                new_len = std::min(new_len, MotionSequence50Hz::kCapacity);
                int blend_start = std::max(0, fgen - cursor_);

                MotionSequence50Hz out;
                out.resize(new_len);
                for (int f = 0; f < new_len; ++f) {
                    int f_old = std::clamp(f + cursor_, 0, playback_motion_.timesteps - 1);
                    int f_new = std::clamp(f + cursor_ - fgen, 0, gen.timesteps - 1);
                    double w_new = std::clamp(double(f - blend_start) / kBlendFrames, 0.0, 1.0);
                    double w_old = 1.0 - w_new;

                    const auto& a = playback_motion_.frames[f_old];
                    const auto& b = gen.frames[f_new];
                    auto& o = out.frames[f];
                    for (int j = 0; j < 29; ++j) {
                        o.joints[j]           = w_old * a.joints[j] + w_new * b.joints[j];
                        o.joint_velocities[j] = w_old * a.joint_velocities[j] + w_new * b.joint_velocities[j];
                    }
                    for (int i = 0; i < 3; ++i)
                        o.position[i] = w_old * a.position[i] + w_new * b.position[i];
                    o.quaternion = quat_slerp(a.quaternion, b.quaternion, w_new);
                }
                playback_motion_ = std::move(out);
                cursor_ = 0;
            }
        }
    }

    // advance one frame per tick, hold last frame at the end
    if (playing_ && playback_motion_.timesteps > 0)
        cursor_ = std::min(cursor_ + 1, playback_motion_.timesteps - 1);
}

// ─── CONTROL tick ─────────────────────────────────────────────────────────────

void ControlLoop::tick_control() {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    if (!gather_robot_state()) {
        state_ = State::WAIT_FOR_CONTROL;
        return;
    }

    MotionSequence50Hz* motion = nullptr;
    int cursor = 0;
    {
        std::lock_guard<std::mutex> lock(playback_mutex_);
        motion = &playback_motion_;
        cursor = cursor_;
    }
    if (motion->timesteps == 0) {
        // no planner motion yet — nothing to track
        advance_playback();
        return;
    }

    // observations (heading state must update first)
    obs_.update_heading_state(*motion, cursor, logger_);
    obs_.fill_encoder_obs(encoder_.input(), *motion, cursor, playing_, logger_);
    auto t1 = clock::now();

    if (!encoder_.Infer())
        return;
    auto t2 = clock::now();

    std::memcpy(decoder_.input(), encoder_.output(),
                ObservationAssembler::kTokenDim * sizeof(float));
    obs_.fill_policy_obs(decoder_.input(), logger_);
    if (!decoder_.Infer())
        return;
    auto t3 = clock::now();

    // action -> motor command (q = default + scale * remap(action))
    const float* action = decoder_.output();
    MotorCommand cmd;
    for (int i = 0; i < 29; ++i) {
        double a = action[isaaclab_to_mujoco[i]] * g1_action_scale[i];
        cmd.q_target[i] = static_cast<float>(g1_default_angles[i] + a);
        cmd.kp[i]       = g1_kps[i];
        cmd.kd[i]       = g1_kds[i];
    }
    motor_command_buf.SetData(cmd);
    for (int i = 0; i < 29; ++i)
        last_action_[i] = action[i];

    advance_playback();

    {
        std::lock_guard<std::mutex> l(timing_mutex_);
        timing_.obs     = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        timing_.encoder = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        timing_.decoder = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
        timing_.total   = std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0).count();
    }
}

// ─── loop ─────────────────────────────────────────────────────────────────────

void ControlLoop::loop() try {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(static_cast<int>(kControlDt * 1e6));

    while (!stop_) {
        auto t0 = clock::now();

        if (!check_safety()) {
            std::this_thread::sleep_until(t0 + period);
            continue;
        }

        switch (state_) {
            case State::INIT:
                tick_init();
                break;
            case State::WAIT_FOR_CONTROL:
                if (operator_start_ || auto_start_) {
                    state_ = State::CONTROL;
                    std::cout << "[ControlLoop] -> CONTROL\n";
                }
                break;
            case State::CONTROL:
                tick_control();
                break;
        }

        std::this_thread::sleep_until(t0 + period);
    }
} catch (const std::exception& e) {
    std::cerr << "[ControlLoop] loop exception: " << e.what() << "\n";
}

} // namespace kist
