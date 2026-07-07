#pragma once

#include "common/data_buffer.hpp"
#include "common/robot_params.hpp"
#include "control/motor_command.hpp"
#include "control/obs_dict_model.hpp"
#include "control/observation.hpp"
#include "control/state_logger.hpp"
#include "planner/motion_sequence.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace kist {

// 50Hz control loop, ported from gear_sonic's Control() thread:
//   INIT (3s ramp to default pose) -> WAIT_FOR_CONTROL -> CONTROL
// Each CONTROL tick: robot state -> history logger -> encoder -> decoder ->
// MotorCommand buffer -> playback cursor advance + planner motion blending.
// It never touches DDS output itself — the (future) 500Hz command writer
// consumes motor_command_buf; until then running this is inherently dry.
class ControlLoop {
public:
    enum class State { INIT, WAIT_FOR_CONTROL, CONTROL };

    static ControlLoop& instance();
    bool start(const std::string& encoder_path, const std::string& decoder_path,
               bool auto_start_control = false);
    void stop();

    // Operator "start" (WAIT_FOR_CONTROL -> CONTROL transition)
    void start_control() { operator_start_ = true; }

    State state() const { return state_; }

    // ── outputs ─────────────────────────────────────────────────
    DataBuffer<MotorCommand> motor_command_buf;

    // Playback snapshot for the planner's context (blended motion + cursor).
    // Returns false until the first planner motion has been adopted.
    bool playback_snapshot(MotionSequence50Hz& motion, int& cursor) const;

    // ── timing (µs, last CONTROL tick) ──────────────────────────
    struct Timing { long obs{0}, encoder{0}, decoder{0}, total{0}; };
    Timing last_timing() const { std::lock_guard<std::mutex> l(timing_mutex_); return timing_; }

private:
    ControlLoop() = default;

    void loop();
    bool check_safety();
    bool gather_robot_state();
    void tick_init();
    void tick_control();
    void advance_playback();

    ObsDictModel encoder_;
    ObsDictModel decoder_;
    StateLogger  logger_;
    ObservationAssembler obs_;

    // Playback state (blended planner motion + cursor)
    mutable std::mutex playback_mutex_;
    MotionSequence50Hz playback_motion_;
    int                cursor_{0};
    bool               playing_{false};
    std::chrono::steady_clock::time_point last_plan_stamp_{};

    // INIT ramp
    std::array<double, 29> init_start_q_{};
    int  init_ticks_{0};
    bool init_captured_{false};

    std::array<double, 29> last_action_{};   // raw policy output (IsaacLab order)

    std::atomic<State> state_{State::INIT};
    std::atomic<bool>  operator_start_{false};
    bool               auto_start_{false};

    mutable std::mutex timing_mutex_;
    Timing             timing_;

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};

    static constexpr double kControlDt   = 0.02;   // 50Hz
    static constexpr int    kInitTicks   = 150;    // 3s ramp
    static constexpr int    kBlendFrames = 8;      // planner crossfade width
};

} // namespace kist
