#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>

namespace kist {

// Mode navigation walks the LocomotionMode values linearly (gear_sonic
// remote scheme, clamped at both ends). Going up past WALK — RUN and the
// advanced modes (squat, crawl, boxing, ...) — requires holding a trigger;
// going down (toward IDLE — always the safer direction) is never gated.
static constexpr int    kBasicModeMax = static_cast<int>(LocomotionMode::WALK);         // 2
static constexpr int    kFullModeMax  = static_cast<int>(LocomotionMode::INJURED_WALK); // 19 (gear_sonic cap)
static constexpr double kTriggerHeld  = 0.5;
// Body height is meaningful only in the crouch modes (squat/kneel, 4..6);
// standing modes force -1 (mode default), matching the gear_sonic gamepad.
// Adjusted by trigger+B (up) / trigger+A (down) while crouched.
static constexpr int    kCrouchFirst  = static_cast<int>(LocomotionMode::IDEL_SQUAT); // 4
static constexpr int    kCrouchLast   = static_cast<int>(LocomotionMode::IDEL_KNEEL); // 6
static constexpr double kHeightSeed   = 0.8;   // on entering a crouch mode (gear_sonic)
static constexpr double kHeightMin    = 0.1;   // gear_sonic clamp
static constexpr double kHeightMax    = 0.8;
static constexpr double kHeightRate   = 0.3;   // m/s while held
// E-stop gesture: both grips squeezed hard for a full second. High
// threshold + both hands + hold time make accidental triggering unlikely
// while matching the panic reflex of clenching the controllers.
static constexpr double kEstopGrip      = 0.8;
static constexpr int    kEstopHoldTicks = 500;  // 1s at 500Hz
static constexpr double kDeadZone   = 0.15;   // gear_sonic JOYSTICK_DEADZONE
static constexpr double kFacingRate = 1.5;    // rad/s at full stick (gear_sonic yaw_gain)
static constexpr double kLoopDt     = 0.002;  // 500Hz

InputHandler& InputHandler::instance() {
    static InputHandler inst;
    return inst;
}

void InputHandler::start() {
    stop_ = false;
    loop_thread_ = std::thread(&InputHandler::loop, this);
    std::cout << "[InputHandler] started\n";
}

void InputHandler::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
}

void InputHandler::loop() {
    using namespace std::chrono_literals;
    using clock = std::chrono::steady_clock;

    constexpr auto period = std::chrono::microseconds(2000); // 500Hz

    while (!stop_) {
        auto t0 = clock::now();

        auto ctrl = PicoVRReader::instance().ctrl_buf.GetData();
        if (ctrl) {
            double lx = ctrl->left_axis[0];
            double ly = ctrl->left_axis[1];
            double rx = ctrl->right_axis[0];

            // ── emergency stop (latched) ──────────────────────────
            if (ctrl->left_grip > kEstopGrip && ctrl->right_grip > kEstopGrip) {
                if (++estop_hold_ticks_ >= kEstopHoldTicks && !estop_) {
                    estop_ = true;
                    std::cerr << "[InputHandler] EMERGENCY STOP (grips held)\n";
                }
            } else {
                estop_hold_ticks_ = 0;
            }

            // ── button edge detection ─────────────────────────────
            btn_a_.update(ctrl->btn_a);
            btn_x_.update(ctrl->btn_x);
            btn_y_.update(ctrl->btn_y);

            // ── mode selection ────────────────────────────────────
            bool trigger_held = ctrl->left_trigger > kTriggerHeld ||
                                ctrl->right_trigger > kTriggerHeld;

            // with a trigger held, A becomes the height-down button
            if (!trigger_held && btn_a_.on_press)
                mode_index_ = static_cast<int>(LocomotionMode::IDLE);  // escape from anywhere
            if (btn_y_.on_press) {
                int limit = trigger_held ? kFullModeMax : kBasicModeMax;
                if (mode_index_ < limit)
                    ++mode_index_;
            }
            if (btn_x_.on_press && mode_index_ > 0)
                --mode_index_;

            // ── body height (crouch modes only) ──────────────────
            bool crouched = mode_index_ >= kCrouchFirst && mode_index_ <= kCrouchLast;
            if (crouched) {
                if (height_ < 0.0)
                    height_ = kHeightSeed;  // just entered the crouch band
                if (trigger_held) {
                    if (ctrl->btn_b) height_ += kHeightRate * kLoopDt;  // B = up
                    if (ctrl->btn_a) height_ -= kHeightRate * kLoopDt;
                    height_ = std::clamp(height_, kHeightMin, kHeightMax);
                }
            } else {
                height_ = -1.0;
            }

            // ── facing angle ──────────────────────────────────────
            if (std::abs(rx) > kDeadZone)
                facing_angle_ -= kFacingRate * rx * kLoopDt;

            // ── build MovementState (gear_sonic pico manager scheme:
            //    stick magnitude drives speed, direction stays continuous) ──
            std::array<double, 3> final_face = {std::cos(facing_angle_), std::sin(facing_angle_), 0.0};

            int                   final_mode;
            double                final_speed;
            std::array<double, 3> final_move{0.0, 0.0, 0.0};

            double raw_mag = std::hypot(lx, ly);
            if (raw_mag < kDeadZone) {
                final_mode  = static_cast<int>(LocomotionMode::IDLE);
                final_speed = -1.0;
            } else {
                double mag = std::min((raw_mag - kDeadZone) / (1.0 - kDeadZone), 1.0);
                final_mode = mode_index_;

                switch (static_cast<LocomotionMode>(final_mode)) {
                    case LocomotionMode::SLOW_WALK: final_speed = 0.1 + 0.5 * mag; break;
                    case LocomotionMode::WALK:      final_speed = -1.0;            break;
                    case LocomotionMode::RUN:       final_speed = 1.5 + 3.0 * mag; break;
                    default:                        final_speed = mag;             break;
                }

                // local stick vector, deadzone-rescaled, rotated into the
                // facing frame (gear_sonic: [[-fy, fx], [fx, fy]] @ [-lx, ly])
                double scale = mag / raw_mag;
                double ml0 = -lx * scale;
                double ml1 =  ly * scale;
                final_move = {
                    -final_face[1] * ml0 + final_face[0] * ml1,
                     final_face[0] * ml0 + final_face[1] * ml1,
                     0.0
                };
            }

            movement_buf.SetData(MovementState(final_mode, final_move, final_face, final_speed, height_));
        } else {
            // VR link lost (stale watchdog cleared ctrl_buf) → safe stop:
            // IDLE with mode-default speed/height. Keep the current facing so
            // the robot doesn't turn toward yaw 0 while idling. Also disarm
            // the selected mode — after the link recovers, walking requires
            // an explicit mode selection again.
            mode_index_ = 0;
            std::array<double, 3> face{std::cos(facing_angle_), std::sin(facing_angle_), 0.0};
            movement_buf.SetData(MovementState(
                static_cast<int>(LocomotionMode::IDLE), {0.0, 0.0, 0.0}, face, -1.0, -1.0));
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

} // namespace kist
