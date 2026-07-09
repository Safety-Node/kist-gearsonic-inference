#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"

#include <cmath>
#include <iostream>
#include <chrono>

namespace kist {

// Mode navigation walks the LocomotionMode values linearly (gear_sonic
// remote scheme, clamped at both ends). Bare X/Y stay inside the everyday
// IDLE..RUN range; entering the advanced modes (squat, crawl, boxing, ...)
// requires holding a trigger as a modifier — trigger+X/Y works everywhere.
static constexpr int    kBasicModeMax = static_cast<int>(LocomotionMode::RUN);          // 3
static constexpr int    kFullModeMax  = static_cast<int>(LocomotionMode::INJURED_WALK); // 19 (gear_sonic cap)
static constexpr double kTriggerHeld  = 0.5;
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

            // ── button edge detection ─────────────────────────────
            btn_a_.update(ctrl->btn_a);
            btn_b_.update(ctrl->btn_b);
            btn_x_.update(ctrl->btn_x);
            btn_y_.update(ctrl->btn_y);

            // ── mode selection ────────────────────────────────────
            bool trigger_held = ctrl->left_trigger > kTriggerHeld ||
                                ctrl->right_trigger > kTriggerHeld;
            int prev_mode = mode_index_;

            if (btn_a_.on_press)
                mode_index_ = static_cast<int>(LocomotionMode::IDLE);      // escape from anywhere
            if (btn_b_.on_press)
                mode_index_ = static_cast<int>(LocomotionMode::SLOW_WALK);
            if (btn_y_.on_press) {
                int limit = trigger_held ? kFullModeMax : kBasicModeMax;
                if (mode_index_ < limit)
                    ++mode_index_;
            }
            if (btn_x_.on_press) {
                bool allowed = trigger_held || mode_index_ <= kBasicModeMax;
                if (allowed && mode_index_ > 0)
                    --mode_index_;
            }
            if (mode_index_ != prev_mode)
                std::cout << "[InputHandler] mode -> " << mode_index_ << "\n";

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

            movement_buf.SetData(MovementState(final_mode, final_move, final_face, final_speed, -1.0));
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
