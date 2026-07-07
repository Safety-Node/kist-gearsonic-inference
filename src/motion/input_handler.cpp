#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"

#include <cmath>
#include <iostream>
#include <chrono>

namespace kist {

static constexpr int    kStandingSet[]  = { 0, 1, 2, 3 }; // IDLE, SLOW_WALK, WALK, RUN
static constexpr int    kStandingSetSize = 4;
static constexpr double kDefaultSpeeds[] = { -1.0, 0.4, 1.5, 3.0 };
static constexpr double kSpeedMin[]      = { -1.0, 0.1, 0.8, 2.5 };
static constexpr double kSpeedMax[]      = { -1.0, 0.8, 2.5, 7.5 };
static constexpr double kSpeedStep  = 0.1;
static constexpr double kDeadZone   = 0.05;
static constexpr double kFacingStep = 0.02;

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
            left_trigger_.update(ctrl->left_trigger);
            right_trigger_.update(ctrl->right_trigger);

            // ── emergency stop ────────────────────────────────────
            if (btn_a_.on_press) {
                mode_index_      = 0; // IDLE
                movement_speed_  = kDefaultSpeeds[0];
            }

            // ── mode cycling ──────────────────────────────────────
            if (btn_y_.on_press) {
                mode_index_     = (mode_index_ + 1) % kStandingSetSize;
                movement_speed_ = kDefaultSpeeds[mode_index_];
            }
            if (btn_x_.on_press) {
                mode_index_     = (mode_index_ - 1 + kStandingSetSize) % kStandingSetSize;
                movement_speed_ = kDefaultSpeeds[mode_index_];
            }

            // ── mode reset ────────────────────────────────────────
            if (btn_b_.on_press) {
                mode_index_     = 1; // SLOW_WALK
                movement_speed_ = kDefaultSpeeds[1];
            }

            // ── speed adjustment ──────────────────────────────────
            if (mode_index_ != 0) {
                if (right_trigger_.on_press)
                    movement_speed_ = std::min(movement_speed_ + kSpeedStep, kSpeedMax[mode_index_]);
                if (left_trigger_.on_press)
                    movement_speed_ = std::max(movement_speed_ - kSpeedStep, kSpeedMin[mode_index_]);
            }

            // ── facing angle ──────────────────────────────────────
            if (std::abs(rx) > kDeadZone)
                facing_angle_ -= kFacingStep * rx;

            // ── build MovementState ───────────────────────────────
            int                   final_mode  = kStandingSet[mode_index_];
            std::array<double, 3> final_move  = {0.0, 0.0, 0.0};
            std::array<double, 3> final_face  = {std::cos(facing_angle_), std::sin(facing_angle_), 0.0};
            double                final_speed = movement_speed_;

            if (std::abs(lx) > kDeadZone || std::abs(ly) > kDeadZone) {
                double raw_angle    = std::atan2(ly, lx);
                double bin_size     = M_PI / 4.0;
                double binned_angle = std::round(raw_angle / bin_size) * bin_size;
                double move_dir     = binned_angle - M_PI / 2.0 + facing_angle_;
                final_move = {std::cos(move_dir), std::sin(move_dir), 0.0};
            } else if (mode_index_ != 0) {
                // dead zone + non-IDLE → send IDLE temporarily
                final_mode  = static_cast<int>(LocomotionMode::IDLE);
                final_speed = -1.0;
            }

            movement_buf.SetData(MovementState(final_mode, final_move, final_face, final_speed, -1.0));
        } else {
            // VR link lost (stale watchdog cleared ctrl_buf) → safe stop:
            // IDLE with mode-default speed/height. Keep the current facing so
            // the robot doesn't turn toward yaw 0 while idling.
            std::array<double, 3> face{std::cos(facing_angle_), std::sin(facing_angle_), 0.0};
            movement_buf.SetData(MovementState(
                static_cast<int>(LocomotionMode::IDLE), {0.0, 0.0, 0.0}, face, -1.0, -1.0));
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

} // namespace kist
