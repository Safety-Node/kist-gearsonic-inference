// Injection test of the NavCommand arbitration (no VR, no robot).
//
// Synthetic controller states go into ctrl_buf (the reader is not
// started) and nav commands into nav_buf; the arbitrated MovementState
// coming out of movement_buf is checked scenario by scenario:
//   1. neutral stick + fresh nav       -> nav drives (SLOW_WALK / WALK)
//   2. nav zero velocity ("arrived")   -> IDLE
//   3. stick deflected                 -> joystick wins over nav
//   4. stick released again            -> nav resumes (momentary priority)
//   5. nav stops publishing (stale)    -> IDLE
//   6. nav vyaw                        -> facing integrates

#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-42s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

static MovementState latest() {
    return *InputHandler::instance().movement_buf.GetData();
}

static void settle(int ms = 30) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    auto& ctrl_buf = PicoVRReader::instance().ctrl_buf;  // injected directly
    auto& nav_buf  = InputHandler::instance().nav_buf;

    PicoVRController neutral;  // sticks at zero, no buttons
    ctrl_buf.SetData(neutral);

    InputHandler::instance().start();
    settle(50);

    // 1a. neutral stick, no nav -> IDLE
    check("neutral stick, no nav -> IDLE",
          latest().locomotion_mode == 0 && latest().movement_speed == -1.0);

    // 1b. fresh slow nav -> SLOW_WALK at |v|, direction = facing-forward
    nav_buf.SetData({0.3, 0.0, 0.0});
    ctrl_buf.SetData(neutral);
    settle();
    {
        auto m = latest();
        bool dir_fwd = m.movement_direction[0] > 0.99;  // facing starts at yaw 0 -> +x
        check("fresh nav 0.3m/s -> SLOW_WALK, fwd",
              m.locomotion_mode == 1 && std::abs(m.movement_speed - 0.3) < 1e-6 && dir_fwd);
    }

    // 1c. fast nav -> WALK with mode-default speed
    nav_buf.SetData({0.9, 0.0, 0.0});
    settle();
    check("fresh nav 0.9m/s -> WALK, default speed",
          latest().locomotion_mode == 2 && latest().movement_speed == -1.0);

    // 2. zero velocity ("arrived") -> IDLE
    nav_buf.SetData({0.0, 0.0, 0.0});
    settle();
    check("nav zeros (arrived) -> IDLE", latest().locomotion_mode == 0);

    // 3. stick deflected -> joystick wins unconditionally
    nav_buf.SetData({0.3, 0.0, 0.0});
    {
        PicoVRController stick = neutral;
        stick.left_axis[1] = 1.0;  // full forward
        ctrl_buf.SetData(stick);
    }
    settle();
    {
        auto m = latest();
        // joystick path: mode_index_ is IDLE (never armed) -> mode 0 but
        // movement nonzero — distinguishable from the nav SLOW_WALK path
        check("stick deflected -> joystick wins",
              m.locomotion_mode == 0 && std::hypot(m.movement_direction[0], m.movement_direction[1]) > 0.5);
    }

    // 4. stick released -> nav resumes on the next ticks
    ctrl_buf.SetData(neutral);
    nav_buf.SetData({0.3, 0.0, 0.0});
    settle();
    check("stick released -> nav resumes", latest().locomotion_mode == 1);

    // 5. nav goes silent -> stale -> IDLE (fallback)
    settle(350);  // > kNavStaleMs with no SetData
    check("nav silent 350ms -> stale -> IDLE", latest().locomotion_mode == 0);

    // 6. vyaw integrates into facing while nav drives
    {
        double face0 = std::atan2(latest().facing_direction[1], latest().facing_direction[0]);
        for (int i = 0; i < 10; ++i) {          // keep fresh while turning
            nav_buf.SetData({0.3, 0.0, 1.0});   // 1 rad/s
            settle(20);
        }
        double face1 = std::atan2(latest().facing_direction[1], latest().facing_direction[0]);
        double turned = face1 - face0;
        check("nav vyaw 1rad/s x 0.2s -> facing +~0.2rad",
              turned > 0.1 && turned < 0.35);
    }

    InputHandler::instance().stop();
    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
