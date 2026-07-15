// Verifies the grip/trigger -> Dex3-1 finger mapping without touching DDS.
// - motor 0..2 (thumb) driven by grip
// - motor 3..6 (index+middle) driven by trigger
// - endpoints come from kDex3{Left,Right}{Open,Close} (empirical, real hand)
// Live wiring (VR + real robot) is checked from the main binary, not here.

#include "unitree/hand_command.hpp"

#include <cmath>
#include <cstdio>

namespace {

kist::HandCommand from_grip_and_trigger(double grip, double trigger, bool is_left) {
    grip    = std::fmax(0.0, std::fmin(1.0, grip));
    trigger = std::fmax(0.0, std::fmin(1.0, trigger));
    kist::HandCommand cmd;
    for (int i = 0; i < kist::kMotorEnd; ++i) {
        float open_q  = is_left ? kist::kDex3LeftOpen[i]  : kist::kDex3RightOpen[i];
        float close_q = is_left ? kist::kDex3LeftClose[i] : kist::kDex3RightClose[i];
        double t = (i < kist::kFingerBegin) ? grip : trigger;
        cmd.q[i] = open_q + static_cast<float>(t) * (close_q - open_q);
    }
    return cmd;
}

bool near_equal(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

int check(const char* label, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", label);
    return ok ? 0 : 1;
}

} // namespace

int main() {
    int fails = 0;

    // ── input=0 -> open pose ──
    {
        auto l = from_grip_and_trigger(0.0, 0.0, /*is_left=*/true);
        auto r = from_grip_and_trigger(0.0, 0.0, /*is_left=*/false);
        for (int i = 0; i < kist::kMotorEnd; ++i) {
            fails += check("left  open  matches endpoint", near_equal(l.q[i], kist::kDex3LeftOpen[i]));
            fails += check("right open  matches endpoint", near_equal(r.q[i], kist::kDex3RightOpen[i]));
        }
    }

    // ── input=1 -> close pose ──
    {
        auto l = from_grip_and_trigger(1.0, 1.0, /*is_left=*/true);
        auto r = from_grip_and_trigger(1.0, 1.0, /*is_left=*/false);
        for (int i = 0; i < kist::kMotorEnd; ++i) {
            fails += check("left  close matches endpoint", near_equal(l.q[i], kist::kDex3LeftClose[i]));
            fails += check("right close matches endpoint", near_equal(r.q[i], kist::kDex3RightClose[i]));
        }
    }

    // ── fingers all open at q=0 (empirical: 0 = extended = open) ──
    {
        auto lo = from_grip_and_trigger(0.5, 0.0, /*is_left=*/true);   // trigger=0 -> fingers open
        auto ro = from_grip_and_trigger(0.5, 0.0, /*is_left=*/false);
        for (int i = kist::kFingerBegin; i < kist::kMotorEnd; ++i) {
            fails += check("left  finger open  == 0", near_equal(lo.q[i], 0.0f));
            fails += check("right finger open  == 0", near_equal(ro.q[i], 0.0f));
        }
    }

    // ── fingers curl to signed extreme at trigger=1 ──
    {
        auto lc = from_grip_and_trigger(0.5, 1.0, /*is_left=*/true);
        auto rc = from_grip_and_trigger(0.5, 1.0, /*is_left=*/false);
        for (int i = kist::kFingerBegin; i < kist::kMotorEnd; ++i) {
            fails += check("left  finger close is negative", lc.q[i] < 0.0f);
            fails += check("right finger close is positive", rc.q[i] > 0.0f);
        }
    }

    // ── axis independence: grip moves only thumb, trigger only fingers ──
    {
        auto base   = from_grip_and_trigger(0.5, 0.5, /*is_left=*/true);
        auto more_g = from_grip_and_trigger(0.9, 0.5, /*is_left=*/true);
        auto more_t = from_grip_and_trigger(0.5, 0.9, /*is_left=*/true);
        for (int i = 0; i < kist::kFingerBegin; ++i) {
            fails += check("grip moves thumb",     !near_equal(base.q[i], more_g.q[i]));
            fails += check("trigger leaves thumb",  near_equal(base.q[i], more_t.q[i]));
        }
        for (int i = kist::kFingerBegin; i < kist::kMotorEnd; ++i) {
            fails += check("trigger moves fingers", !near_equal(base.q[i], more_t.q[i]));
            fails += check("grip leaves fingers",    near_equal(base.q[i], more_g.q[i]));
        }
    }

    // ── clamp ──
    {
        auto neg = from_grip_and_trigger(-0.5, -0.5, /*is_left=*/true);
        auto big = from_grip_and_trigger( 2.0,  2.0, /*is_left=*/true);
        for (int i = 0; i < kist::kMotorEnd; ++i) {
            fails += check("clamp low  == open",  near_equal(neg.q[i], kist::kDex3LeftOpen[i]));
            fails += check("clamp high == close", near_equal(big.q[i], kist::kDex3LeftClose[i]));
        }
    }

    std::printf("\n%s (%d failures)\n", fails == 0 ? "ALL PASS" : "FAIL", fails);
    return fails == 0 ? 0 : 1;
}
