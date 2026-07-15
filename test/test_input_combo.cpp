// HoldTrigger unit test — the combo-grace state machine that
// InputHandler uses for all face-button gestures.
//
// Simulates ticks at the 500Hz loop rate (kSingleHoldTicks=15 -> 30ms,
// kEstopHoldTicks=500 -> 1s). Verifies: single-button gesture fires,
// second button entering the window voids it, one-shot semantics (no
// re-fire while held), and post-release re-fire.

#include <cstdio>

namespace {

struct HoldTrigger {
    int  ticks{0};
    bool fired{false};
    bool tick(bool active_alone, int threshold) {
        if (!active_alone) { ticks = 0; fired = false; return false; }
        if (fired) return false;
        if (++ticks >= threshold) { fired = true; return true; }
        return false;
    }
};

constexpr int kSingleHoldTicks = 15;
constexpr int kEstopHoldTicks  = 500;

int check(const char* label, bool ok) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", label);
    return ok ? 0 : 1;
}

} // namespace

int main() {
    int fails = 0;

    // ── single button held alone fires exactly once at threshold ──
    {
        HoldTrigger t;
        for (int i = 0; i < kSingleHoldTicks - 1; ++i)
            fails += check("no early fire", !t.tick(true, kSingleHoldTicks));
        fails += check("fires on threshold tick", t.tick(true, kSingleHoldTicks));
        // one-shot: subsequent ticks stay quiet even though input is held
        for (int i = 0; i < 100; ++i)
            fails += check("no re-fire while held", !t.tick(true, kSingleHoldTicks));
    }

    // ── combo (active_alone=false) resets counter and never fires ──
    {
        HoldTrigger t;
        // simulate: A pressed for 5 ticks, then X joins → active_alone=false
        for (int i = 0; i < 5; ++i)
            t.tick(true, kSingleHoldTicks);
        for (int i = 0; i < 100; ++i)
            fails += check("combo suppresses", !t.tick(false, kSingleHoldTicks));
        // release: still no phantom fire on next tick
        fails += check("release stays quiet", !t.tick(false, kSingleHoldTicks));
    }

    // ── release resets: next hold fires again after threshold ──
    {
        HoldTrigger t;
        for (int i = 0; i < kSingleHoldTicks; ++i)
            t.tick(true, kSingleHoldTicks);
        // release
        for (int i = 0; i < 10; ++i)
            t.tick(false, kSingleHoldTicks);
        // re-press
        for (int i = 0; i < kSingleHoldTicks - 1; ++i)
            fails += check("no early fire (2nd press)", !t.tick(true, kSingleHoldTicks));
        fails += check("fires on 2nd press", t.tick(true, kSingleHoldTicks));
    }

    // ── typical A+X combo entry pattern ──
    //    tick 0: A rises alone → hold=1
    //    tick 3: X arrives, active_alone becomes false → hold=0
    //    tick 3..1000: never fires (combo suppresses)
    {
        HoldTrigger a;
        for (int i = 0; i < 3; ++i) {
            bool alone = true;  // A alone
            a.tick(alone, kSingleHoldTicks);
        }
        // The 30ms grace shrunk to 6ms before X joined
        // and everything cancels.
        for (int i = 0; i < 1000; ++i)
            fails += check("A+X entry: A never fires",
                           !a.tick(false, kSingleHoldTicks));
    }

    // ── e-stop hold: 4-btn condition == active_alone=true for the estop ──
    //    (this mirrors "if (n_face == 4) ++estop_ticks"; a HoldTrigger with
    //    threshold=500 fires once after 1s.)
    {
        HoldTrigger estop;
        // 499 ticks of "all four held"
        for (int i = 0; i < kEstopHoldTicks - 1; ++i)
            fails += check("no early e-stop", !estop.tick(true, kEstopHoldTicks));
        fails += check("e-stop fires at 1s", estop.tick(true, kEstopHoldTicks));
    }

    // ── e-stop cancellation: one button released mid-hold ──
    {
        HoldTrigger estop;
        for (int i = 0; i < 400; ++i)  // 0.8s
            estop.tick(true, kEstopHoldTicks);
        // one button releases → condition breaks
        for (int i = 0; i < 200; ++i)
            fails += check("e-stop cancelled by release", !estop.tick(false, kEstopHoldTicks));
        // resume all four: needs fresh full hold
        for (int i = 0; i < kEstopHoldTicks - 1; ++i)
            fails += check("no premature re-arm", !estop.tick(true, kEstopHoldTicks));
        fails += check("re-arms after full 1s", estop.tick(true, kEstopHoldTicks));
    }

    std::printf("\n%s (%d failures)\n", fails == 0 ? "ALL PASS" : "FAIL", fails);
    return fails == 0 ? 0 : 1;
}
