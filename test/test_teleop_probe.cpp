// Deterministic probe of TeleopTracker: fixed synthetic SMPL body poses
// injected into body_buf (reader not started), outputs printed at high
// precision. Verified against a Python reference implementing the original
// gear_sonic ThreePointPose math; also a regression tool for refactors.

#include "pico/pico_vr_reader.hpp"
#include "teleop/g1_arm_fk.hpp"
#include "teleop/teleop_tracker.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

using namespace kist;

static PicoVRBodyPose make_body(double phase) {
    PicoVRBodyPose b;
    for (int i = 0; i < 24; ++i) {
        auto& j = b.joints[i];
        j[0] = 0.10 * std::sin(0.3 * i + phase);
        j[1] = 0.90 + 0.02 * i + 0.05 * std::cos(0.7 * i + phase);
        j[2] = 0.20 * std::cos(0.4 * i - phase);

        double ax = std::sin(0.2 * i + 1.0 + phase);
        double ay = std::cos(0.3 * i - phase);
        double az = std::sin(0.5 * i + 2.0);
        double n  = std::sqrt(ax * ax + ay * ay + az * az);
        double ang = 0.1 * i + 0.2 + 0.1 * phase;
        j[3] = ax / n * std::sin(ang / 2);
        j[4] = ay / n * std::sin(ang / 2);
        j[5] = az / n * std::sin(ang / 2);
        j[6] = std::cos(ang / 2);
    }
    return b;
}

static void print_out(const char* tag) {
    auto v = TeleopTracker::instance().vr3point_buf.GetData();
    if (!v) {
        std::printf("%s: <empty>\n", tag);
        return;
    }
    std::printf("%s pos :", tag);
    for (double p : v->position) std::printf(" %.6f", p);
    std::printf("\n%s quat:", tag);
    for (double q : v->orientation) std::printf(" %.6f", q);
    std::printf("\n");
}

int main() {
    auto& tracker  = TeleopTracker::instance();
    auto& body_buf = PicoVRReader::instance().body_buf;  // injected directly

    tracker.start();

    // frame 1: capture calibration, output = calibrated same frame
    body_buf.SetData(make_body(0.0));
    tracker.request_calibration();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    print_out("calib");

    // frame 2: different pose through the captured calibration
    body_buf.SetData(make_body(0.8));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    print_out("moved");

    // frame 3: recalibration with a measured-q wrist reference —
    // neck must be preserved, wrist offsets recomputed against FK(q)
    tracker.set_measured_q_provider([](std::array<double, 29>& q) {
        for (int i = 0; i < 29; ++i)
            q[i] = 0.3 * std::sin(0.5 * i + 1.0);
        return true;
    });
    tracker.request_calibration();
    body_buf.SetData(make_body(1.6));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    print_out("recal");

    tracker.stop();

    // ── wrist FK: zero pose (must match calibration reference constants),
    //    then a deterministic non-zero pose (verified vs URDF-chain python) ──
    std::array<double, 29> q{};
    auto pl = g1_wrist_fk(q, true);
    auto pr = g1_wrist_fk(q, false);
    std::printf("fk0 L: %.6f %.6f %.6f | %.6f %.6f %.6f %.6f\n",
                pl.position[0], pl.position[1], pl.position[2],
                pl.quaternion[0], pl.quaternion[1], pl.quaternion[2], pl.quaternion[3]);
    std::printf("fk0 R: %.6f %.6f %.6f | %.6f %.6f %.6f %.6f\n",
                pr.position[0], pr.position[1], pr.position[2],
                pr.quaternion[0], pr.quaternion[1], pr.quaternion[2], pr.quaternion[3]);

    for (int i = 0; i < 29; ++i) q[i] = 0.3 * std::sin(0.5 * i + 1.0);
    pl = g1_wrist_fk(q, true);
    pr = g1_wrist_fk(q, false);
    std::printf("fk1 L: %.6f %.6f %.6f | %.6f %.6f %.6f %.6f\n",
                pl.position[0], pl.position[1], pl.position[2],
                pl.quaternion[0], pl.quaternion[1], pl.quaternion[2], pl.quaternion[3]);
    std::printf("fk1 R: %.6f %.6f %.6f | %.6f %.6f %.6f %.6f\n",
                pr.position[0], pr.position[1], pr.position[2],
                pr.quaternion[0], pr.quaternion[1], pr.quaternion[2], pr.quaternion[3]);
    return 0;
}
