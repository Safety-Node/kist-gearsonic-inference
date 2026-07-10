// Main deployment entry — THE ROBOT WILL MOVE.
//
//   VR joystick -> InputHandler ─┐
//   VR body    -> TeleopTracker ─┤ -> PlannerInference (10Hz)
//                                └-> WholeBodyController (50Hz)
//                                    -> UnitreeCommandWriter (500Hz, rt/lowcmd)
//
// Operator flow:
//   Enter        : 3s INIT ramp to the standing pose
//   Enter        : hand over to the policy (WAIT_FOR_CONTROL -> CONTROL)
//   B held 1s    : teleop on (calibrate at reference pose) / off
//   both grips 1s: emergency stop (latched, damping)
//   Ctrl+C       : damping, orderly shutdown

#include "common/config.hpp"
#include "control/whole_body_controller.hpp"
#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"
#include "planner/planner_inference.hpp"
#include "teleop/g1_arm_fk.hpp"
#include "teleop/teleop_tracker.hpp"
#include "unitree/unitree_command_writer.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <cmath>

#include <atomic>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_quit{false};
static void on_sigint(int) { g_quit = true; }

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(config_path);
    auto root = kist::Config::instance().root();

    std::signal(SIGINT, on_sigint);

    // ── inputs ──────────────────────────────────────────────────
    // VR is mandatory: the joystick drives the planner and the grips
    // are the operator e-stop.
    auto& vr = kist::PicoVRReader::instance();
    if (!vr.start())
        return 1;
    kist::InputHandler::instance().start();
    // wrist calibration reference = FK of the robot's measured joints
    kist::TeleopTracker::instance().set_measured_q_provider(
        [](std::array<double, 29>& q) {
            auto st = kist::UnitreeStateReader::instance().unitree_state_buf.GetData();
            if (!st)
                return false;
            for (int i = 0; i < 29; ++i)
                q[i] = st->motors[i].q;
            return true;
        });
    kist::TeleopTracker::instance().start();

    auto& robot = kist::UnitreeStateReader::instance();
    if (!robot.start(root["unitree"]["domain_id"].as<int>(),
                     root["unitree"]["network_interface"].as<std::string>()))
        return 1;

    std::cout << "Waiting for robot state...\n";
    while (!robot.unitree_state_buf.GetData()) {
        if (g_quit) return 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // ── planner + control ───────────────────────────────────────
    auto& planner = kist::PlannerInference::instance();
    if (!planner.start(root["planner"]["model_path"].as<std::string>()))
        return 1;

    auto& control = kist::WholeBodyController::instance();
    planner.set_playback_provider([&control](kist::MotionSequence50Hz& m, int& c) {
        return control.playback_snapshot(m, c);
    });

    // ── gate 1: INIT ramp ───────────────────────────────────────
    std::cout << "\n*** ROBOT WILL MOVE ***\n"
              << "Press Enter to start the 3s ramp to the standing pose...\n";
    std::cin.get();
    if (g_quit) return 0;

    auto& writer = kist::UnitreeCommandWriter::instance();
    if (!writer.start(&control.motor_command_buf))
        return 1;
    if (!control.start(root["control"]["encoder_path"].as<std::string>(),
                       root["control"]["decoder_path"].as<std::string>(),
                       /*auto_start_control=*/false)) {
        writer.stop();
        return 1;
    }

    while (control.state() == kist::WholeBodyController::State::INIT && !g_quit)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ── gate 2: policy control ──────────────────────────────────
    if (!g_quit) {
        std::cout << "\nStanding pose reached. Press Enter to hand over to the policy...\n";
        std::cin.get();
        if (!g_quit)
            control.start_control();
    }

    // ── status until Ctrl+C ─────────────────────────────────────
    while (!g_quit) {
        auto t = control.last_timing();
        std::cout << "state=" << static_cast<int>(control.state())
                  << "  mode=" << kist::InputHandler::instance().mode()
                  << "  enc=" << control.encoder_mode()
                  << "  tick=" << t.total << "us"
                  << (kist::InputHandler::instance().estop() ? "  [E-STOP]" : "");

        // teleop tracking error: VR wrist target vs measured-joint FK (cm).
        // Splits accuracy issues: target side (calibration/tracking) vs
        // policy tracking side.
        auto vr3 = kist::TeleopTracker::instance().vr3point_buf.GetData();
        auto st  = kist::UnitreeStateReader::instance().unitree_state_buf.GetData();
        if (vr3 && st) {
            std::array<double, 29> q;
            for (int i = 0; i < 29; ++i)
                q[i] = st->motors[i].q;
            for (bool left : {true, false}) {
                auto fk = kist::g1_wrist_fk(q, left);
                const double* tgt = vr3->position.data() + (left ? 0 : 3);
                double e = std::sqrt(std::pow(tgt[0] - fk.position[0], 2) +
                                     std::pow(tgt[1] - fk.position[1], 2) +
                                     std::pow(tgt[2] - fk.position[2], 2));
                std::cout << (left ? "  errL=" : "  errR=")
                          << std::fixed << std::setprecision(1) << e * 100.0 << "cm";
            }
        }
        std::cout << "\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ── shutdown: damping before anything else stops ────────────
    std::cout << "\nStopping: damping...\n";
    control.stop();
    writer.stop();
    planner.stop();
    kist::TeleopTracker::instance().stop();
    kist::InputHandler::instance().stop();
    robot.stop();
    vr.stop();
    return 0;
}
