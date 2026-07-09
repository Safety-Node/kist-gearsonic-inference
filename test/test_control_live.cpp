// FULL LIVE TEST — THE ROBOT WILL MOVE.
//
//   VR joystick -> InputHandler -> PlannerInference -> ControlLoop
//     -> motor_command_buf -> UnitreeCommandWriter -> rt/lowcmd (DDS)
//
// Safety gating:
//   1. First Enter  : starts the writer + control loop -> 3s INIT ramp
//                     (robot moves to the default standing pose).
//   2. Second Enter : WAIT_FOR_CONTROL -> CONTROL (policy takes over).
//   Ctrl+C          : damping command (kp=0, kd=8) then exit.
//
// Preconditions: robot suspended or in a safe area, built-in sport
// controller released (damping via remote), headset connected.

#include "common/config.hpp"
#include "control/control_loop.hpp"
#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"
#include "planner/planner_inference.hpp"
#include "unitree/unitree_command_writer.hpp"
#include "unitree/unitree_state_reader.hpp"

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
    auto& vr = kist::PicoVRReader::instance();
    vr.start();
    kist::InputHandler::instance().start();

    auto& robot = kist::UnitreeStateReader::instance();
    if (!robot.start(root["unitree"]["domain_id"].as<int>(),
                     root["unitree"]["network_interface"].as<std::string>()))
        return 1;

    std::cout << "Waiting for robot state...\n";
    while (!robot.unitree_state_buf.GetData()) {
        if (g_quit) return 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "Robot state received.\n";

    // ── planner + control ───────────────────────────────────────
    auto& planner = kist::PlannerInference::instance();
    if (!planner.start(root["planner"]["model_path"].as<std::string>()))
        return 1;

    auto& control = kist::ControlLoop::instance();
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

    while (control.state() == kist::ControlLoop::State::INIT && !g_quit)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ── gate 2: policy control ──────────────────────────────────
    if (!g_quit) {
        std::cout << "\nStanding pose reached. Press Enter to hand over to the policy...\n";
        std::cin.get();
        if (!g_quit)
            control.start_control();
    }

    // ── monitor until Ctrl+C ────────────────────────────────────
    while (!g_quit) {
        auto cmd = control.motor_command_buf.GetDataWithTime();
        auto t   = control.last_timing();
        auto ms  = kist::InputHandler::instance().movement_buf.GetData();
        if (cmd.HasData()) {
            std::cout << std::fixed << std::setprecision(3)
                      << "[Live] state=" << static_cast<int>(control.state())
                      << "  mode=" << (ms ? ms->locomotion_mode : -1)
                      << "  q[0]=" << cmd.data->q_target[0]
                      << "  q[3]=" << cmd.data->q_target[3]
                      << "  tick=" << t.total << "us\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // ── shutdown: damping before anything else stops ────────────
    std::cout << "\nStopping: damping...\n";
    control.stop();
    writer.stop();   // publishes damping burst
    planner.stop();
    kist::InputHandler::instance().stop();
    robot.stop();
    vr.stop();
    return 0;
}
