// Integration test: PlannerInference with the real VR device and robot.
//
//   PicoVRReader (VR) ─→ InputHandler ─→ movement_buf ─┐
//                                                       ├─→ PlannerInference
//   UnitreeStateReader (DDS) ─→ unitree_state_buf ──────┘
//
// Requires:
//   - PICO VR headset connected (PXREA service reachable).
//   - Robot (or simulator) publishing LowState on the configured DDS
//     network interface (config.yaml: unitree.network_interface).
//
// Read-only: subscribes to robot state, never sends motor commands.
//
// Drive the joystick / buttons and watch the planner replan:
//   - mode/facing/height change   → replan immediately
//   - moving with speed != 0      → replan on the per-mode timer
//   - idle with no input changes  → no replanning (age keeps growing)

#include "common/config.hpp"
#include "motion/input_handler.hpp"
#include "pico/pico_vr_reader.hpp"
#include "planner/planner_inference.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(config_path);

    auto unitree_cfg = kist::Config::instance().root()["unitree"];
    auto domain_id   = unitree_cfg["domain_id"].as<int>();
    auto interface   = unitree_cfg["network_interface"].as<std::string>();

    auto planner_cfg = kist::Config::instance().root()["planner"];
    auto onnx_path   = planner_cfg["model_path"].as<std::string>();
    auto precision   = planner_cfg["precision"].as<std::string>("fp16") == "fp32"
                           ? Precision::FP32 : Precision::FP16;
    auto height      = planner_cfg["default_height"].as<double>(0.788740);
    auto seed        = planner_cfg["initial_random_seed"].as<int64_t>(1234);

    // ── VR input chain ──────────────────────────────────────────
    auto& vr = kist::PicoVRReader::instance();
    if (!vr.start())
        return 1;
    kist::InputHandler::instance().start();

    // ── robot state (read-only DDS subscriber) ──────────────────
    auto& robot = kist::UnitreeStateReader::instance();
    robot.start(domain_id, interface);

    std::cout << "Waiting for robot state on \"" << interface << "\"...\n";
    while (!robot.unitree_state_buf.GetData()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "Robot state received.\n";

    // ── planner ─────────────────────────────────────────────────
    auto& planner = kist::PlannerInference::instance();
    if (!planner.start(onnx_path, precision, height, seed)) {
        return 1;
    }

    std::cout << "Running. Drive the joystick to trigger replans. (Ctrl+C to exit)\n";

    while (true) {
        auto ms     = kist::InputHandler::instance().movement_buf.GetData();
        auto motion = planner.motion_50hz_buf.GetDataWithTime();

        if (ms) {
            std::cout << std::fixed << std::setprecision(3)
                      << "[Input]   mode=" << ms->locomotion_mode
                      << "  speed=" << ms->movement_speed
                      << "  move=(" << ms->movement_direction[0] << ", "
                                    << ms->movement_direction[1] << ")"
                      << "  face=(" << ms->facing_direction[0] << ", "
                                    << ms->facing_direction[1] << ")"
                      << "\n";
        }
        if (motion.HasData()) {
            const auto& first = motion.data->frames[0];
            std::cout << std::fixed << std::setprecision(4)
                      << "[Planner] age=" << std::setprecision(0) << motion.GetAgeMs() << "ms"
                      << std::setprecision(4)
                      << "  timesteps=" << motion.data->timesteps
                      << "  gen_frame=" << motion.data->gen_frame
                      << "  frame0.pos=(" << first.position[0] << ", " << first.position[1] << ", " << first.position[2] << ")"
                      << "  frame0.joint[0]=" << first.joints[0]
                      << "\n";
        } else {
            std::cout << "[Planner] waiting for first plan...\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    planner.stop();
    robot.stop();
    kist::InputHandler::instance().stop();
    vr.stop();
    return 0;
}
