// Verify the PlannerInference class runs end-to-end.
//   - Feeds a fake UnitreeState into UnitreeStateReader.unitree_state_buf so
//     the planner can initialize without a robot (initial IDLE plan uses the
//     robot's measured joint angles; here all zeros).
//   - Feeds a fake MovementState directly into InputHandler.movement_buf so
//     no PICO VR hardware is needed for this test.
//   - Starts PlannerInference (loads planner_sonic.trt or builds it from .onnx).
//   - Polls motion_50hz_buf and prints the first frame of the 50Hz sequence.

#include "common/config.hpp"
#include "motion/input_handler.hpp"
#include "motion/movement_state.hpp"
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
    auto planner_cfg = kist::Config::instance().root()["planner"];
    auto onnx_path   = planner_cfg["model_path"].as<std::string>();
    auto precision   = planner_cfg["precision"].as<std::string>("fp16") == "fp32"
                           ? Precision::FP32 : Precision::FP16;
    auto height      = planner_cfg["default_height"].as<double>(0.788740);
    auto seed        = planner_cfg["initial_random_seed"].as<int64_t>(1234);

    // Inject a fake robot state (standing pose, all joints 0 rad) so the
    // planner's initialization path has joint angles to build context from.
    kist::UnitreeStateReader::instance().unitree_state_buf.SetData(kist::UnitreeState{});

    // Inject a fake MovementState (SLOW_WALK forward, 0.4 m/s).
    // Differs from the initial IDLE plan, so it triggers one mode-change
    // replan, then periodic replans every 1 s (speed != 0).
    kist::MovementState fake(
        static_cast<int>(kist::LocomotionMode::SLOW_WALK),
        {1.0, 0.0, 0.0},   // movement_direction: +x
        {1.0, 0.0, 0.0},   // facing_direction: +x
        0.4,               // movement_speed
        -1.0               // height (mode default)
    );
    kist::InputHandler::instance().movement_buf.SetData(fake);

    auto& planner = kist::PlannerInference::instance();
    if (!planner.start(onnx_path, precision, height, seed)) {
        return 1;
    }

    std::cout << "Waiting for planner output... (Ctrl+C to exit)\n";

    while (true) {
        auto motion = planner.motion_50hz_buf.GetDataWithTime();
        if (motion.HasData()) {
            const auto& first = motion.data->frames[0];
            std::cout << std::fixed << std::setprecision(4)
                      << "[PlannerInference] age=" << motion.GetAgeMs() << "ms"
                      << "  timesteps=" << motion.data->timesteps
                      << "  gen_frame=" << motion.data->gen_frame
                      << "  frame0.pos=(" << first.position[0] << ", " << first.position[1] << ", " << first.position[2] << ")"
                      << "  frame0.quat=(" << first.quaternion[0] << ", " << first.quaternion[1] << ", " << first.quaternion[2] << ", " << first.quaternion[3] << ")"
                      << "  frame0.joint[0]=" << first.joints[0]
                      << "  frame0.dq[0]=" << first.joint_velocities[0]
                      << "\n";
        } else {
            std::cout << "[PlannerInference] waiting...\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    planner.stop();
    return 0;
}
