// Dry-run of the full TASK-7 pipeline, no motor output anywhere:
//   fake MovementState -> PlannerInference -> WholeBodyController
//   (blend + observations -> encoder -> decoder -> motor_command_buf)
// Robot state is faked (standing pose, re-injected to stay fresh) so this
// runs offline; on the real robot UnitreeStateReader provides it instead.

#include "common/config.hpp"
#include "common/robot_params.hpp"
#include "control/whole_body_controller.hpp"
#include "motion/input_handler.hpp"
#include "planner/planner_inference.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(config_path);
    auto root = kist::Config::instance().root();

    // Fake robot state: default standing pose
    kist::UnitreeState fake_state{};
    for (int i = 0; i < 29; ++i)
        fake_state.motors[i].q = kist::g1_default_angles[i];
    fake_state.imu_pelvis.quaternion = {1.0, 0.0, 0.0, 0.0};
    kist::UnitreeStateReader::instance().unitree_state_buf.SetData(fake_state);

    // Fake movement command: SLOW_WALK forward
    kist::InputHandler::instance().movement_buf.SetData(kist::MovementState(
        static_cast<int>(kist::LocomotionMode::SLOW_WALK),
        {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.4, -1.0));

    auto& planner = kist::PlannerInference::instance();
    if (!planner.start(root["planner"]["model_path"].as<std::string>()))
        return 1;

    auto& control = kist::WholeBodyController::instance();
    // Planner context <- control loop's blended motion + cursor
    planner.set_playback_provider([&control](kist::MotionSequence50Hz& m, int& c) {
        return control.playback_snapshot(m, c);
    });
    if (!control.start(root["control"]["encoder_path"].as<std::string>(),
                       root["control"]["decoder_path"].as<std::string>(),
                       /*auto_start_control=*/true))
        return 1;

    std::cout << "Dry run: printing motor commands. (Ctrl+C to exit)\n";

    while (true) {
        // keep the fake robot state fresh for the safety check
        kist::UnitreeStateReader::instance().unitree_state_buf.SetData(fake_state);

        auto cmd = control.motor_command_buf.GetDataWithTime();
        auto t   = control.last_timing();
        if (cmd.HasData()) {
            std::cout << std::fixed << std::setprecision(4)
                      << "[Cmd] age=" << std::setprecision(0) << cmd.GetAgeMs() << "ms"
                      << std::setprecision(4)
                      << "  q[0]=" << cmd.data->q_target[0]
                      << "  q[3]=" << cmd.data->q_target[3]
                      << "  q[15]=" << cmd.data->q_target[15]
                      << "  kp[0]=" << std::setprecision(1) << cmd.data->kp[0]
                      << "  | tick: enc=" << t.encoder
                      << "us dec=" << t.decoder << "us total=" << t.total << "us\n";
        } else {
            std::cout << "[Cmd] waiting... (state=" << static_cast<int>(control.state()) << ")\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}
