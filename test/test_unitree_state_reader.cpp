#include "common/config.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(config_path);

    auto unitree_cfg = kist::Config::instance().root()["unitree"];
    auto domain_id   = unitree_cfg["domain_id"].as<int>();
    auto interface   = unitree_cfg["network_interface"].as<std::string>();

    auto& reader = kist::UnitreeStateReader::instance();
    if (!reader.start(domain_id, interface))
        return 1;

    std::cout << "Waiting for Unitree state... (Ctrl+C to exit)\n";

    while (true) {
        auto us  = reader.unitree_state_buf.GetDataWithTime();
        auto imu = reader.imu_torso_buf.GetData();

        if (us.HasData()) {
            const auto& q = us.data->imu_pelvis.quaternion;
            std::cout << std::fixed << std::setprecision(4)
                      << "[UnitreeState] age=" << us.GetAgeMs() << "ms"
                      << "  tick="     << us.data->tick
                      << "  imu_pelvis_quat=(" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ")"
                      << "  motor[0].q=" << us.data->motors[0].q
                      << "\n";
        } else {
            std::cout << "[UnitreeState] waiting...\n";
        }

        if (imu) {
            const auto& q = imu->quaternion;
            std::cout << std::fixed << std::setprecision(4)
                      << "[Torso IMU] quat=(" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ")\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    reader.stop();
    return 0;
}
