// Release the robot's built-in motion control service so it stops
// publishing on rt/lowcmd (SDK MotionSwitcher API, same procedure as the
// unitree_sdk2 humanoid low-level example).
//
// Run this ONCE before test_control_live, then verify with lowcmd_monitor
// (should report zero traffic). The robot goes limp when released —
// keep it suspended.

#include "common/config.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(config_path);
    auto cfg = kist::Config::instance().root()["unitree"];

    unitree::robot::ChannelFactory::Instance()->Init(
        cfg["domain_id"].as<int>(), cfg["network_interface"].as<std::string>());

    unitree::robot::b2::MotionSwitcherClient msc;
    msc.SetTimeout(5.0f);
    msc.Init();

    for (int attempt = 0; attempt < 5; ++attempt) {
        std::string form, name;
        int32_t ret = msc.CheckMode(form, name);
        if (ret != 0) {
            std::cerr << "CheckMode failed (error " << ret << ")\n";
            return 1;
        }
        if (name.empty()) {
            std::cout << "Motion control service is released. Safe to run our controller.\n";
            return 0;
        }
        std::cout << "Active motion service: \"" << name << "\" (form: " << form
                  << ") — releasing...\n";
        ret = msc.ReleaseMode();
        std::cout << (ret == 0 ? "ReleaseMode succeeded" : "ReleaseMode failed") << "\n";
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    std::cerr << "Could not release the motion service after 5 attempts.\n";
    return 1;
}
