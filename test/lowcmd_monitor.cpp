// Diagnostic: count publishers on rt/lowcmd.
// Run while OUR controller is NOT running. Any traffic seen here means
// something else (the robot's built-in motion service) is still publishing
// motor commands — which fights our commands and causes trembling.

#include "common/config.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

static std::atomic<long> g_count{0};
static std::atomic<float> g_last_kp0{0};

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/config.yaml";
    kist::Config::instance().load(config_path);
    auto cfg = kist::Config::instance().root()["unitree"];

    unitree::robot::ChannelFactory::Instance()->Init(
        cfg["domain_id"].as<int>(), cfg["network_interface"].as<std::string>());

    auto sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>>("rt/lowcmd");
    sub->InitChannel([](const void* msg) {
        const auto& cmd = *static_cast<const unitree_hg::msg::dds_::LowCmd_*>(msg);
        ++g_count;
        g_last_kp0 = cmd.motor_cmd().at(0).kp();
    });

    std::cout << "Listening on rt/lowcmd for 5 seconds...\n";
    for (int s = 1; s <= 5; ++s) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "  t=" << s << "s  total msgs=" << g_count.load()
                  << "  last kp[0]=" << g_last_kp0.load() << "\n";
    }

    long n = g_count;
    if (n == 0) {
        std::cout << "\nNo lowcmd traffic: no competing publisher. (good)\n";
    } else {
        std::cout << "\n*** " << n / 5 << " msg/s on rt/lowcmd from ANOTHER publisher ***\n"
                  << "The built-in motion service is still commanding the motors.\n"
                  << "Release/stop it before running our controller.\n";
    }
    return 0;
}
