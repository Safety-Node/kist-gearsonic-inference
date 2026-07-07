#include "unitree/unitree_state_reader.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <chrono>
#include <iostream>

namespace kist {

static const std::string kLowStateTopic = "rt/lowstate";
static const std::string kImuTorsoTopic = "rt/secondary_imu";

// ─── SDK → kist conversion helpers ────────────────────────────────────────────

static IMU convert(const unitree_hg::msg::dds_::IMUState_& src) {
    IMU out;
    const auto& q = src.quaternion();
    const auto& g = src.gyroscope();
    const auto& a = src.accelerometer();
    out.quaternion    = { q[0], q[1], q[2], q[3] };
    out.gyroscope     = { g[0], g[1], g[2] };
    out.accelerometer = { a[0], a[1], a[2] };
    return out;
}

static UnitreeState convert(const unitree_hg::msg::dds_::LowState_& src) {
    UnitreeState out;
    out.tick = src.tick();
    out.mode_machine = src.mode_machine();
    out.imu_pelvis = convert(src.imu_state());
    const auto& motors = src.motor_state();
    for (int i = 0; i < kNumMotors; ++i) {
        out.motors[i].q   = motors[i].q();
        out.motors[i].dq  = motors[i].dq();
        out.motors[i].tau = motors[i].tau_est();
    }
    return out;
}

// ─── UnitreeStateReader ───────────────────────────────────────────────────────

UnitreeStateReader& UnitreeStateReader::instance() {
    static UnitreeStateReader inst;
    return inst;
}

void UnitreeStateReader::start(int domain_id, const std::string& network_interface) {
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);

    lowstate_sub_.reset(new LowStateSub(kLowStateTopic));
    lowstate_sub_->InitChannel(
        [this](const void* msg) { on_lowstate(msg); }, 1);

    imu_torso_sub_.reset(new IMUStateSub(kImuTorsoTopic));
    imu_torso_sub_->InitChannel(
        [this](const void* msg) { on_imu_torso(msg); }, 1);

    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&UnitreeStateReader::watchdog_loop, this);
    std::cout << "[UnitreeStateReader] started on domain=" << domain_id
              << " interface=" << network_interface << "\n";
}

void UnitreeStateReader::stop() {
    connected = false;
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    lowstate_sub_.reset();
    imu_torso_sub_.reset();
}

void UnitreeStateReader::on_lowstate(const void* message) {
    connected = true;
    unitree_state_buf.SetData(convert(*static_cast<const SdkLowState*>(message)));
}

void UnitreeStateReader::on_imu_torso(const void* message) {
    imu_torso_buf.SetData(convert(*static_cast<const SdkIMUState*>(message)));
}

void UnitreeStateReader::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 10.0;  // 5 frames at 500Hz

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(2ms);

        auto ts = unitree_state_buf.GetDataWithTime();
        if (ts.HasData() && ts.GetAgeMs() > stale_ms) {
            std::cerr << "[UnitreeStateReader] stale — disconnected\n";
            connected = false;
            unitree_state_buf.Clear();
            imu_torso_buf.Clear();
        }
    }
}

} // namespace kist
