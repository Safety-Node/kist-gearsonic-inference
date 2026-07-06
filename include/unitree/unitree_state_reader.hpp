#pragma once

#include "common/data_buffer.hpp"
#include "unitree/imu.hpp"
#include "unitree/unitree_state.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace kist {

class UnitreeStateReader {
public:
    // ── lifecycle ──────────────────────────────────────────────
    static UnitreeStateReader& instance();
    void                       start(int domain_id, const std::string& network_interface);
    void                       stop();

    // ── state ──────────────────────────────────────────────────
    std::atomic<bool> connected{false};

    // ── data buffers (read from any thread) ────────────────────
    DataBuffer<UnitreeState> unitree_state_buf;
    DataBuffer<IMU>          imu_torso_buf;

private:
    UnitreeStateReader() = default;

    void on_lowstate(const void* message);
    void on_imu_torso(const void* message);
    void watchdog_loop();

    using SdkLowState = unitree_hg::msg::dds_::LowState_;
    using SdkIMUState = unitree_hg::msg::dds_::IMUState_;
    using LowStateSub = unitree::robot::ChannelSubscriber<SdkLowState>;
    using IMUStateSub = unitree::robot::ChannelSubscriber<SdkIMUState>;

    unitree::robot::ChannelSubscriberPtr<SdkLowState> lowstate_sub_;
    unitree::robot::ChannelSubscriberPtr<SdkIMUState> imu_torso_sub_;

    std::thread       watchdog_thread_;
    std::atomic<bool> stop_watchdog_{false};
};

} // namespace kist
