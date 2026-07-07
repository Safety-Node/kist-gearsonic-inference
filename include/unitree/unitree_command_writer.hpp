#pragma once

#include "common/data_buffer.hpp"
#include "control/motor_command.hpp"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>

#include <atomic>
#include <thread>

namespace kist {

// 500Hz LowCmd writer (gear_sonic LowCommandWriter): reads the latest
// MotorCommand from the control loop's buffer, packs it into a LowCmd_
// with CRC32, and publishes on rt/lowcmd.
//
// SAFETY: this is the only component that makes the robot move.
//  - start() requires UnitreeStateReader to be running (shared DDS factory,
//    and mode_machine is echoed from LowState).
//  - stop() publishes a damping-only command (kp=0, kd=8) before exiting,
//    as does send_damping() for emergency paths.
class UnitreeCommandWriter {
public:
    static UnitreeCommandWriter& instance();
    bool start(const DataBuffer<MotorCommand>* source);
    void stop();

    // Immediately publish a zero-torque damping command (safe shutdown).
    void send_damping();

private:
    UnitreeCommandWriter() = default;

    void loop();
    void publish(const MotorCommand& cmd);

    using SdkLowCmd = unitree_hg::msg::dds_::LowCmd_;
    unitree::robot::ChannelPublisherPtr<SdkLowCmd> publisher_;

    const DataBuffer<MotorCommand>* source_{nullptr};

    std::thread       loop_thread_;
    std::atomic<bool> stop_{false};
};

} // namespace kist
