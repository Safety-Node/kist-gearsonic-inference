#include "unitree/unitree_command_writer.hpp"
#include "common/thread_priority.hpp"
#include "unitree/unitree_state_reader.hpp"

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include <chrono>
#include <iostream>

namespace kist {

static const std::string kLowCmdTopic = "rt/lowcmd";
static constexpr auto    kPublishPeriod = std::chrono::microseconds(2000);  // 500Hz
static constexpr uint8_t kModePR = 0;  // series pitch/roll ankle mode

// Unitree SDK CRC (gear_sonic utils.hpp Crc32Core)
static uint32_t crc32_core(const uint32_t* ptr, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t xbit = 1u << 31;
        uint32_t data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (crc & 0x80000000) {
                crc <<= 1;
                crc ^= poly;
            } else {
                crc <<= 1;
            }
            if (data & xbit) crc ^= poly;
            xbit >>= 1;
        }
    }
    return crc;
}

UnitreeCommandWriter& UnitreeCommandWriter::instance() {
    static UnitreeCommandWriter inst;
    return inst;
}

// Make sure the robot's built-in motion service is not publishing on
// rt/lowcmd — two publishers alternating commands makes the motors judder.
// Same procedure as the unitree_sdk2 humanoid low-level example.
static bool ensure_exclusive_lowcmd() {
    unitree::robot::b2::MotionSwitcherClient msc;
    msc.SetTimeout(2.0f);
    msc.Init();

    std::string form, name;
    if (msc.CheckMode(form, name) != 0) {
        // No motion_switcher service (e.g., simulator): nothing to release.
        std::cout << "[UnitreeCommandWriter] motion_switcher not reachable — assuming no built-in publisher\n";
        return true;
    }
    if (name.empty())
        return true;  // already released

    std::cout << "[UnitreeCommandWriter] built-in motion service active (\"" << name
              << "\") — releasing (robot will go limp)...\n";
    for (int attempt = 0; attempt < 5; ++attempt) {
        msc.ReleaseMode();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (msc.CheckMode(form, name) == 0 && name.empty()) {
            std::cout << "[UnitreeCommandWriter] motion service released\n";
            return true;
        }
    }
    std::cerr << "[UnitreeCommandWriter] FAILED to release the built-in motion service — refusing to publish\n";
    return false;
}

bool UnitreeCommandWriter::start(const DataBuffer<MotorCommand>* source) {
    if (!source) {
        std::cerr << "[UnitreeCommandWriter] null command source\n";
        return false;
    }
    source_ = source;

    if (!ensure_exclusive_lowcmd())
        return false;

    // ChannelFactory must already be initialized by UnitreeStateReader::start()
    publisher_ = std::make_shared<unitree::robot::ChannelPublisher<SdkLowCmd>>(kLowCmdTopic);
    publisher_->InitChannel();

    stop_        = false;
    loop_thread_ = std::thread(&UnitreeCommandWriter::loop, this);
    try_realtime_priority(loop_thread_, 90, "UnitreeCommandWriter");
    std::cout << "[UnitreeCommandWriter] started (500 Hz, publishing to " << kLowCmdTopic << ")\n";
    return true;
}

void UnitreeCommandWriter::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
    send_damping();
    std::cout << "[UnitreeCommandWriter] stopped (damping command sent)\n";
}

void UnitreeCommandWriter::send_damping() {
    if (!publisher_)
        return;
    MotorCommand damping;
    for (int i = 0; i < 29; ++i) {
        damping.kp[i] = 0.0f;
        damping.kd[i] = 8.0f;
    }
    // publish a short burst so it isn't lost
    for (int n = 0; n < 5; ++n) {
        publish(damping);
        std::this_thread::sleep_for(kPublishPeriod);
    }
}

void UnitreeCommandWriter::publish(const MotorCommand& cmd) {
    SdkLowCmd msg;
    msg.mode_pr() = kModePR;

    auto st = UnitreeStateReader::instance().unitree_state_buf.GetData();
    msg.mode_machine() = st ? st->mode_machine : 0;

    for (int i = 0; i < 29; ++i) {
        msg.motor_cmd().at(i).mode() = 1;  // enable
        msg.motor_cmd().at(i).q()    = cmd.q_target[i];
        msg.motor_cmd().at(i).dq()   = cmd.dq_target[i];
        msg.motor_cmd().at(i).kp()   = cmd.kp[i];
        msg.motor_cmd().at(i).kd()   = cmd.kd[i];
        msg.motor_cmd().at(i).tau()  = cmd.tau_ff[i];
    }

    msg.crc() = crc32_core(reinterpret_cast<uint32_t*>(&msg), (sizeof(msg) >> 2) - 1);
    publisher_->Write(msg);
}

void UnitreeCommandWriter::loop() {
    using clock = std::chrono::steady_clock;

    // Period jitter instrumentation: firmware expects a steady lowcmd
    // stream; large gaps make the motors judder. Log max gap once per second.
    auto last_tick = clock::now();
    long max_gap_us = 0;
    int  ticks = 0;

    while (!stop_) {
        auto t0 = clock::now();

        long gap = std::chrono::duration_cast<std::chrono::microseconds>(t0 - last_tick).count();
        last_tick = t0;
        max_gap_us = std::max(max_gap_us, gap);
        if (++ticks >= 500) {
            if (max_gap_us > 6000)  // 3x nominal period
                std::cerr << "[UnitreeCommandWriter] JITTER: max gap " << max_gap_us << "us in last 1s\n";
            ticks = 0;
            max_gap_us = 0;
        }

        auto cmd = source_->GetData();
        if (cmd)
            publish(*cmd);

        std::this_thread::sleep_until(t0 + kPublishPeriod);
    }
}

} // namespace kist
