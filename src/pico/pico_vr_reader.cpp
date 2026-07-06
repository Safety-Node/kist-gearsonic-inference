#include "pico/pico_vr_reader.hpp"

#include <PXREARobotSDK.h>
#include <nlohmann/json.hpp>

#include <sstream>
#include <iostream>
#include <chrono>

using json = nlohmann::json;

namespace kist {

// ─── helpers ──────────────────────────────────────────────────────────────────

static std::array<double, 7> parse_pose(const std::string& s) {
    std::array<double, 7> result{};
    std::stringstream ss(s);
    std::string token;
    int i = 0;
    while (std::getline(ss, token, ',') && i < 7)
        result[i++] = std::stod(token);
    return result;
}

// ─── SDK callback ─────────────────────────────────────────────────────────────

static void pxrea_callback(void*, PXREAClientCallbackType type, int, void* user_data) {
    auto& reader = PicoVRReader::instance();

    if (type != PXREADeviceStateJson)
        return;

    auto& dsj = *static_cast<PXREADevStateJson*>(user_data);

    try {
        auto data = json::parse(dsj.stateJson);
        if (!data.contains("value"))
            return;

        auto value = json::parse(data["value"].get<std::string>());

        // ── Body tracking ──────────────────────────────────────────────────
        if (value.contains("Body")) {
            auto& body = value["Body"];
            if (body.contains("joints") && body["joints"].is_array()) {
                BodyPose pose;
                pose.timestamp_ns = body.value("timeStampNs", int64_t{0});

                auto& joints = body["joints"];
                int n = std::min(static_cast<int>(joints.size()), 24);
                for (int i = 0; i < n; i++) {
                    if (joints[i].contains("p"))
                        pose.joints[i] = parse_pose(joints[i]["p"].get<std::string>());
                }
                reader.on_body_update(pose);
            }
        }

        // ── Controller ────────────────────────────────────────────────────
        ControllerState ctrl;
        bool ctrl_updated = false;

        if (value["Controller"].contains("left")) {
            auto& l = value["Controller"]["left"];
            ctrl.left_axis[0] = l.value("axisX", 0.0);
            ctrl.left_axis[1] = l.value("axisY", 0.0);
            ctrl.left_trigger = l.value("trigger", 0.0);
            ctrl.left_grip    = l.value("grip", 0.0);
            ctrl.btn_x        = l.value("primaryButton", false);
            ctrl.btn_y        = l.value("secondaryButton", false);
            ctrl_updated = true;
        }
        if (value["Controller"].contains("right")) {
            auto& r = value["Controller"]["right"];
            ctrl.right_axis[0] = r.value("axisX", 0.0);
            ctrl.right_axis[1] = r.value("axisY", 0.0);
            ctrl.right_trigger = r.value("trigger", 0.0);
            ctrl.right_grip    = r.value("grip", 0.0);
            ctrl.btn_a         = r.value("primaryButton", false);
            ctrl.btn_b         = r.value("secondaryButton", false);
            ctrl_updated = true;
        }
        if (ctrl_updated)
            reader.on_controller_update(ctrl);

    } catch (const json::exception& e) {
        std::cerr << "[PicoVRReader] JSON parse error: " << e.what() << "\n";
    }
}

// ─── PicoVRReader ─────────────────────────────────────────────────────────────

PicoVRReader& PicoVRReader::instance() {
    static PicoVRReader inst;
    return inst;
}

void PicoVRReader::start() {
    if (PXREAInit(nullptr, pxrea_callback, PXREAFullMask) != 0) {
        std::cerr << "[PicoVRReader] PXREAInit failed\n";
        return;
    }
    connected = true;
    stop_watchdog_ = false;
    watchdog_thread_ = std::thread(&PicoVRReader::watchdog_loop, this);
    std::cout << "[PicoVRReader] started\n";
}

void PicoVRReader::stop() {
    connected = false;
    stop_watchdog_ = true;
    if (watchdog_thread_.joinable())
        watchdog_thread_.join();
    PXREADeinit();
}

void PicoVRReader::watchdog_loop() {
    using namespace std::chrono_literals;
    constexpr double stale_ms = 50.0;

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(10ms);

        auto ts = body_buf.GetDataWithTime();
        if (ts.HasData() && ts.GetAgeMs() > stale_ms) {
            std::cerr << "[PicoVRReader] stale — disconnected\n";
            connected = false;
            body_buf.Clear();
            ctrl_buf.Clear();
        }
    }
}

void PicoVRReader::on_body_update(const BodyPose& pose) {
    connected = true;
    body_buf.SetData(pose);
}

void PicoVRReader::on_controller_update(const ControllerState& ctrl) {
    ctrl_buf.SetData(ctrl);
}

} // namespace kist
