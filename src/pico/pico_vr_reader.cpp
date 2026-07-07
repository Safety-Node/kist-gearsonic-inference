#include "pico/pico_vr_reader.hpp"

#include <PXREARobotSDK.h>
#include <nlohmann/json.hpp>

#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

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
                PicoVRBodyPose pose;
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
        PicoVRController ctrl;
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

// ─── RoboticsService daemon management ────────────────────────────────────────
//
// The PXREA SDK is a gRPC client of the XRoboToolkit PC service
// (RoboticsServiceProcess) listening on 127.0.0.1:60061. start() launches the
// daemon if it isn't already running (ensure-running).
//
// Lifetime semantics:
//  - stop() / process exit does NOT kill the daemon: the headset session
//    stays connected and the next start() reattaches instantly.
//  - The daemon lives as long as whatever spawned it: launched inside a
//    Docker container it dies with that container (use docker/run.sh's
//    persistent container to keep it across shell sessions); launched on
//    the host it survives until killed manually.
//  - Only one instance can bind the port. With host networking a daemon
//    on the host and one in a container are the same instance to us —
//    the port probe below treats "something listens on 60061" as running.

static constexpr const char* kDaemonPath =
    "/opt/apps/roboticsservice/RoboticsServiceProcess";
static constexpr const char* kDaemonLibPath =
    "/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib";
static constexpr const char* kDaemonLogPath = "/tmp/roboticsservice.log";
static constexpr uint16_t    kDaemonPort    = 60061;

static bool daemon_port_open() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(kDaemonPort);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    // Blocking connect to localhost: refused/accepted both return immediately.
    bool open = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    close(fd);
    return open;
}

static bool spawn_daemon() {
    if (access(kDaemonPath, X_OK) != 0) {
        std::cerr << "[PicoVRReader] daemon binary not found: " << kDaemonPath
                  << " (mount /opt/apps/roboticsservice into the container)\n";
        return false;
    }

    // Double-fork so the daemon is fully detached from this process
    // (survives our exit, no zombie to reap).
    pid_t pid = fork();
    if (pid < 0)
        return false;
    if (pid == 0) {
        setsid();
        if (fork() != 0)
            _exit(0);

        // The deb ships its libraries without rpath
        setenv("LD_LIBRARY_PATH", kDaemonLibPath, 1);

        int devnull = open("/dev/null", O_RDWR);
        int logfd   = open(kDaemonLogPath, O_CREAT | O_WRONLY | O_APPEND, 0644);
        dup2(devnull, 0);
        dup2(logfd >= 0 ? logfd : devnull, 1);
        dup2(logfd >= 0 ? logfd : devnull, 2);

        execl(kDaemonPath, kDaemonPath, static_cast<char*>(nullptr));
        _exit(127);
    }
    waitpid(pid, nullptr, 0);  // reap the intermediate child
    return true;
}

// ─── PicoVRReader ─────────────────────────────────────────────────────────────

PicoVRReader& PicoVRReader::instance() {
    static PicoVRReader inst;
    return inst;
}

void PicoVRReader::start() {
    if (!daemon_port_open()) {
        std::cout << "[PicoVRReader] RoboticsService not running, launching it...\n";
        if (spawn_daemon()) {
            for (int i = 0; i < 50 && !daemon_port_open(); ++i)  // up to 10 s
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (!daemon_port_open()) {
            std::cerr << "[PicoVRReader] RoboticsService failed to start"
                      << " (see " << kDaemonLogPath << ")\n";
            return;
        }
        std::cout << "[PicoVRReader] RoboticsService launched\n";
    }

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
    constexpr double stale_ms = 75.0;  // 5 frames at 70Hz (worst case, low battery)

    while (!stop_watchdog_) {
        std::this_thread::sleep_for(14ms);

        auto ts = body_buf.GetDataWithTime();
        if (ts.HasData() && ts.GetAgeMs() > stale_ms) {
            std::cerr << "[PicoVRReader] stale — disconnected\n";
            connected = false;
            body_buf.Clear();
            ctrl_buf.Clear();
        }
    }
}

void PicoVRReader::on_body_update(const PicoVRBodyPose& pose) {
    connected = true;
    body_buf.SetData(pose);
}

void PicoVRReader::on_controller_update(const PicoVRController& ctrl) {
    ctrl_buf.SetData(ctrl);
}

} // namespace kist
