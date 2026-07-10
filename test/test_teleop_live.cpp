// Live TeleopTracker test with the PICO headset (no robot needed):
// start body tracking, calibrate on Enter (operator holds the reference
// pose: arms L-shaped, palms inward), then stream the 3-point output.

#include "pico/pico_vr_reader.hpp"
#include "teleop/teleop_tracker.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <thread>

using namespace kist;

static std::atomic<bool> g_stop{false};

int main() {
    std::signal(SIGINT, [](int) { g_stop = true; });

    if (!PicoVRReader::instance().start())
        return 1;
    TeleopTracker::instance().start();

    std::cout << "Toggle teleop: hold the reference pose (upper arms down, forearms\n"
                 "90° forward, palms inward, look straight ahead) and squeeze BOTH\n"
                 "triggers for 1 second. Same gesture again turns teleop off.\n";

    while (!g_stop) {
        auto v = TeleopTracker::instance().vr3point_buf.GetDataWithTime();
        if (!v.HasData()) {
            std::printf("vr3point: <empty> (body tracking absent or not calibrated)\n");
        } else {
            const auto& p = v.data->position;
            std::printf("L [%6.3f %6.3f %6.3f]  R [%6.3f %6.3f %6.3f]  "
                        "neck [%6.3f %6.3f %6.3f]  age %.0fms\n",
                        p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8],
                        v.GetAgeMs());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    TeleopTracker::instance().stop();
    PicoVRReader::instance().stop();
    return 0;
}
