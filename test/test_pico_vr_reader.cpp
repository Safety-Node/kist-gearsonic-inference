#include "pico/pico_vr_reader.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

int main() {
    auto& reader = kist::PicoVRReader::instance();

    if (!reader.init())
        return 1;

    std::cout << "Waiting for PICO VR data... (Ctrl+C to exit)\n";

    while (true) {
        auto body_ts = reader.body_buf.GetDataWithTime();
        auto ctrl    = reader.ctrl_buf.GetData();

        if (body_ts.HasData()) {
            double age_ms = body_ts.GetAgeMs();
            if (age_ms > 500.0) {
                std::cout << "[Body] stale (" << age_ms << "ms)\n";
            } else {
                std::cout << "[Body] age=" << age_ms << "ms\n";
                for (int i = 0; i < 24; i++) {
                    auto& j = body_ts.data->joints[i];
                    std::cout << "  joint[" << i << "] pos=("
                              << std::fixed << std::setprecision(6)
                              << j[0] << ", " << j[1] << ", " << j[2] << ")"
                              << "  quat=(" << j[3] << ", " << j[4] << ", " << j[5] << ", " << j[6] << ")\n";
                }
            }
        } else {
            std::cout << "[Body] waiting...\n";
        }

        if (ctrl) {
            std::cout << "[Ctrl] left_axis=(" << ctrl->left_axis[0] << ", " << ctrl->left_axis[1] << ")"
                      << "  right_axis=(" << ctrl->right_axis[0] << ", " << ctrl->right_axis[1] << ")"
                      << "  left_trigger=" << ctrl->left_trigger
                      << "  right_trigger=" << ctrl->right_trigger
                      << "  left_grip=" << ctrl->left_grip
                      << "  right_grip=" << ctrl->right_grip
                      << "  A=" << ctrl->btn_a << " B=" << ctrl->btn_b
                      << " X=" << ctrl->btn_x << " Y=" << ctrl->btn_y << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    reader.deinit();
    return 0;
}
