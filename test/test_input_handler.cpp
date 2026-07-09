#include "pico/pico_vr_reader.hpp"
#include "motion/input_handler.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

int main() {
    auto& reader  = kist::PicoVRReader::instance();
    auto& handler = kist::InputHandler::instance();

    if (!reader.start())
        return 1;

    handler.start();

    std::cout << "Waiting for input... (Ctrl+C to exit)\n";

    while (true) {
        auto ms = handler.movement_buf.GetData();

        if (ms) {
            std::cout << std::fixed << std::setprecision(3)
                      << "[Input]"
                      << "  mode="     << ms->locomotion_mode
                      << "  move=("    << ms->movement_direction[0] << ", "
                                       << ms->movement_direction[1] << ", "
                                       << ms->movement_direction[2] << ")"
                      << "  face=("    << ms->facing_direction[0] << ", "
                                       << ms->facing_direction[1] << ", "
                                       << ms->facing_direction[2] << ")"
                      << "  speed="    << ms->movement_speed
                      << "\n";
        } else {
            std::cout << "[Input] waiting...\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    handler.stop();
    reader.stop();
    return 0;
}
