#pragma once

#include <array>
#include <cstdint>

namespace kist {

struct PicoVRBodyPose {
    // 24 SMPL joints, each [x, y, z, qx, qy, qz, qw]
    std::array<std::array<double, 7>, 24> joints{};
    int64_t timestamp_ns{0};
};

} // namespace kist
