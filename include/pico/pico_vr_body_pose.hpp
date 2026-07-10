#pragma once

#include <array>

namespace kist {

// The device stream carries no usable timestamp — consumers needing
// freshness/dedup use the DataBuffer's receive time.
struct PicoVRBodyPose {
    // 24 SMPL joints, each [x, y, z, qx, qy, qz, qw]
    std::array<std::array<double, 7>, 24> joints{};
};

} // namespace kist
