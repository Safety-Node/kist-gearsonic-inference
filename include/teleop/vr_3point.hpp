#pragma once

#include <array>

namespace kist {

// Calibrated 3-point upper-body target: [L wrist, R wrist, neck],
// relative to the operator's root (pelvis). Layout matches the encoder's
// vr_3point observation slots: position 3x[x,y,z], orientation 3x[w,x,y,z].
struct VR3Point {
    std::array<double, 9>  position{};
    std::array<double, 12> orientation{};
};

} // namespace kist
