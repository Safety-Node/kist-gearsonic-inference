#pragma once

#include <array>
#include <cmath>

namespace kist {

// SLERP for unit quaternions (wxyz). t in [0, 1].
inline std::array<double, 4> quat_slerp(const std::array<double, 4>& a,
                                        const std::array<double, 4>& b,
                                        double t) {
    double dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    std::array<double, 4> bb = b;
    if (dot < 0.0) {
        for (auto& v : bb) v = -v;
        dot = -dot;
    }
    // Fall back to linear interpolation for very close quaternions.
    if (dot > 0.9995) {
        std::array<double, 4> r{
            a[0] + t * (bb[0] - a[0]),
            a[1] + t * (bb[1] - a[1]),
            a[2] + t * (bb[2] - a[2]),
            a[3] + t * (bb[3] - a[3])
        };
        double n = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
        return { r[0]/n, r[1]/n, r[2]/n, r[3]/n };
    }
    double theta_0 = std::acos(dot);
    double theta   = theta_0 * t;
    double sin_t0  = std::sin(theta_0);
    double s0      = std::cos(theta) - dot * std::sin(theta) / sin_t0;
    double s1      = std::sin(theta) / sin_t0;
    return {
        s0 * a[0] + s1 * bb[0],
        s0 * a[1] + s1 * bb[1],
        s0 * a[2] + s1 * bb[2],
        s0 * a[3] + s1 * bb[3]
    };
}

} // namespace kist
