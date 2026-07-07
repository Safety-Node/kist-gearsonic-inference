#pragma once

#include <array>
#include <cmath>

// Quaternion / rotation math ported from gear_sonic math_utils.hpp.
// All quaternions are wxyz (scalar-first). Double precision only — the
// original's float variants exist for SDK compatibility we don't need.

namespace kist {

// ─── vectors ──────────────────────────────────────────────────────────────────

inline std::array<double, 3> normalize_vector(const std::array<double, 3>& v,
                                              double eps = 1e-12) {
    double norm = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    norm = std::max(norm, eps);
    return {v[0]/norm, v[1]/norm, v[2]/norm};
}

// ─── quaternion algebra ───────────────────────────────────────────────────────

inline std::array<double, 4> quat_unit(const std::array<double, 4>& q,
                                       double eps = 1e-12) {
    double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    norm = std::max(norm, eps);
    return {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
}

inline std::array<double, 4> quat_conjugate(const std::array<double, 4>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

inline std::array<double, 4> quat_mul(const std::array<double, 4>& a,
                                      const std::array<double, 4>& b) {
    double w1 = a[0], x1 = a[1], y1 = a[2], z1 = a[3];
    double w2 = b[0], x2 = b[1], y2 = b[2], z2 = b[3];

    double ww = (z1 + x1) * (x2 + y2);
    double yy = (w1 - y1) * (w2 + z2);
    double zz = (w1 + y1) * (w2 - z2);
    double xx = ww + yy + zz;
    double qq = 0.5 * (xx + (z1 - x1) * (x2 - y2));

    return {
        qq - ww + (z1 - y1) * (y2 - z2),
        qq - xx + (x1 + w1) * (x2 + w2),
        qq - yy + (w1 - x1) * (y2 + z2),
        qq - zz + (z1 + y1) * (w2 - x2)
    };
}

// Rotate a 3D vector by a quaternion
inline std::array<double, 3> quat_rotate(const std::array<double, 4>& q,
                                         const std::array<double, 3>& v) {
    double qw = q[0];
    std::array<double, 3> qv = {q[1], q[2], q[3]};

    double scale_a = 2.0 * qw * qw - 1.0;
    std::array<double, 3> a = {v[0]*scale_a, v[1]*scale_a, v[2]*scale_a};

    std::array<double, 3> cross = {qv[1]*v[2] - qv[2]*v[1],
                                   qv[2]*v[0] - qv[0]*v[2],
                                   qv[0]*v[1] - qv[1]*v[0]};
    std::array<double, 3> b = {cross[0]*qw*2.0, cross[1]*qw*2.0, cross[2]*qw*2.0};

    double dot = qv[0]*v[0] + qv[1]*v[1] + qv[2]*v[2];
    std::array<double, 3> c = {qv[0]*dot*2.0, qv[1]*dot*2.0, qv[2]*dot*2.0};

    return {a[0]+b[0]+c[0], a[1]+b[1]+c[1], a[2]+b[2]+c[2]};
}

// ─── heading (yaw-only) helpers ───────────────────────────────────────────────

inline std::array<double, 4> quat_from_angle_axis(double angle,
                                                  const std::array<double, 3>& axis) {
    double theta = angle / 2.0;
    auto n = normalize_vector(axis);
    std::array<double, 4> q = {std::cos(theta),
                               n[0]*std::sin(theta),
                               n[1]*std::sin(theta),
                               n[2]*std::sin(theta)};
    return quat_unit(q);
}

// Heading angle: yaw of the body-frame +x axis in world frame
inline double calc_heading(const std::array<double, 4>& q) {
    auto dir = quat_rotate(q, {1.0, 0.0, 0.0});
    return std::atan2(dir[1], dir[0]);
}

inline std::array<double, 4> calc_heading_quat(const std::array<double, 4>& q) {
    return quat_from_angle_axis(calc_heading(q), {0.0, 0.0, 1.0});
}

inline std::array<double, 4> calc_heading_quat_inv(const std::array<double, 4>& q) {
    return quat_from_angle_axis(-calc_heading(q), {0.0, 0.0, 1.0});
}

inline std::array<double, 4> euler_z_to_quat(double angle_rad) {
    return quat_from_angle_axis(angle_rad, {0.0, 0.0, 1.0});
}

// ─── conversions ──────────────────────────────────────────────────────────────

inline std::array<std::array<double, 3>, 3>
quat_to_rotation_matrix(const std::array<double, 4>& quat) {
    auto q = quat_unit(quat);
    double w = q[0], x = q[1], y = q[2], z = q[3];

    return {{
        {1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)},
        {2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)},
        {2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)}
    }};
}

// Gravity direction in the body frame: rotate world (0,0,-1) by conj(q)
inline std::array<double, 3> gravity_in_body_frame(const std::array<double, 4>& base_quat) {
    return quat_rotate(quat_conjugate(base_quat), {0.0, 0.0, -1.0});
}

// ─── interpolation ────────────────────────────────────────────────────────────

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
        return quat_unit(r);
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
