#include "teleop/g1_arm_fk.hpp"
#include "common/math_utils.hpp"

namespace kist {

namespace {

// One revolute joint: fixed origin (translation + rpy rotation), rotation
// axis in the local frame, and its index in the MuJoCo/DDS joint order.
struct Link {
    std::array<double, 3> xyz;
    std::array<double, 3> rpy;
    std::array<double, 3> axis;
    int q_idx;
};

// URDF rpy -> quaternion: R = Rz(y) * Ry(p) * Rx(r)
std::array<double, 4> rpy_to_quat(const std::array<double, 3>& rpy) {
    return quat_mul(quat_from_angle_axis(rpy[2], {0.0, 0.0, 1.0}),
           quat_mul(quat_from_angle_axis(rpy[1], {0.0, 1.0, 0.0}),
                    quat_from_angle_axis(rpy[0], {1.0, 0.0, 0.0})));
}

// pelvis -> {left,right}_wrist_yaw_link chains, g1_29dof_with_hand.urdf
const std::array<Link, 10> kLeftChain = {{
    {{0.0, 0.0, 0.0},              {0.0, 0.0, 0.0},                    {0.0, 0.0, 1.0}, 12},  // waist_yaw
    {{-0.0039635, 0.0, 0.035},     {0.0, 0.0, 0.0},                    {1.0, 0.0, 0.0}, 13},  // waist_roll
    {{0.0, 0.0, 0.019},            {0.0, 0.0, 0.0},                    {0.0, 1.0, 0.0}, 14},  // waist_pitch
    {{0.0039563, 0.10022, 0.23778},{0.27931, 5.4949e-05, -0.00019159}, {0.0, 1.0, 0.0}, 15},  // shoulder_pitch
    {{0.0, 0.038, -0.013831},      {-0.27925, 0.0, 0.0},               {1.0, 0.0, 0.0}, 16},  // shoulder_roll
    {{0.0, 0.00624, -0.1032},      {0.0, 0.0, 0.0},                    {0.0, 0.0, 1.0}, 17},  // shoulder_yaw
    {{0.015783, 0.0, -0.080518},   {0.0, 0.0, 0.0},                    {0.0, 1.0, 0.0}, 18},  // elbow
    {{0.100, 0.00188791, -0.010},  {0.0, 0.0, 0.0},                    {1.0, 0.0, 0.0}, 19},  // wrist_roll
    {{0.038, 0.0, 0.0},            {0.0, 0.0, 0.0},                    {0.0, 1.0, 0.0}, 20},  // wrist_pitch
    {{0.046, 0.0, 0.0},            {0.0, 0.0, 0.0},                    {0.0, 0.0, 1.0}, 21},  // wrist_yaw
}};

const std::array<Link, 10> kRightChain = {{
    {{0.0, 0.0, 0.0},              {0.0, 0.0, 0.0},                    {0.0, 0.0, 1.0}, 12},
    {{-0.0039635, 0.0, 0.035},     {0.0, 0.0, 0.0},                    {1.0, 0.0, 0.0}, 13},
    {{0.0, 0.0, 0.019},            {0.0, 0.0, 0.0},                    {0.0, 1.0, 0.0}, 14},
    {{0.0039563, -0.10021, 0.23778},{-0.27931, 5.4949e-05, 0.00019159},{0.0, 1.0, 0.0}, 22},
    {{0.0, -0.038, -0.013831},     {0.27925, 0.0, 0.0},                {1.0, 0.0, 0.0}, 23},
    {{0.0, -0.00624, -0.1032},     {0.0, 0.0, 0.0},                    {0.0, 0.0, 1.0}, 24},
    {{0.015783, 0.0, -0.080518},   {0.0, 0.0, 0.0},                    {0.0, 1.0, 0.0}, 25},
    {{0.100, -0.00188791, -0.010}, {0.0, 0.0, 0.0},                    {1.0, 0.0, 0.0}, 26},
    {{0.038, 0.0, 0.0},            {0.0, 0.0, 0.0},                    {0.0, 1.0, 0.0}, 27},
    {{0.046, 0.0, 0.0},            {0.0, 0.0, 0.0},                    {0.0, 0.0, 1.0}, 28},
}};

// Key-frame local offset in the wrist_yaw_link frame
constexpr std::array<double, 3> kLeftOffset  = {0.18, -0.025, 0.0};
constexpr std::array<double, 3> kRightOffset = {0.18, 0.025, 0.0};

} // namespace

G1WristPose g1_wrist_fk(const std::array<double, 29>& q, bool left) {
    const auto& chain  = left ? kLeftChain : kRightChain;
    const auto& offset = left ? kLeftOffset : kRightOffset;

    std::array<double, 3> p{0.0, 0.0, 0.0};
    std::array<double, 4> r{1.0, 0.0, 0.0, 0.0};

    for (const auto& link : chain) {
        auto d = quat_rotate(r, link.xyz);
        p = {p[0] + d[0], p[1] + d[1], p[2] + d[2]};
        r = quat_mul(r, rpy_to_quat(link.rpy));
        r = quat_mul(r, quat_from_angle_axis(q[link.q_idx], link.axis));
    }

    auto o = quat_rotate(r, offset);
    return {{p[0] + o[0], p[1] + o[1], p[2] + o[2]}, r};
}

} // namespace kist
