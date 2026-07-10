#include "teleop/teleop_tracker.hpp"
#include "common/math_utils.hpp"
#include "pico/pico_vr_reader.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

namespace kist {

namespace {

// Unity (X-right, Y-up, Z-forward) -> robot (X-forward, Y-left, Z-up):
// position [x,y,z] -> [-x, z, y]. The same permutation as a rotation
// matrix Q = [[-1,0,0],[0,0,1],[0,1,0]] is proper (det=+1), a 180° turn
// about (0,1,1)/sqrt(2) — so orientations map by quaternion conjugation.
const std::array<double, 4> kUnityToRobot = {0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5)};

// Extrinsic xyz Euler (scipy "xyz") -> quaternion: R = Rz(c)·Ry(b)·Rx(a)
std::array<double, 4> euler_xyz_extrinsic(double a, double b, double c) {
    return quat_mul(quat_from_angle_axis(c, {0.0, 0.0, 1.0}),
           quat_mul(quat_from_angle_axis(b, {0.0, 1.0, 0.0}),
                    quat_from_angle_axis(a, {1.0, 0.0, 0.0})));
}

constexpr double kDeg = M_PI / 180.0;

// Per-keypoint frame corrections (gear_sonic OFFSETS), post-multiplied:
// [root, L wrist, R wrist, neck]
const std::array<std::array<double, 4>, 4> kOffsets = {
    euler_xyz_extrinsic(0.0,       0.0, -90.0 * kDeg),   // root: yaw -90°
    euler_xyz_extrinsic(90.0 * kDeg,  0.0, 0.0),          // L wrist: roll +90°
    euler_xyz_extrinsic(-90.0 * kDeg, 0.0, 180.0 * kDeg), // R wrist: roll -90°, yaw 180°
    euler_xyz_extrinsic(0.0,       0.0, -90.0 * kDeg),   // neck: yaw -90°
};

// G1 key-frame poses at the all-zero joint pose (root frame): FK through
// the URDF origin chain to {left,right}_wrist_yaw_link, plus the local
// offsets [0.18, ∓0.025, 0] from gear_sonic force.yaml. The calibration
// reference the operator's zero pose is matched against.
constexpr std::array<double, 3> kG1LWristPos = {0.3797694914, 0.1236272185, 0.0952214409};
constexpr std::array<double, 3> kG1RWristPos = {0.3797694914, -0.1236172185, 0.0952214409};
const std::array<double, 4> kG1LWristQuat = {0.9999999946, 0.0000300026, 0.0000274716, -0.0000957958};
const std::array<double, 4> kG1RWristQuat = {0.9999999946, -0.0000300026, 0.0000274716, 0.0000957958};

// Neck position kinematic chain (gear_sonic ThreePointPose)
constexpr double kTorsoLinkOffsetZ = 0.05;  // m, root -> torso_link
constexpr double kNeckLinkLength   = 0.35;  // m, torso_link -> neck along neck local Z

} // namespace

TeleopTracker& TeleopTracker::instance() {
    static TeleopTracker inst;
    return inst;
}

void TeleopTracker::start() {
    stop_        = false;
    loop_thread_ = std::thread(&TeleopTracker::loop, this);
    std::cout << "[TeleopTracker] started (50 Hz)\n";
}

void TeleopTracker::stop() {
    stop_ = true;
    if (loop_thread_.joinable())
        loop_thread_.join();
    vr3point_buf.Clear();
}

// ─── loop ─────────────────────────────────────────────────────────────────────

void TeleopTracker::loop() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(static_cast<int>(kLoopDt * 1e6));

    while (!stop_) {
        auto t0 = clock::now();

        if (reset_request_.exchange(false)) {
            have_neck_calib_ = false;
            calibrated_      = false;
            vr3point_buf.Clear();
            std::cout << "[TeleopTracker] calibration reset\n";
        }

        check_calibration_gesture();

        auto body = PicoVRReader::instance().body_buf.GetData();
        if (!body) {
            // body tracking absent/stale — no teleop target
            vr3point_buf.Clear();
        } else if (body->timestamp_ns != last_body_stamp_ns_) {
            last_body_stamp_ns_ = body->timestamp_ns;
            process(*body);
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

// Both triggers squeezed >0.8 for 1s — a toggle: not calibrated ->
// calibrate (teleop on); calibrated -> reset (back to g1). Must be
// released before it can fire again; re-entry recalibrates fresh.
void TeleopTracker::check_calibration_gesture() {
    auto ctrl = PicoVRReader::instance().ctrl_buf.GetData();
    bool held = ctrl && ctrl->left_trigger > kCalibTrigger &&
                        ctrl->right_trigger > kCalibTrigger;
    if (!held) {
        calib_hold_ticks_     = 0;
        calib_gesture_latched_ = false;
        return;
    }
    if (calib_gesture_latched_)
        return;
    if (++calib_hold_ticks_ >= kCalibHoldTicks) {
        calib_gesture_latched_ = true;
        if (calibrated_) {
            reset_calibration();
            std::cout << "[TeleopTracker] gesture: teleop off -> g1\n";
        } else {
            request_calibration();
            std::cout << "[TeleopTracker] gesture: calibrating -> teleop on\n";
        }
    }
}

void TeleopTracker::process(const PicoVRBodyPose& body) {
    Raw3Point raw = extract_raw(body);

    if (calibrate_request_.exchange(false))
        capture_calibration(raw);

    if (!calibrated_)
        return;

    VR3Point out;
    apply_calibration(raw, out);
    vr3point_buf.SetData(out);
}

// ─── SMPL -> raw root-relative 3-point ────────────────────────────────────────

TeleopTracker::Raw3Point TeleopTracker::extract_raw(const PicoVRBodyPose& body) const {
    // keypoints: root/pelvis(0), L wrist(22), R wrist(23), neck(12) —
    // neck instead of head(15): rigidly coupled to the torso, stabler.
    static constexpr int kSmplIdx[4] = {0, 22, 23, 12};

    std::array<std::array<double, 3>, 4> pos;
    std::array<std::array<double, 4>, 4> quat;
    for (int k = 0; k < 4; ++k) {
        const auto& j = body.joints[kSmplIdx[k]];  // [x,y,z, qx,qy,qz,qw]
        pos[k] = {-j[0], j[2], j[1]};
        std::array<double, 4> q = {j[6], j[3], j[4], j[5]};
        q = quat_mul(kUnityToRobot, quat_mul(q, quat_conjugate(kUnityToRobot)));
        quat[k] = quat_mul(q, kOffsets[k]);
    }

    Raw3Point raw;
    auto root_inv = quat_conjugate(quat[0]);
    for (int k = 1; k < 4; ++k) {
        std::array<double, 3> d = {pos[k][0] - pos[0][0],
                                   pos[k][1] - pos[0][1],
                                   pos[k][2] - pos[0][2]};
        raw.pos[k - 1]  = quat_rotate(root_inv, d);
        raw.quat[k - 1] = quat_mul(root_inv, quat[k]);
    }
    return raw;
}

// ─── calibration ──────────────────────────────────────────────────────────────

void TeleopTracker::capture_calibration(const Raw3Point& raw) {
    // Neck orientation: keep an existing capture (recalibration must not
    // jump from SMPL noise), otherwise store inv(current neck).
    if (!have_neck_calib_) {
        neck_quat_inv_   = quat_conjugate(raw.quat[2]);
        have_neck_calib_ = true;
    }

    // Wrist offsets against the G1 zero-pose FK reference,
    // in the neck-corrected frame.
    auto lw_pos = quat_rotate(neck_quat_inv_, raw.pos[0]);
    auto rw_pos = quat_rotate(neck_quat_inv_, raw.pos[1]);
    auto lw_rot = quat_mul(neck_quat_inv_, raw.quat[0]);
    auto rw_rot = quat_mul(neck_quat_inv_, raw.quat[1]);

    for (int i = 0; i < 3; ++i) {
        lwrist_pos_offset_[i] = lw_pos[i] - kG1LWristPos[i];
        rwrist_pos_offset_[i] = rw_pos[i] - kG1RWristPos[i];
    }
    lwrist_rot_offset_ = quat_mul(kG1LWristQuat, quat_conjugate(lw_rot));
    rwrist_rot_offset_ = quat_mul(kG1RWristQuat, quat_conjugate(rw_rot));

    calibrated_ = true;
    std::cout << "[TeleopTracker] calibration captured"
              << "  L offset [" << lwrist_pos_offset_[0] << ", " << lwrist_pos_offset_[1]
              << ", " << lwrist_pos_offset_[2] << "]"
              << "  R offset [" << rwrist_pos_offset_[0] << ", " << rwrist_pos_offset_[1]
              << ", " << rwrist_pos_offset_[2] << "]\n";
}

void TeleopTracker::apply_calibration(const Raw3Point& raw, VR3Point& out) const {
    // Neck: calibrated = inv(initial) * current
    auto neck_q = quat_mul(neck_quat_inv_, raw.quat[2]);

    // Wrists: position rotated by neck inverse minus captured offset;
    // orientation = rot_offset * (neck_inv * current)
    auto lw_pos = quat_rotate(neck_quat_inv_, raw.pos[0]);
    auto rw_pos = quat_rotate(neck_quat_inv_, raw.pos[1]);
    auto lw_q = quat_mul(lwrist_rot_offset_, quat_mul(neck_quat_inv_, raw.quat[0]));
    auto rw_q = quat_mul(rwrist_rot_offset_, quat_mul(neck_quat_inv_, raw.quat[1]));

    // Neck position is synthesized, not tracked: root -> torso_link (+Z),
    // then along the calibrated neck's local Z.
    auto neck_z = quat_rotate(neck_q, {0.0, 0.0, 1.0});

    for (int i = 0; i < 3; ++i) {
        out.position[i]     = lw_pos[i] - lwrist_pos_offset_[i];
        out.position[3 + i] = rw_pos[i] - rwrist_pos_offset_[i];
        out.position[6 + i] = (i == 2 ? kTorsoLinkOffsetZ : 0.0) + kNeckLinkLength * neck_z[i];
    }
    for (int i = 0; i < 4; ++i) {
        out.orientation[i]     = lw_q[i];
        out.orientation[4 + i] = rw_q[i];
        out.orientation[8 + i] = neck_q[i];
    }
}

} // namespace kist
