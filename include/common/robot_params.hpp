#pragma once

#include <array>

namespace kist {

// Unitree G1 29-DOF robot constants, ported from gear_sonic
// policy_parameters.hpp. All per-joint arrays are in MuJoCo/URDF order
// (== DDS motor order); the policy's IsaacLab-order output is remapped
// through isaaclab_to_mujoco before these are applied.

// ─── actuator constants ───────────────────────────────────────────────────────

// Motor armature (used for PD gain computation)
constexpr double kArmature5020   = 0.003609725;
constexpr double kArmature7520_14 = 0.010177520;
constexpr double kArmature7520_22 = 0.025101925;
constexpr double kArmature4010   = 0.00425;

// Second-order critically-damped model: stiffness = armature * w^2,
// damping = 2 * zeta * armature * w  (w = 10 Hz, zeta = 2.0)
constexpr double kNaturalFreq  = 10.0 * 2.0 * 3.1415926535;
constexpr double kDampingRatio = 2.0;

constexpr double kStiffness5020    = kArmature5020    * kNaturalFreq * kNaturalFreq;
constexpr double kStiffness7520_14 = kArmature7520_14 * kNaturalFreq * kNaturalFreq;
constexpr double kStiffness7520_22 = kArmature7520_22 * kNaturalFreq * kNaturalFreq;
constexpr double kStiffness4010    = kArmature4010    * kNaturalFreq * kNaturalFreq;

constexpr double kDamping5020    = 2.0 * kDampingRatio * kArmature5020    * kNaturalFreq;
constexpr double kDamping7520_14 = 2.0 * kDampingRatio * kArmature7520_14 * kNaturalFreq;
constexpr double kDamping7520_22 = 2.0 * kDampingRatio * kArmature7520_22 * kNaturalFreq;
constexpr double kDamping4010    = 2.0 * kDampingRatio * kArmature4010    * kNaturalFreq;

// Effort limits (Nm)
constexpr double kEffort5020    = 25.0;
constexpr double kEffort7520_14 = 88.0;
constexpr double kEffort7520_22 = 139.0;
constexpr double kEffort4010    = 5.0;

// ─── per-joint arrays (MuJoCo order) ──────────────────────────────────────────

// Policy action scale: 0.25 * effort_limit / stiffness
// (IsaacLab G1_CYLINDER_CFG actuator configuration)
inline constexpr std::array<double, 29> g1_action_scale = {
    0.25 * kEffort7520_22 / kStiffness7520_22,  // left_hip_pitch
    0.25 * kEffort7520_22 / kStiffness7520_22,  // left_hip_roll
    0.25 * kEffort7520_14 / kStiffness7520_14,  // left_hip_yaw
    0.25 * kEffort7520_22 / kStiffness7520_22,  // left_knee
    0.25 * kEffort5020    / kStiffness5020,     // left_ankle_pitch
    0.25 * kEffort5020    / kStiffness5020,     // left_ankle_roll
    0.25 * kEffort7520_22 / kStiffness7520_22,  // right_hip_pitch
    0.25 * kEffort7520_22 / kStiffness7520_22,  // right_hip_roll
    0.25 * kEffort7520_14 / kStiffness7520_14,  // right_hip_yaw
    0.25 * kEffort7520_22 / kStiffness7520_22,  // right_knee
    0.25 * kEffort5020    / kStiffness5020,     // right_ankle_pitch
    0.25 * kEffort5020    / kStiffness5020,     // right_ankle_roll
    0.25 * kEffort7520_14 / kStiffness7520_14,  // waist_yaw
    0.25 * kEffort5020    / kStiffness5020,     // waist_roll
    0.25 * kEffort5020    / kStiffness5020,     // waist_pitch
    0.25 * kEffort5020    / kStiffness5020,     // left_shoulder_pitch
    0.25 * kEffort5020    / kStiffness5020,     // left_shoulder_roll
    0.25 * kEffort5020    / kStiffness5020,     // left_shoulder_yaw
    0.25 * kEffort5020    / kStiffness5020,     // left_elbow
    0.25 * kEffort5020    / kStiffness5020,     // left_wrist_roll
    0.25 * kEffort4010    / kStiffness4010,     // left_wrist_pitch
    0.25 * kEffort4010    / kStiffness4010,     // left_wrist_yaw
    0.25 * kEffort5020    / kStiffness5020,     // right_shoulder_pitch
    0.25 * kEffort5020    / kStiffness5020,     // right_shoulder_roll
    0.25 * kEffort5020    / kStiffness5020,     // right_shoulder_yaw
    0.25 * kEffort5020    / kStiffness5020,     // right_elbow
    0.25 * kEffort5020    / kStiffness5020,     // right_wrist_roll
    0.25 * kEffort4010    / kStiffness4010,     // right_wrist_pitch
    0.25 * kEffort4010    / kStiffness4010,     // right_wrist_yaw
};

// PD position gains (ankles / waist roll+pitch use 2x)
inline constexpr std::array<float, 29> g1_kps = {
    static_cast<float>(kStiffness7520_22),        // left_hip_pitch
    static_cast<float>(kStiffness7520_22),        // left_hip_roll
    static_cast<float>(kStiffness7520_14),        // left_hip_yaw
    static_cast<float>(kStiffness7520_22),        // left_knee
    static_cast<float>(2.0 * kStiffness5020),     // left_ankle_pitch
    static_cast<float>(2.0 * kStiffness5020),     // left_ankle_roll
    static_cast<float>(kStiffness7520_22),        // right_hip_pitch
    static_cast<float>(kStiffness7520_22),        // right_hip_roll
    static_cast<float>(kStiffness7520_14),        // right_hip_yaw
    static_cast<float>(kStiffness7520_22),        // right_knee
    static_cast<float>(2.0 * kStiffness5020),     // right_ankle_pitch
    static_cast<float>(2.0 * kStiffness5020),     // right_ankle_roll
    static_cast<float>(kStiffness7520_14),        // waist_yaw
    static_cast<float>(2.0 * kStiffness5020),     // waist_roll
    static_cast<float>(2.0 * kStiffness5020),     // waist_pitch
    static_cast<float>(kStiffness5020),           // left_shoulder_pitch
    static_cast<float>(kStiffness5020),           // left_shoulder_roll
    static_cast<float>(kStiffness5020),           // left_shoulder_yaw
    static_cast<float>(kStiffness5020),           // left_elbow
    static_cast<float>(kStiffness5020),           // left_wrist_roll
    static_cast<float>(kStiffness4010),           // left_wrist_pitch
    static_cast<float>(kStiffness4010),           // left_wrist_yaw
    static_cast<float>(kStiffness5020),           // right_shoulder_pitch
    static_cast<float>(kStiffness5020),           // right_shoulder_roll
    static_cast<float>(kStiffness5020),           // right_shoulder_yaw
    static_cast<float>(kStiffness5020),           // right_elbow
    static_cast<float>(kStiffness5020),           // right_wrist_roll
    static_cast<float>(kStiffness4010),           // right_wrist_pitch
    static_cast<float>(kStiffness4010),           // right_wrist_yaw
};

// PD damping gains (ankles / waist roll+pitch use 2x)
inline constexpr std::array<float, 29> g1_kds = {
    static_cast<float>(kDamping7520_22),          // left_hip_pitch
    static_cast<float>(kDamping7520_22),          // left_hip_roll
    static_cast<float>(kDamping7520_14),          // left_hip_yaw
    static_cast<float>(kDamping7520_22),          // left_knee
    static_cast<float>(2.0 * kDamping5020),       // left_ankle_pitch
    static_cast<float>(2.0 * kDamping5020),       // left_ankle_roll
    static_cast<float>(kDamping7520_22),          // right_hip_pitch
    static_cast<float>(kDamping7520_22),          // right_hip_roll
    static_cast<float>(kDamping7520_14),          // right_hip_yaw
    static_cast<float>(kDamping7520_22),          // right_knee
    static_cast<float>(2.0 * kDamping5020),       // right_ankle_pitch
    static_cast<float>(2.0 * kDamping5020),       // right_ankle_roll
    static_cast<float>(kDamping7520_14),          // waist_yaw
    static_cast<float>(2.0 * kDamping5020),       // waist_roll
    static_cast<float>(2.0 * kDamping5020),       // waist_pitch
    static_cast<float>(kDamping5020),             // left_shoulder_pitch
    static_cast<float>(kDamping5020),             // left_shoulder_roll
    static_cast<float>(kDamping5020),             // left_shoulder_yaw
    static_cast<float>(kDamping5020),             // left_elbow
    static_cast<float>(kDamping5020),             // left_wrist_roll
    static_cast<float>(kDamping4010),             // left_wrist_pitch
    static_cast<float>(kDamping4010),             // left_wrist_yaw
    static_cast<float>(kDamping5020),             // right_shoulder_pitch
    static_cast<float>(kDamping5020),             // right_shoulder_roll
    static_cast<float>(kDamping5020),             // right_shoulder_yaw
    static_cast<float>(kDamping5020),             // right_elbow
    static_cast<float>(kDamping5020),             // right_wrist_roll
    static_cast<float>(kDamping4010),             // right_wrist_pitch
    static_cast<float>(kDamping4010),             // right_wrist_yaw
};

// Default joint angles (standing pose)
inline constexpr std::array<double, 29> g1_default_angles = {
    -0.312,  // left_hip_pitch
     0.0,    // left_hip_roll
     0.0,    // left_hip_yaw
     0.669,  // left_knee
    -0.363,  // left_ankle_pitch
     0.0,    // left_ankle_roll
    -0.312,  // right_hip_pitch
     0.0,    // right_hip_roll
     0.0,    // right_hip_yaw
     0.669,  // right_knee
    -0.363,  // right_ankle_pitch
     0.0,    // right_ankle_roll
     0.0,    // waist_yaw
     0.0,    // waist_roll
     0.0,    // waist_pitch
     0.2,    // left_shoulder_pitch
     0.2,    // left_shoulder_roll
     0.0,    // left_shoulder_yaw
     0.6,    // left_elbow
     0.0,    // left_wrist_roll
     0.0,    // left_wrist_pitch
     0.0,    // left_wrist_yaw
     0.2,    // right_shoulder_pitch
    -0.2,    // right_shoulder_roll
     0.0,    // right_shoulder_yaw
     0.6,    // right_elbow
     0.0,    // right_wrist_roll
     0.0,    // right_wrist_pitch
     0.0,    // right_wrist_yaw
};

} // namespace kist
