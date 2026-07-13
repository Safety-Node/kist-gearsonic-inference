# kist-gearsonic-inference

C++ inference pipeline for GR00T WholeBodyControl on the Unitree G1 humanoid robot.

## Architecture

[![Architecture](docs/kist-gearsonic-inference.svg)](docs/kist-gearsonic-inference.svg)

## Dependencies

| Package | Purpose |
|---|---|
| `XRoboToolkit` | PICO VR PC daemon |
| `libPXREARobotSDK` | PICO VR C++ client library |
| `unitree_sdk2` | Unitree G1 DDS client library |
| `yaml-cpp` | YAML config parser |
| `CUDA == 12.6` | GPU runtime for TensorRT inference |
| `TensorRT == 10.7` | ONNX → TRT engine conversion and inference |

### Installing XRoboToolkit

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

### Installing libPXREARobotSDK

```bash
mkdir -p thirdparty/pxrea/lib thirdparty/pxrea/include
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git thirdparty/XRoboToolkit-PC-Service

cd thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK
bash build.sh
cd ../../../..

cp thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h thirdparty/pxrea/include/
cp -r thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann thirdparty/pxrea/include/nlohmann/
cp thirdparty/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so thirdparty/pxrea/lib/
```

### Installing unitree_sdk2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git thirdparty/unitree_sdk2
```

### Installing yaml-cpp

```bash
sudo apt install libyaml-cpp-dev
```

## Build

```bash
cmake -B build && cmake --build build
```

## Run

### 1. XRoboToolkit (PICO VR daemon)

```bash
source env.sh
run_vr_daemon
```

Connect the headset from its XRoboToolkit app.

### 2. Control

```bash
./build/gearsonic
```

### Controller

| Input | Action |
|---|---|
| Left stick | Move (magnitude = speed) |
| Right stick | Rotate facing |
| A | Return to IDLE |
| Y | Mode up (IDLE / Slow Walk / Walk) |
| Trigger + Y | Mode up (hard actions, e.g. Run) |
| X | Mode down |
| Trigger + B / A | Height up / down (crouch modes) |
| B held 1s | Teleop on / off (engage in the reference pose: forearms 90° forward, palms inward) |
| Both grips 1s | Emergency stop |
