# kist-gearsonic-inference

C++ inference pipeline for GR00T WholeBodyControl on the Unitree G1 humanoid robot.

## Architecture

<!-- architecture image here -->

## Dependencies

| Package | Purpose |
|---|---|
| `XRoboToolkit` | PICO VR PC daemon |
| `libPXREARobotSDK` | PICO VR C++ client library |
| `unitree_sdk2` | Unitree G1 DDS client library |
| `yaml-cpp` | YAML config parser |
| `CUDA` | GPU runtime for TensorRT inference |
| `TensorRT` | ONNX → TRT engine conversion and inference |

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

## Run

Our binaries need no environment setup — CMake bakes the SDK library
paths into each executable's RUNPATH at build time.

### XRoboToolkit (PICO VR daemon)

The daemon must be running on the **host** before any VR-consuming binary
starts (they probe port 60061 and fail fast with a clear error if it is
absent):

```bash
source env.sh    # defines run_vr_daemon (the daemon needs its own LD_LIBRARY_PATH)
run_vr_daemon
```

- Keep exactly **one** daemon running; our binaries connect to it via
  localhost (the dev container uses host networking).
- Closing the terminal that launched it may kill the daemon (SIGHUP);
  use `nohup` if it should survive.
- After the daemon is up, connect the headset from its XRoboToolkit app.
