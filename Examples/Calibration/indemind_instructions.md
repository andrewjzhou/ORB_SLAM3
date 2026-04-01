# INDEMIND Stereo-Inertial Camera — Ubuntu 22.04 Setup

This document describes how to set up the INDEMIND stereo camera (two grayscale cameras + IMU) on Ubuntu 22.04 for use with ORB-SLAM3.

---

## Background / Why This Is Non-Trivial

The INDEMIND SDK (`libindemind.so`) is a closed-source prebuilt binary last updated in 2020, compiled against OpenCV 3.4. Ubuntu 22.04 ships OpenCV 4.x. Two problems arise:

1. `libindemind.so` links against `libopencv_core.so.3.4` etc., which don't exist on Ubuntu 22.04.
2. Loading OpenCV 3.4 and OpenCV 4.x in the same process causes symbol conflicts that crash `imshow`.

The solution: build a minimal OpenCV 3.4 from source into an isolated local directory, and link the recorder binary exclusively against it.

---

## Step 1 — Clone the INDEMIND SDK

```bash
git clone https://github.com/INDEMIND/IMSEE-SDK.git ~/Dev/IMSEE-SDK
```

The relevant files after cloning:
- `lib/others/x64-opencv3.4.3/libindemind.so` — the correct variant to use on x86-64
- `src/detector/lib/x86-64/libMNN.so` — neural net runtime (required by libindemind)
- `src/driver/lib/x86-64/libusbdriver.so` — USB driver (libusb is statically included in libindemind, this file is only for the ROS wrapper)
- `include/` — SDK headers (`imrsdk.h`, `types.h`, `imrdata.h`)

Use the `lib/others/x64-opencv3.4.3/libindemind.so` variant (standard OpenCV naming, no suffix quirks), NOT `lib/x86-64/libindemind.so` (that one links against `libopencv_core3.so.3.3` with a `3` suffix that doesn't exist anywhere).

---

## Step 2 — Install GTK3 Dev Package

Required for OpenCV 3.4's `highgui` (imshow) to work:

```bash
sudo apt-get install -y libgtk-3-dev
```

---

## Step 3 — Build Minimal OpenCV 3.4 From Source

Build only the modules `libindemind.so` needs, plus `highgui` for display. Install to an isolated local directory — nothing touches `/usr` or `/usr/local`.

```bash
git clone --depth 1 --branch 3.4.20 https://github.com/opencv/opencv.git ~/Dev/opencv_3.4_src

mkdir ~/Dev/opencv_3.4_build && cd ~/Dev/opencv_3.4_build

cmake ~/Dev/opencv_3.4_src \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=~/Dev/opencv_3.4 \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_opencv_apps=OFF \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=OFF \
  -DWITH_CUDA=OFF \
  -DWITH_GTK=ON \
  -DWITH_GTK_2_X=OFF \
  -DWITH_QT=OFF \
  -DBUILD_LIST=core,imgproc,calib3d,videoio,imgcodecs,highgui

make -j$(nproc) install
```

After this, `~/Dev/opencv_3.4/lib/` contains `libopencv_core.so.3.4`, `libopencv_highgui.so.3.4`, etc. No other part of the system is affected.

---

## Step 4 — USB Permissions (udev rule)

The INDEMIND camera uses an OmniVision OV580 sensor (USB Vendor ID `05a9`). By default, the USB device node is read-only for non-root users, which prevents libusb from communicating with it.

Install the udev rule (file is at `Examples/Calibration/99-indemind.rules`):

```bash
sudo cp Examples/Calibration/99-indemind.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug** the camera. Verify access:

```bash
# Find the device: lsusb | grep "05a9"
# Should show: Bus 00X Device 00Y: ID 05a9:f581 OmniVision Technologies, Inc. USB Camera-OV580
ls -la /dev/bus/usb/<bus>/<device>
# Should show crw-rw-rw- (mode 0666)
```

---

## Step 5 — CMakeLists.txt Integration

The key CMake block (already in `CMakeLists.txt`):

```cmake
set(IMSEE_SDK_ROOT  "/home/andrew/Dev/IMSEE-SDK" CACHE PATH "Path to INDEMIND IMSEE-SDK")
set(OPENCV34_ROOT   "/home/andrew/Dev/opencv_3.4"  CACHE PATH "OpenCV 3.4 install root")

if(EXISTS "${IMSEE_SDK_ROOT}/include/imrsdk.h")
    set(IMSEE_SDK_LIB  "${IMSEE_SDK_ROOT}/lib/others/x64-opencv3.4.3/libindemind.so")
    set(IMSEE_MNN_LIB  "${IMSEE_SDK_ROOT}/src/detector/lib/x86-64/libMNN.so")
    set(OPENCV34_LIBS
        "${OPENCV34_ROOT}/lib/libopencv_highgui.so.3.4"
        "${OPENCV34_ROOT}/lib/libopencv_imgcodecs.so.3.4"
        "${OPENCV34_ROOT}/lib/libopencv_imgproc.so.3.4"
        "${OPENCV34_ROOT}/lib/libopencv_core.so.3.4"
    )
    add_executable(recorder_indemind Examples/Calibration/recorder_indemind.cc)
    target_include_directories(recorder_indemind PRIVATE
        "${IMSEE_SDK_ROOT}/include"
        "${OPENCV34_ROOT}/include"
    )
    target_link_libraries(recorder_indemind ${OPENCV34_LIBS} ${IMSEE_SDK_LIB} ${IMSEE_MNN_LIB} pthread)
    target_link_options(recorder_indemind PRIVATE "-Wl,--allow-shlib-undefined" "-Wl,--disable-new-dtags")
    set_target_properties(recorder_indemind PROPERTIES
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "${IMSEE_SDK_ROOT}/lib/others/x64-opencv3.4.3:${IMSEE_SDK_ROOT}/src/detector/lib/x86-64:${OPENCV34_ROOT}/lib"
    )
endif()
```

Key points:
- Link against **OpenCV 3.4 only** (not system OpenCV 4.x) to avoid symbol conflicts.
- `--disable-new-dtags` embeds `DT_RPATH` instead of `DT_RUNPATH` so paths propagate transitively to `libindemind.so`'s own dependencies at runtime.
- `--allow-shlib-undefined` prevents link-time errors for OpenCV 3.4 symbols inside `libindemind.so`.
- RPATH is embedded so no `LD_LIBRARY_PATH` is needed at runtime.

---

## Step 6 — Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target recorder_indemind -j$(nproc)
```

---

## Step 7 — Run

```bash
./Examples/Calibration/recorder_indemind <save_dir>
```

The `cam0/`, `cam1/`, and `IMU/` subdirectories are created automatically inside `<save_dir>`.

Expected output:
```
INDEMIND SDK initialized. Saving to: <save_dir>
Streaming. Press 'q' or Ctrl+C to quit.
[IMU] t=0.470713  accel=(0.017673, -0.010358, 0.990963) m/s^2  gyro=(0.179933, 0.008998, 0.204508) rad/s
...
```

Two OpenCV windows show the left and right grayscale feeds. IMU prints every 100 samples. Press `q` or `Ctrl+C` to quit.

Saved data layout:
```
<save_dir>/
├── cam0/
│   ├── times.txt          # nanosecond timestamps, one per line
│   └── <ns_timestamp>.png
├── cam1/
│   ├── times.txt
│   └── <ns_timestamp>.png
└── IMU/
    ├── acc.txt            # timestamp,x,y,z  (seconds, 15 decimal places)
    └── gyro.txt           # timestamp,x,y,z
```

This matches the EuRoC dataset format expected by ORB-SLAM3 and Kalibr.

---

## SDK API Quick Reference

```cpp
#include "imrsdk.h"   // CIMRSDK, MRCONFIG
#include "types.h"    // ImuData, IMG_RESOLUTION

indem::CIMRSDK *sdk = new indem::CIMRSDK();
indem::MRCONFIG config = {0};
config.bSlam         = false;
config.imgResolution = indem::IMG_640;   // 640x400; IMG_1280 = 1280x800
config.imgFrequency  = 50;               // max 200 Hz
config.imuFrequency  = 1000;             // max 1000 Hz
sdk->Init(config);

// Raw image callback (safe — no cv::Mat crossing the ABI boundary)
sdk->RegistModuleCameraCallback(
    [](double time, unsigned char *pLeft, unsigned char *pRight,
       int width, int height, int channel, void *param) {
        int type = (channel == 1) ? CV_8UC1 : CV_8UC3;
        cv::Mat left(height, width, type, pLeft);
        cv::Mat right(height, width, type, pRight);
        // clone() before returning if you need to keep the data
    }, nullptr);

// IMU callback
sdk->RegistModuleIMUCallback([](indem::ImuData imu) {
    // imu.timestamp  — seconds
    // imu.accel[3]   — m/s^2 (X, Y, Z)
    // imu.gyro[3]    — rad/s (X, Y, Z)
});

delete sdk;  // calls Release() internally
```

Use `RegistModuleCameraCallback` (raw bytes) rather than `RegistImgCallback` (cv::Mat) to avoid passing `cv::Mat` objects across the OpenCV 3.4/4.x ABI boundary.

**IMU output units (confirmed from device firmware):**
- `imu.accel[]` is in **g** — multiply by 9.81 to get m/s²
- `imu.gyro[]` is in **deg/s** — multiply by π/180 to get rad/s

`recorder_indemind.cc` applies these conversions automatically before saving.

**Reading device calibration from firmware:**

> **Note on factory camera calibration:** `MoudleAllParam` also stores `_left_camera` / `_right_camera`
> (`CameraParameter` structs with `_K`, `_focal_length`, `_principal_point`, distortion). However the
> firmware distortion model is pinhole-radtan (`k1, k2, t1, t2`), which cannot accurately model this
> camera's 140° FOV. This appears to be a manufacturer oversight — the equidistant (Kannala-Brandt)
> model is required for fisheye lenses. **Do not use the factory camera calibration for ORB-SLAM3.**
> Use the Kalibr `pinhole-equi` calibration instead (see Kalibr section below).

```cpp
auto params = sdk->GetModuleParams();   // returns indem::MoudleAllParam
auto& imu = params._imu;               // indem::IMUParameter

// Full-scale ranges (SI units):  _a_max ≈ 176 m/s²,  _g_max ≈ 30 rad/s
// Noise params (native units — divide by unit factor for SI):
//   _sigma_g_c:  0.12   deg/s/√Hz    → *pi/180 → 0.00209 rad/s/√Hz
//   _sigma_a_c:  0.009  g/√Hz        → *9.81   → 0.0883  m/s²/√Hz
//   _sigma_gw_c: 4e-5   deg/s/√s     → *pi/180 → 6.98e-7 rad/s/√s
//   _sigma_aw_c: 4e-5   g/√s         → *9.81   → 3.92e-4 m/s²/√s
// _T_BS[16]: IMU→body transform (4×4, identity on this device)
// _device._imu[32]: IMU chip description string ("120mm" for this unit)
```

---

## Calibration Results (2026-04-01)

Calibrated with Kalibr using an Aprilgrid 6x6 (tagSize 0.088m, tagSpacing 0.3) at 640×400.
ORB-SLAM3 yaml: `Examples/Stereo-Inertial/INDEMIND.yaml`
Kalibr output files: `data/calib/indemind/kalibr/`

### Camera intrinsics (KannalaBrandt8 / pinhole-equi)

| | fx | fy | cx | cy |
|---|---|---|---|---|
| cam0 (left) | 244.438 | 244.781 | 313.775 | 196.187 |
| cam1 (right) | 245.448 | 245.612 | 324.000 | 201.470 |

Distortion coefficients (k1, k2, k3, k4):
- cam0: `0.6010, 0.1609, -0.5731, 0.2728`
- cam1: `0.6015, 0.1728, -0.6398, 0.3347`

Visual calibration reprojection error: σ ≈ 0.15–0.19 px (excellent; threshold is < 0.3 px)

### Stereo extrinsics

Baseline: **119.82mm** (nominal 120mm — confirms tagSize was correct)

T_c1_c2 translation (right cam → left cam): `[0.1198, 0.0001, 0.0003]` m

### IMU-camera extrinsics

T_cam0_imu translation: `[0.0605, -0.0003, -0.0061]` m — IMU is ~60mm from left camera (physically between the two cameras, consistent with 120mm baseline)

IMU-camera rotation: ~180° around z-axis (IMU x/y axes point opposite to camera x/y axes)

### Timing

**Camera-IMU time offset: 23.6ms** (`t_imu = t_cam + 0.0236s`)

The camera clock lags the IMU clock by 23.6ms. This is a hardware clock offset, not a frame-rate effect (50Hz camera = 20ms between frames, so the offset is slightly more than one frame period).

**Implications for diffusion policy / data collection:**
- When replaying recorded data offline, timestamps from the INDEMIND SDK are camera timestamps. The IMU data is already saved with its own timestamps — the offset is accounted for in the ORB-SLAM3 yaml via `IMU.T_b_c1`.
- If feeding live data to a custom pipeline (e.g. extracting poses for demonstration recording), either use ORB-SLAM3 directly (handles offset internally) or shift camera timestamps by +23.6ms before fusing with IMU.
- For diffusion policy training, poses are extracted after-the-fact by ORB-SLAM3, so the offset is handled transparently — no manual correction needed.

### IMU noise parameters (from device firmware)

| Parameter | Value | Unit |
|---|---|---|
| Gyro noise density | 0.002094 | rad/s/√Hz |
| Accel noise density | 0.08829 | m/s²/√Hz |
| Gyro random walk | 6.981e-7 | rad/s/√s |
| Accel random walk | 3.924e-4 | m/s²/√s |

If ORB-SLAM3 IMU initialization fails to converge, multiply NoiseGyro and NoiseAcc by 10 (they are priors, not hard constraints).

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Module Parameters Load Fail!` + segfault | USB permission denied | Install udev rule, replug camera |
| `The function is not implemented... cvNamedWindow` | OpenCV built without GTK | `sudo apt install libgtk-3-dev`, rebuild OpenCV 3.4 |
| `Invalid number of channels` crash in imshow | OpenCV 3.4 + 4.x symbol conflict | Link recorder against OpenCV 3.4 only (not system OpenCV) |
| `libopencv_core.so.3.4 => not found` | Missing OpenCV 3.4 libs | Build OpenCV 3.4 from source (Step 3) |
