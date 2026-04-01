// API docs
// https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/data/get_imgage.html
// https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/data/get_imu.html
// https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/install_ubuntu.html
//
// Ubuntu 22.04 compatibility:
//   libindemind.so (x64-opencv3.4.3 variant) needs OpenCV 3.4 at runtime.
//   We shim this with symlinks in IMSEE-SDK/opencv_compat/ pointing to OpenCV 4.5.
//   RPATHs are set in CMakeLists.txt so no LD_LIBRARY_PATH is needed.
//
//   USB permissions: add yourself to the plugdev group or install udev rules:
//     sudo cp 99-indemind.rules /etc/udev/rules.d/
//     sudo udevadm control --reload-rules && sudo udevadm trigger

#include <signal.h>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// INDEMIND IMSEE-SDK headers
#include "imrsdk.h"
#include "types.h"

using namespace std;

bool b_continue_session = true;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    auto sdk = new indem::CIMRSDK();
    indem::MRCONFIG config = {0};
    config.bSlam        = false;
    config.imgResolution = indem::IMG_640;   // 640x400
    config.imgFrequency  = 50;
    config.imuFrequency  = 1000;

    if (!sdk->Init(config)) {
        cerr << "Failed to initialize INDEMIND SDK.\n"
             << "  - Is the camera plugged in?\n"
             << "  - Do you have USB permissions? Run:\n"
             << "      sudo cp 99-indemind.rules /etc/udev/rules.d/\n"
             << "      sudo udevadm control --reload-rules && sudo udevadm trigger\n"
             << "    then unplug/replug the camera, or run with sudo.\n";
        delete sdk;
        return 1;
    }

    cout << "INDEMIND SDK initialized." << endl;

    // --- Image state (shared between callback and main thread) ---
    mutex img_mutex;
    condition_variable img_cv;
    cv::Mat left_img, right_img;
    double img_timestamp = 0.0;
    bool img_ready = false;

    // Use raw-bytes callback to avoid cv::Mat ABI issues between
    // OpenCV 3.4 (inside libindemind.so) and OpenCV 4.x (our code).
    sdk->RegistModuleCameraCallback(
        [&](double time, unsigned char *pLeft, unsigned char *pRight,
            int width, int height, int channel, void *param) {
            int type = (channel == 1) ? CV_8UC1 : CV_8UC3;
            cv::Mat l(height, width, type, pLeft);
            cv::Mat r(height, width, type, pRight);
            {
                lock_guard<mutex> lock(img_mutex);
                left_img      = l.clone();   // deep copy before SDK frees its buffer
                right_img     = r.clone();
                img_timestamp = time;
                img_ready     = true;
            }
            img_cv.notify_one();
        },
        nullptr);

    // --- IMU callback ---
    int imu_count = 0;
    sdk->RegistModuleIMUCallback([&](indem::ImuData imu) {
        // Print every 100th sample to avoid flooding the terminal
        if (imu_count % 100 == 0) {
            cout << fixed << setprecision(6)
                 << "[IMU] t=" << imu.timestamp
                 << "  accel=(" << imu.accel[0] << ", "
                                << imu.accel[1] << ", "
                                << imu.accel[2] << ") m/s^2"
                 << "  gyro=("  << imu.gyro[0]  << ", "
                                << imu.gyro[1]  << ", "
                                << imu.gyro[2]  << ") rad/s"
                 << endl;
        }
        ++imu_count;
    });

    cout << "Streaming. Press 'q' or Ctrl+C to quit." << endl;

    cv::namedWindow("INDEMIND Left",  cv::WINDOW_AUTOSIZE);
    cv::namedWindow("INDEMIND Right", cv::WINDOW_AUTOSIZE);

    while (b_continue_session) {
        cv::Mat l, r;
        {
            unique_lock<mutex> lock(img_mutex);
            // Wait up to 100 ms for a new frame
            if (!img_cv.wait_for(lock, chrono::milliseconds(100),
                                 [&] { return img_ready; })) {
                continue;  // timeout — no frame yet, loop again
            }
            l = left_img.clone();
            r = right_img.clone();
            img_ready = false;
        }

        if (!l.empty()) cv::imshow("INDEMIND Left",  l);
        if (!r.empty()) cv::imshow("INDEMIND Right", r);

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 'Q' || key == 27 /* ESC */)
            break;
    }

    delete sdk;
    cout << "System shutdown!" << endl;
    return 0;
}
