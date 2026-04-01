// API docs
// https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/data/get_imgage.html
// https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/data/get_imu.html
// https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/install_ubuntu.html
//
// Ubuntu 22.04 compatibility:
//   libindemind.so (x64-opencv3.4.3 variant) needs OpenCV 3.4 at runtime.
//   Built from source into ~/Dev/opencv_3.4/ (isolated, no system impact).
//   RPATHs are set in CMakeLists.txt so no LD_LIBRARY_PATH is needed.
//
//   USB permissions: install udev rule once:
//     sudo cp 99-indemind.rules /etc/udev/rules.d/
//     sudo udevadm control --reload-rules && sudo udevadm trigger
//   then unplug/replug the camera.
//
// Usage:
//   ./recorder_indemind <save_dir>

#include <signal.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iomanip>
#include <array>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

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
    if (argc != 2) {
        cerr << "Usage: ./recorder_indemind path_to_saving_folder" << endl;
        return 1;
    }
    string directory = string(argv[1]);

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // --- Create output directories ---
    filesystem::create_directories(directory + "/cam0");
    filesystem::create_directories(directory + "/cam1");
    filesystem::create_directories(directory + "/IMU");

    // --- Open output files ---
    ofstream accFile, gyroFile, cam0TsFile, cam1TsFile;
    accFile.open(directory + "/IMU/acc.txt");
    gyroFile.open(directory + "/IMU/gyro.txt");
    cam0TsFile.open(directory + "/cam0/times.txt");
    cam1TsFile.open(directory + "/cam1/times.txt");
    if (!accFile.is_open() || !gyroFile.is_open() ||
        !cam0TsFile.is_open() || !cam1TsFile.is_open()) {
        cerr << "Could not open output files in: " << directory << "\n"
             << "Make sure the following subdirectories exist:\n"
             << "  " << directory << "/cam0/\n"
             << "  " << directory << "/cam1/\n"
             << "  " << directory << "/IMU/\n";
        return 1;
    }

    // --- Init SDK ---
    auto sdk = new indem::CIMRSDK();
    indem::MRCONFIG config = {0};
    config.bSlam         = false;
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
    cout << "INDEMIND SDK initialized. Saving to: " << directory << endl;

    // --- Shared state ---
    mutex imu_mutex;
    condition_variable img_cv;

    // Image state
    cv::Mat left_img, right_img;
    double img_timestamp = 0.0;
    bool img_ready = false;

    // IMU buffers (drained each frame in the main loop)
    vector<double>          v_acc_timestamp, v_gyro_timestamp;
    vector<array<float, 3>> v_acc_data, v_gyro_data;

    // --- Image callback (raw bytes — avoids cv::Mat ABI issues) ---
    sdk->RegistModuleCameraCallback(
        [&](double time, unsigned char *pLeft, unsigned char *pRight,
            int width, int height, int channel, void *param) {
            int type = (channel == 1) ? CV_8UC1 : CV_8UC3;
            cv::Mat l(height, width, type, pLeft);
            cv::Mat r(height, width, type, pRight);
            {
                lock_guard<mutex> lock(imu_mutex);
                left_img      = l.clone();
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
        {
            lock_guard<mutex> lock(imu_mutex);
            v_acc_data.push_back({imu.accel[0], imu.accel[1], imu.accel[2]});
            v_acc_timestamp.push_back(imu.timestamp);
            v_gyro_data.push_back({imu.gyro[0], imu.gyro[1], imu.gyro[2]});
            v_gyro_timestamp.push_back(imu.timestamp);
        }
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
        double imTs;
        vector<double>          vAccTs, vGyroTs;
        vector<array<float, 3>> vAcc, vGyro;
        {
            unique_lock<mutex> lock(imu_mutex);
            if (!img_cv.wait_for(lock, chrono::milliseconds(100),
                                 [&] { return img_ready; }))
                continue;

            l     = left_img.clone();
            r     = right_img.clone();
            imTs  = img_timestamp;

            // Drain IMU buffers
            vAcc    = v_acc_data;      vAccTs  = v_acc_timestamp;
            vGyro   = v_gyro_data;     vGyroTs = v_gyro_timestamp;
            v_acc_data.clear();        v_acc_timestamp.clear();
            v_gyro_data.clear();       v_gyro_timestamp.clear();

            img_ready = false;
        }

        // Display
        if (!l.empty()) cv::imshow("INDEMIND Left",  l);
        if (!r.empty()) cv::imshow("INDEMIND Right", r);

        // Save images
        long int imTsInt = (long int)(1e9 * imTs);
        string imgLeft  = directory + "/cam0/" + to_string(imTsInt) + ".png";
        string imgRight = directory + "/cam1/" + to_string(imTsInt) + ".png";
        if (!l.empty()) { cv::imwrite(imgLeft,  l); cam0TsFile << imTsInt << endl; }
        if (!r.empty()) { cv::imwrite(imgRight, r); cam1TsFile << imTsInt << endl; }

        // Save IMU
        for (size_t i = 0; i < vAcc.size(); i++)
            accFile << setprecision(15)
                    << vAccTs[i] << "," << vAcc[i][0] << ","
                    << vAcc[i][1] << "," << vAcc[i][2] << endl;
        for (size_t i = 0; i < vGyro.size(); i++)
            gyroFile << setprecision(15)
                     << vGyroTs[i] << "," << vGyro[i][0] << ","
                     << vGyro[i][1] << "," << vGyro[i][2] << endl;

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 'Q' || key == 27 /* ESC */)
            break;
    }

    delete sdk;

    accFile.close();
    gyroFile.close();
    cam0TsFile.close();
    cam1TsFile.close();

    cout << "System shutdown!" << endl;
    return 0;
}
