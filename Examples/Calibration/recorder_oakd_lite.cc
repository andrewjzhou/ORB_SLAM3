/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <iostream>
#include <iomanip>
#include <chrono>

#include <opencv2/highgui.hpp>

#include <depthai/depthai.hpp>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main() {

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Check device IMU before starting pipeline
    auto device = std::make_shared<dai::Device>();
    std::string connectedIMU = device->getConnectedIMU();
    cout << "Connected IMU: " << (connectedIMU.empty() ? "none" : connectedIMU) << endl;

    // Pipeline
    dai::Pipeline pipeline(device);

    // Left grayscale camera (CAM_B) — used for monocular-inertial
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camQueue = cam->requestOutput({640, 400}, dai::ImgFrame::Type::GRAY8,
                                       dai::ImgResizeMode::CROP, 30.0f)->createOutputQueue();

    // IMU: accelerometer + gyroscope at 200 Hz
    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableFirmwareUpdate(true);
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    auto imuQueue = imu->out.createOutputQueue(50, false);

    pipeline.start();
    cout << "OAK-D Lite pipeline started. Press Ctrl+C or 'q' to stop." << endl;
    cout << fixed << setprecision(6);

    cv::namedWindow("cam0", cv::WINDOW_AUTOSIZE);

    while (b_continue_session && pipeline.isRunning()) {
        // Show camera frame
        auto imgFrame = camQueue->tryGet<dai::ImgFrame>();
        if (imgFrame) {
            cv::imshow("cam0", imgFrame->getCvFrame());
        }

        // Print all available IMU packets
        auto imuData = imuQueue->tryGet<dai::IMUData>();
        if (imuData) {
            for (const auto& pkt : imuData->packets) {
                const auto& acc  = pkt.acceleroMeter;
                const auto& gyro = pkt.gyroscope;

                double accTs  = chrono::duration<double>(acc.getTimestamp().time_since_epoch()).count();
                double gyroTs = chrono::duration<double>(gyro.getTimestamp().time_since_epoch()).count();

                cout << "Accel  [" << accTs  << " s]  x: " << acc.x
                     << "  y: " << acc.y  << "  z: " << acc.z  << "  m/s^2\n";
                cout << "Gyro   [" << gyroTs << " s]  x: " << gyro.x
                     << "  y: " << gyro.y << "  z: " << gyro.z << "  rad/s\n";
            }
        }

        if (cv::waitKey(1) == 'q')
            break;
    }

    pipeline.stop();
    pipeline.wait();

    cout << "System shutdown!\n";
    return 0;
}
