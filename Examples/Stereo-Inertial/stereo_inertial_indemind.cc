/**
 * Stereo-inertial ORB-SLAM3 runner for INDEMIND recorder output.
 *
 * Reads the directory layout produced by recorder_indemind:
 *   <data_dir>/cam0/times.txt          nanosecond timestamps, one per line
 *   <data_dir>/cam0/<timestamp>.png
 *   <data_dir>/cam1/times.txt
 *   <data_dir>/cam1/<timestamp>.png
 *   <data_dir>/IMU/gyro.txt            timestamp_s,wx,wy,wz
 *   <data_dir>/IMU/acc.txt             timestamp_s,ax,ay,az
 *
 * Usage:
 *   ./stereo_inertial_indemind Vocabulary/ORBvoc.txt \
 *       Examples/Stereo-Inertial/INDEMIND.yaml \
 *       <data_dir> [trajectory_name]
 *
 * Outputs:
 *   CameraTrajectory.txt / kf_<name>.txt   (EuRoC format)
 *   KeyFrameTrajectory.txt / f_<name>.txt
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <chrono>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

// -----------------------------------------------------------------------------
// Load image paths and timestamps from cam0/times.txt.
// cam1 images share the same timestamps as cam0.
// -----------------------------------------------------------------------------
void LoadImages(const string &dataDir,
                vector<string> &vstrLeft,
                vector<string> &vstrRight,
                vector<double> &vTimestamps)
{
    const string timesPath = dataDir + "/cam0/times.txt";
    ifstream fTimes(timesPath);
    if (!fTimes.is_open()) {
        cerr << "ERROR: cannot open " << timesPath << endl;
        return;
    }

    string line;
    while (getline(fTimes, line)) {
        if (line.empty()) continue;
        // Skip sub-second startup frames (< 10 digit ns timestamp)
        if (line.size() < 10) continue;

        double t = stod(line) / 1e9;  // ns → seconds
        vTimestamps.push_back(t);
        vstrLeft.push_back(dataDir  + "/cam0/" + line + ".png");
        vstrRight.push_back(dataDir + "/cam1/" + line + ".png");
    }
}

// -----------------------------------------------------------------------------
// Load and merge IMU data from separate gyro.txt and acc.txt files.
// Both files: timestamp_s,x,y,z
// Accel is linearly interpolated onto gyro timestamps.
// Output vectors are sorted by timestamp.
// -----------------------------------------------------------------------------
void LoadIMU(const string &dataDir,
             vector<double> &vTimestamps,
             vector<cv::Point3f> &vAcc,
             vector<cv::Point3f> &vGyro)
{
    // --- load gyro ---
    const string gyroPath = dataDir + "/IMU/gyro.txt";
    ifstream fGyro(gyroPath);
    if (!fGyro.is_open()) {
        cerr << "ERROR: cannot open " << gyroPath << endl;
        return;
    }

    vector<double> gyroT;
    vector<cv::Point3f> gyro;
    string line;
    while (getline(fGyro, line)) {
        if (line.empty()) continue;
        istringstream ss(line);
        string tok;
        double vals[4];
        for (int i = 0; i < 4; i++) { getline(ss, tok, ','); vals[i] = stod(tok); }
        gyroT.push_back(vals[0]);
        gyro.push_back(cv::Point3f(vals[1], vals[2], vals[3]));
    }

    // --- load acc ---
    const string accPath = dataDir + "/IMU/acc.txt";
    ifstream fAcc(accPath);
    if (!fAcc.is_open()) {
        cerr << "ERROR: cannot open " << accPath << endl;
        return;
    }

    vector<double> accT;
    vector<cv::Point3f> acc;
    while (getline(fAcc, line)) {
        if (line.empty()) continue;
        istringstream ss(line);
        string tok;
        double vals[4];
        for (int i = 0; i < 4; i++) { getline(ss, tok, ','); vals[i] = stod(tok); }
        accT.push_back(vals[0]);
        acc.push_back(cv::Point3f(vals[1], vals[2], vals[3]));
    }

    if (gyroT.empty() || accT.empty()) {
        cerr << "ERROR: empty IMU data" << endl;
        return;
    }

    // --- interpolate acc onto gyro timestamps ---
    // Walk through gyro samples; skip those outside the acc time range.
    size_t ai = 0;
    for (size_t gi = 0; gi < gyroT.size(); gi++) {
        double t = gyroT[gi];

        // Advance acc index so accT[ai] <= t < accT[ai+1]
        while (ai + 1 < accT.size() - 1 && accT[ai + 1] <= t)
            ai++;

        // Skip if outside acc range
        if (t < accT.front() || t > accT.back())
            continue;

        // Linear interpolation
        cv::Point3f aInterp;
        if (ai + 1 < accT.size()) {
            double dt = accT[ai + 1] - accT[ai];
            double alpha = (dt > 0.0) ? (t - accT[ai]) / dt : 0.0;
            aInterp = acc[ai] * (1.0 - alpha) + acc[ai + 1] * alpha;
        } else {
            aInterp = acc[ai];
        }

        vTimestamps.push_back(t);
        vGyro.push_back(gyro[gi]);
        vAcc.push_back(aInterp);
    }
}

// -----------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Parse named flags
    string vocabPath, settingsPath, dataDir, fileName;
    bool bViewer  = true;
    bool bFileName = false;

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if      (arg == "--vocab"    && i+1 < argc) vocabPath    = argv[++i];
        else if (arg == "--settings" && i+1 < argc) settingsPath = argv[++i];
        else if (arg == "--data"     && i+1 < argc) dataDir      = argv[++i];
        else if (arg == "--name"     && i+1 < argc) { fileName = argv[++i]; bFileName = true; }
        else if (arg == "--no-viewer") bViewer = false;
        else { cerr << "Unknown argument: " << arg << endl; }
    }

    if (vocabPath.empty() || settingsPath.empty() || dataDir.empty()) {
        cerr << "\nUsage: ./stereo_inertial_indemind \\\n"
                "    --vocab    Vocabulary/ORBvoc.txt \\\n"
                "    --settings Examples/Stereo-Inertial/INDEMIND.yaml \\\n"
                "    --data     <data_dir> \\\n"
                "    [--name    trajectory_name] \\\n"
                "    [--no-viewer]\n\n";
        return 1;
    }

    // Load images
    vector<string> vstrLeft, vstrRight;
    vector<double> vTimestampsCam;
    LoadImages(dataDir, vstrLeft, vstrRight, vTimestampsCam);

    const int nImages = vstrLeft.size();
    if (nImages == 0) {
        cerr << "ERROR: no images loaded from " << dataDir << endl;
        return 1;
    }
    cout << "Loaded " << nImages << " stereo pairs." << endl;

    // Load IMU
    vector<double>       vTimestampsImu;
    vector<cv::Point3f>  vAcc, vGyro;
    LoadIMU(dataDir, vTimestampsImu, vAcc, vGyro);

    if (vTimestampsImu.empty()) {
        cerr << "ERROR: no IMU data loaded from " << dataDir << endl;
        return 1;
    }
    cout << "Loaded " << vTimestampsImu.size() << " IMU measurements." << endl;

    // Find first IMU sample just before the first camera frame
    int firstImu = 0;
    while (firstImu < (int)vTimestampsImu.size() &&
           vTimestampsImu[firstImu] <= vTimestampsCam[0])
        firstImu++;
    firstImu = max(0, firstImu - 1);

    // Init SLAM
    ORB_SLAM3::System SLAM(vocabPath, settingsPath, ORB_SLAM3::System::IMU_STEREO, bViewer);

    vector<float> vTimesTrack(nImages);
    cout << "\n-------\nStart processing sequence ...\n"
         << "Images in sequence: " << nImages << "\n-------\n";

    cv::Mat imLeft, imRight;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for (int ni = 0; ni < nImages; ni++) {
        imLeft  = cv::imread(vstrLeft[ni],  cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrRight[ni], cv::IMREAD_UNCHANGED);

        if (imLeft.empty()) {
            cerr << "Failed to load: " << vstrLeft[ni] << endl;
            return 1;
        }
        if (imRight.empty()) {
            cerr << "Failed to load: " << vstrRight[ni] << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        // Bundle IMU measurements since previous frame
        vImuMeas.clear();
        if (ni > 0) {
            while (firstImu < (int)vTimestampsImu.size() &&
                   vTimestampsImu[firstImu] <= tframe) {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                    vAcc[firstImu].x,  vAcc[firstImu].y,  vAcc[firstImu].z,
                    vGyro[firstImu].x, vGyro[firstImu].y, vGyro[firstImu].z,
                    vTimestampsImu[firstImu]));
                firstImu++;
            }
        }

        auto t1 = chrono::steady_clock::now();
        SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);
        auto t2 = chrono::steady_clock::now();

        double ttrack = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        // Real-time pacing: wait if tracking finished early
        double T = (ni < nImages - 1) ? vTimestampsCam[ni + 1] - tframe
                                       : tframe - vTimestampsCam[ni - 1];
        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    SLAM.Shutdown();

    // Save trajectories
    if (bFileName) {
        SLAM.SaveTrajectoryEuRoC("f_"  + fileName + ".txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("kf_" + fileName + ".txt");
    } else {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}
