/**
 * Stereo-inertial ORB-SLAM3 runner for INDEMIND recorder output.
 * Supports map save/load for manipulation/welding data collection.
 *
 * Workflow:
 *   1. Build map:   --save-map workshop_map --no-realtime --no-viewer
 *   2. Localize + extend: --load-map workshop_map --save-map workshop_demo01
 *
 * When --load-map is used, the system loads a pre-built atlas and localizes
 * against it via place recognition. Local mapping remains active so the map
 * can be extended as the environment changes.
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
 *   ./stereo_inertial_indemind \
 *       --vocab    Vocabulary/ORBvoc.txt \
 *       --settings Examples/Stereo-Inertial/INDEMIND.yaml \
 *       --data     <data_dir> \
 *       [--name      trajectory_name] \
 *       [--save-map  map_name] \
 *       [--load-map  map_name] \
 *       [--max-lost  100] \
 *       [--no-realtime] \
 *       [--no-viewer]
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
    // Parse CLI flags
    string vocabPath, settingsPath, dataDir, fileName;
    string loadMapPath, saveMapPath, saveDir;
    int maxLostFrames = 100;
    bool bViewer = true;
    bool bFileName = false;
    bool bNoRealtime = false;

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if      (arg == "--vocab"      && i+1 < argc) vocabPath    = argv[++i];
        else if (arg == "--settings"   && i+1 < argc) settingsPath = argv[++i];
        else if (arg == "--data"       && i+1 < argc) dataDir      = argv[++i];
        else if (arg == "--name"       && i+1 < argc) { fileName = argv[++i]; bFileName = true; }
        else if (arg == "--no-viewer")                 bViewer = false;
        else if (arg == "--load-map"   && i+1 < argc) loadMapPath  = argv[++i];
        else if (arg == "--save-map"   && i+1 < argc) saveMapPath  = argv[++i];
        else if (arg == "--max-lost"   && i+1 < argc) maxLostFrames = stoi(argv[++i]);
        else if (arg == "--no-realtime")               bNoRealtime = true;
        else if (arg == "--save-dir"   && i+1 < argc) saveDir      = argv[++i];
        else { cerr << "Unknown argument: " << arg << endl; }
    }

    if (vocabPath.empty() || settingsPath.empty() || dataDir.empty()) {
        cerr << "\nUsage: ./stereo_inertial_indemind \\\n"
                "    --vocab      Vocabulary/ORBvoc.txt \\\n"
                "    --settings   Examples/Stereo-Inertial/INDEMIND.yaml \\\n"
                "    --data       <data_dir> \\\n"
                "    [--name      trajectory_name] \\\n"
                "    [--save-map  map_name]         Save atlas on shutdown\\\n"
                "    [--load-map  map_name]         Load atlas at startup\\\n"
                "    [--max-lost  100]              Max consecutive lost frames before abort\\\n"
                "    [--no-realtime]                Process as fast as possible\\\n"
                "    [--no-viewer]\n\n";
        return 1;
    }

    // Log configuration
    if (!loadMapPath.empty())
        cout << "Loading atlas from: ./" << loadMapPath << ".osa" << endl;
    if (!saveMapPath.empty())
        cout << "Will save atlas to: ./" << saveMapPath << ".osa" << endl;

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

    // Init SLAM with optional atlas load/save paths
    ORB_SLAM3::System SLAM(vocabPath, settingsPath, ORB_SLAM3::System::IMU_STEREO,
                           bViewer, 0, string(), loadMapPath, saveMapPath);

    // Local mapping stays active — the map is extended as the environment changes.
    // Do NOT call SLAM.ActivateLocalizationMode().

    cout << "\n-------\nStart processing sequence ...\n"
         << "Images in sequence: " << nImages << "\n-------\n";

    int consecutiveLost = 0;
    vector<float> vTimesTrack(nImages);
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

        // Check tracking state
        int trackingState = SLAM.GetTrackingState();
        if (trackingState == 4) {  // Tracking::LOST
            consecutiveLost++;
            if (consecutiveLost % 10 == 1)
                cout << "LOST frame " << ni << " (" << consecutiveLost
                     << "/" << maxLostFrames << ")" << endl;
            if (maxLostFrames > 0 && consecutiveLost >= maxLostFrames) {
                cerr << "Exceeded max consecutive lost frames (" << maxLostFrames
                     << "), aborting." << endl;
                break;
            }
        } else {
            if (consecutiveLost > 0)
                cout << "Tracking recovered after " << consecutiveLost
                     << " lost frames at frame " << ni << endl;
            consecutiveLost = 0;
        }

        // Real-time pacing (skip if --no-realtime)
        if (!bNoRealtime) {
            double T = (ni < nImages - 1) ? vTimestampsCam[ni + 1] - tframe
                                           : tframe - vTimestampsCam[ni - 1];
            if (ttrack < T)
                usleep((T - ttrack) * 1e6);
        }
    }

    // Shutdown triggers SaveAtlas automatically if --save-map was provided
    SLAM.Shutdown();

    // Save trajectories
    string prefix = saveDir.empty() ? "" : (saveDir.back() == '/' ? saveDir : saveDir + "/");
    if (bFileName) {
        SLAM.SaveTrajectoryEuRoC(prefix + "f_"  + fileName + ".txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC(prefix + "kf_" + fileName + ".txt");
    } else {
        SLAM.SaveTrajectoryEuRoC(prefix + "CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC(prefix + "KeyFrameTrajectory.txt");
    }
    SLAM.SaveTrajectoryCSV(prefix + "camera_trajectory.csv");

    return 0;
}
