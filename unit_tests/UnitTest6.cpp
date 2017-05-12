/*
 * Unit test for MSCKF.
 * Use KITTI IMU data to propagate, KITTI GPS data to update.
 * Propagate and update.
 */

#include "SLAMTrajectoryDrawer.h"
#include "Utils.h"
#include "VirtualGPSTracker.h"

#include "MSCKF/MSCKF.h"
#include "MSCKF_Simulation/Helpers.h"

#include <glm/glm.hpp>
#include <yaml-cpp/yaml.h>

#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;
using namespace glm;
using namespace Eigen;

namespace {

string imu_filename, gt_filename;
string out_prefix, out_suffix;

double sigma_na, sigma_nwa;
double sigma_ng, sigma_nwg;
double sigma_gps;

Matrix3d R_gps_in_imu;
Vector3d p_gps_in_imu;

double virtual_gps_sigma;
size_t imu_per_gps;

size_t start, stop;
bool shut_up;

const Quaterniond INIT_ORIENTATION(Matrix3d::Identity());

void LoadYAMLProfile(const char yaml_filename[]) throw(YAMLParseError) {
  YAML::Node yaml = YAML::LoadFile(yaml_filename);
  if (!yaml.IsMap())
    throw YAMLParseError();

  const YAML::Node& io = yaml["io"];
  if (!io.IsMap())
    throw YAMLParseError();
  imu_filename = io["imu_file"].as<string>();
  gt_filename = io["ground_truth_file"].as<string>();
  out_prefix = io["out_name_prefix"].as<string>();
  out_suffix = io["out_name_suffix"].as<string>();

  const YAML::Node& sigma = yaml["sigma"];
  if (!sigma.IsMap())
    throw YAMLParseError();
  sigma_nwg = sigma["gyroscope_bias"].as<double>();
  sigma_ng = sigma["gyroscope_white_noise"].as<double>();
  sigma_nwa = sigma["accelerometer_bias"].as<double>();
  sigma_na = sigma["accelerometer_white_noise"].as<double>();
  sigma_gps = sigma["gps_noise"].as<double>();

  const YAML::Node& calib = yaml["calibration"];
  if (!calib.IsMap())
    throw YAMLParseError();
  if (!calib["R_gps_in_imu"].IsSequence())
    throw YAMLParseError();
  R_gps_in_imu << calib["R_gps_in_imu"][0].as<double>(),
                  calib["R_gps_in_imu"][1].as<double>(),
                  calib["R_gps_in_imu"][2].as<double>(),
                  calib["R_gps_in_imu"][3].as<double>(),
                  calib["R_gps_in_imu"][4].as<double>(),
                  calib["R_gps_in_imu"][5].as<double>(),
                  calib["R_gps_in_imu"][6].as<double>(),
                  calib["R_gps_in_imu"][7].as<double>(),
                  calib["R_gps_in_imu"][8].as<double>();
  if (!calib["p_gps_in_imu"].IsSequence())
    throw YAMLParseError();
  p_gps_in_imu << calib["p_gps_in_imu"][0].as<double>(),
                  calib["p_gps_in_imu"][1].as<double>(),
                  calib["p_gps_in_imu"][2].as<double>();

  const YAML::Node& virtual_gps = yaml["virtual_gps"];
  if (!virtual_gps.IsMap())
    throw YAMLParseError();
  virtual_gps_sigma = virtual_gps["gps_sigma"].as<double>();
  imu_per_gps = virtual_gps["refresh_rate"].as<size_t>();

  const YAML::Node& others = yaml["others"];
  if (!others.IsMap())
    throw YAMLParseError();
  start = others["start"].as<size_t>();
  stop = others["stop"].as<size_t>();
  shut_up = others["shut_up"].as<bool>();
}

void LogTrajectory(const MSCKF& ekf, vector<vec3>* loc, vector<vec4>* ori) {
  loc->push_back(vec3(ekf.position().x(),
                      ekf.position().y(),
                      ekf.position().z()));
  JPL_Quaternion q = HamiltonToJPL(ekf.orientation());
  ori->push_back(vec4(q.x(), q.y(), q.z(), q.w()));
}

}

int main(int argc, char* argv[]) {
  /****************************************************************************
   * O. Process the arguments.
   ****************************************************************************/
  assert(argc > 1);
  try {
    LoadYAMLProfile(argv[1]);
  } catch (YAMLParseError e) {
    cerr << "[ " << setw(8) << "Error" << " ] " << e.what() << endl;
  }

  /****************************************************************************
   * I. Create a virtual tracker & Generate the first IMU and feature frame.
   ****************************************************************************/
  VirtualGPSTracker* vtracker = nullptr;
  try {
    VirtualGPSTracker::GPS gps;
    gps.R_gps_in_imu = R_gps_in_imu;
    gps.p_gps_in_imu = p_gps_in_imu;
    gps.noise = virtual_gps_sigma;
    vtracker = new VirtualGPSTracker(imu_filename, gt_filename, gps, start);
    if (shut_up)
      vtracker->ShutUp();
  } catch (FileNotFound e) {
    cerr << "[ " << setw(8) << "Error" << " ] " << e.what() << endl;
    exit(-1);
  }
  Vector3d gps;
  VirtualGPSTracker::Vector9d imu;
  double gps_timestamp = 0;
  double imu_timestamp = 0;
  bool has_gps = false;
  bool has_imu = false;
  has_gps = vtracker->NextGPS(&gps, &gps_timestamp);
  has_imu = vtracker->NextSensor(&imu, &imu_timestamp);

  /****************************************************************************
   * II. Set up the MSCKFs.
   ****************************************************************************/
  MSCKF ekf_ppg_only, ekf;
  ekf_ppg_only.setNoiseCov(Matrix3d::Identity() * sigma_ng * sigma_ng,
                           Matrix3d::Identity() * sigma_nwg * sigma_nwg,
                           Matrix3d::Identity() * sigma_na * sigma_na,
                           Matrix3d::Identity() * sigma_nwa * sigma_nwa, 0);
  ekf_ppg_only.initialize(HamiltonToJPL(INIT_ORIENTATION),
                          Vector3d::Zero(),
                          Vector3d{imu[6], imu[7], imu[8]},
                          Vector3d::Zero(),
                          Vector3d::Zero());
  ekf.setNoiseCov(Matrix3d::Identity() * sigma_ng * sigma_ng,
                  Matrix3d::Identity() * sigma_nwg * sigma_nwg,
                  Matrix3d::Identity() * sigma_na * sigma_na,
                  Matrix3d::Identity() * sigma_nwa * sigma_nwa, 0);
  ekf.initialize(HamiltonToJPL(INIT_ORIENTATION),
                 Vector3d::Zero(),
                 Vector3d{imu[6], imu[7], imu[8]},
                 Vector3d::Zero(),
                 Vector3d::Zero());

  /****************************************************************************
   * III. Prepare for the main loop.
   ****************************************************************************/
  vector<glm::vec3> loc_ppg, loc_upd;
  vector<glm::vec4> ori_ppg, ori_upd;
  has_gps = vtracker->NextGPS(&gps, &gps_timestamp);
  has_imu = vtracker->NextSensor(&imu, &imu_timestamp);
  double t_second = 0;

  /****************************************************************************
   * IV. Main loop.
   *     - On reading a new IMU, integrate it to both EFKs;
   *     - On reading a new feature frame, let the EFK decide
   *       whether to perform a update.
   ****************************************************************************/
  size_t imu_counter = 0;
  while (has_imu && imu_counter < stop - start) {
    t_second = imu_timestamp;

    Vector3d gyro {imu[0], imu[1], imu[2]};
    Vector3d acce {imu[3], imu[4], imu[5]};
    ekf.propagate(t_second, gyro, acce);
    ekf_ppg_only.propagate(t_second, gyro, acce);
    LogTrajectory(ekf_ppg_only, &loc_ppg, &ori_ppg);
    LogTrajectory(ekf, &loc_upd, &ori_upd);
    ++imu_counter;

    if (has_gps && imu_counter % imu_per_gps == 0) {
      /* perform an update */
      ekf.update(t_second, gps, sigma_gps);
      LogTrajectory(ekf, &loc_upd, &ori_upd);

      if (!vtracker->shut_up()) {
        cout << "\t================================================================\n"
             << "\t| IMU: " << setw(6) << imu_counter << " | "
             << "A new GPS received: " << gps.transpose() << " |\n"
             << "\t================================================================\n";
      }
    }

    has_imu = vtracker->NextSensor(&imu, &imu_timestamp);
    has_gps = vtracker->NextGPS(&gps, &gps_timestamp);
  }

  /****************************************************************************
   *  V. Draw the trajectories.
   ***************************************************************************/
  const list<VirtualGPSTracker::Pose>& true_poses = vtracker->true_pose_list();
  vector<glm::vec3> loc_gt;
  vector<glm::vec4> ori_gt;
  double quat[4] = {0};
  for (const auto& pose : true_poses) {
    /* gps orientation and position in global coordinates */
    Matrix3d R = pose.R * vtracker->virtual_gps().R_gps_in_imu;
    Vector3d t = pose.R * vtracker->virtual_gps().p_gps_in_imu + pose.t;
    utils::RotToJPLQuat(R.data(), quat);
    loc_gt.push_back(vec3(t.x(), t.y(), t.z()));
    ori_gt.push_back(vec4(quat[0], quat[1], quat[2], quat[3]));
  }

  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_ppg, ori_ppg);
  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_upd, ori_upd);
  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_gt, ori_gt);
  SLAMTrajectoryDrawer::SetupGLUT(argc, argv);
  SLAMTrajectoryDrawer::SetupGLEW();
  SLAMTrajectoryDrawer::SetupGLSL();
  SLAMTrajectoryDrawer::StartDrawing();
  SLAMTrajectoryDrawer::FreeTrajectory();
  SLAMTrajectoryDrawer::FreeLandmarks();
  SLAMTrajectoryDrawer::WriteTrajectoryToFile(loc_ppg, ori_ppg, (out_prefix + "ppg" + out_suffix).c_str());
  SLAMTrajectoryDrawer::WriteTrajectoryToFile(loc_upd, ori_upd, (out_prefix + "upd" + out_suffix).c_str());
  SLAMTrajectoryDrawer::WriteTrajectoryToFile(loc_gt, ori_gt, (out_prefix + "gt" + out_suffix).c_str());

  delete vtracker;
  return 0;
}

