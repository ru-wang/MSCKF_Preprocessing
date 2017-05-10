/*
 * Unit test for MSCKF.
 * Use KITTI dataset to propagate, simulation landmarks to update.
 * Propagate and update.
 */

#include "SLAMTrajectoryDrawer.h"
#include "VirtualFeatureTracker.h"
#include "Utils.h"

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
double sigma_nim;

Matrix3d R_cam_in_imu;
Vector3d p_cam_in_imu;
Matrix3d K;

Vector2d FOV_u, FOV_v;
Vector2d DOF;
size_t imu_per_image;
size_t landmark_num;

size_t start, stop;
bool shut_up;
bool use_inverse_depth;

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
  sigma_nim = sigma["image_noise"].as<double>();

  const YAML::Node& calib = yaml["calibration"];
  if (!calib.IsMap())
    throw YAMLParseError();
  if (!calib["R_cam_in_imu"].IsSequence())
    throw YAMLParseError();
  R_cam_in_imu << calib["R_cam_in_imu"][0].as<double>(),
                  calib["R_cam_in_imu"][1].as<double>(),
                  calib["R_cam_in_imu"][2].as<double>(),
                  calib["R_cam_in_imu"][3].as<double>(),
                  calib["R_cam_in_imu"][4].as<double>(),
                  calib["R_cam_in_imu"][5].as<double>(),
                  calib["R_cam_in_imu"][6].as<double>(),
                  calib["R_cam_in_imu"][7].as<double>(),
                  calib["R_cam_in_imu"][8].as<double>();
  if (!calib["p_cam_in_imu"].IsSequence())
    throw YAMLParseError();
  p_cam_in_imu << calib["p_cam_in_imu"][0].as<double>(),
                  calib["p_cam_in_imu"][1].as<double>(),
                  calib["p_cam_in_imu"][2].as<double>();
  if (!calib["K"].IsMap())
    throw YAMLParseError();
  K << calib["K"]["fx"].as<double>(),  calib["K"]["s"].as<double>(), calib["K"]["cx"].as<double>(),
                                   0, calib["K"]["fy"].as<double>(), calib["K"]["cy"].as<double>(),
                                   0,                             0,                             1;

  const YAML::Node& virtual_camera = yaml["virtual_camera"];
  if (!virtual_camera.IsMap())
    throw YAMLParseError();
  if (!virtual_camera["field_of_view_u"].IsSequence() ||
      !virtual_camera["field_of_view_u"].IsSequence() ||
      !virtual_camera["depth_of_field"].IsSequence())
    throw YAMLParseError();
  FOV_u << virtual_camera["field_of_view_u"][0].as<double>(),
           virtual_camera["field_of_view_u"][1].as<double>();
  FOV_v << virtual_camera["field_of_view_v"][0].as<double>(),
           virtual_camera["field_of_view_v"][1].as<double>();
  DOF << virtual_camera["depth_of_field"][0].as<double>(),
         virtual_camera["depth_of_field"][1].as<double>();
  imu_per_image = virtual_camera["refresh_rate"].as<size_t>();
  landmark_num = virtual_camera["landmark_number"].as<size_t>();

  const YAML::Node& others = yaml["others"];
  if (!others.IsMap())
    throw YAMLParseError();
  start = others["start"].as<size_t>();
  stop = others["stop"].as<size_t>();
  shut_up = others["shut_up"].as<bool>();
  use_inverse_depth = others["use_inverse_depth"].as<bool>();
}

void LogTrajectory(const MSCKF& ekf, vector<vec3>* loc, vector<vec4>* ori) {
  loc->push_back(vec3(ekf.cameraPosition().x(),
                      ekf.cameraPosition().y(),
                      ekf.cameraPosition().z()));
  JPL_Quaternion q = HamiltonToJPL(ekf.cameraOrientation());
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
  VirtualFeatureTracker* vtracker = nullptr;
  try {
    VirtualFeatureTracker::Camera camera;
    camera.R_cam_in_imu = R_cam_in_imu;
    camera.p_cam_in_imu = p_cam_in_imu;
    camera.K = K;
    camera.K_inv = K.inverse();
    camera.FOV_u = FOV_u;
    camera.FOV_v = FOV_v;
    camera.DOF = DOF;
    camera.noise = sigma_nim;
    vtracker = new VirtualFeatureTracker(imu_filename, gt_filename, camera, start, landmark_num);
    if (shut_up)
      vtracker->ShutUp();
  } catch (FileNotFound e) {
    cerr << "[ " << setw(8) << "Error" << " ] " << e.what() << endl;
    exit(-1);
  }
  VirtualFeatureTracker::Pose true_pose;
  VirtualFeatureTracker::Vector9d imu;
  double img_timestamp = 0;
  double imu_timestamp = 0;
  bool has_img = false;
  bool has_imu = false;
  has_img = vtracker->NextImage(&true_pose, &img_timestamp);
  has_imu = vtracker->NextSensor(&imu, &imu_timestamp);

  /****************************************************************************
   * II. Set up the MSCKFs.
   ****************************************************************************/
  MSCKF ekf_ppg_only(vtracker->virtual_camera().R_cam_in_imu.transpose(),
                     vtracker->virtual_camera().p_cam_in_imu,
                     use_inverse_depth);
  MSCKF ekf(vtracker->virtual_camera().R_cam_in_imu.transpose(),
            vtracker->virtual_camera().p_cam_in_imu,
            use_inverse_depth);
  ekf_ppg_only.setNoiseCov(Matrix3d::Identity() * sigma_ng * sigma_ng,
                           Matrix3d::Identity() * sigma_nwg * sigma_nwg,
                           Matrix3d::Identity() * sigma_na * sigma_na,
                           Matrix3d::Identity() * sigma_nwa * sigma_nwa,
                           sigma_nim * sigma_nim);
  ekf_ppg_only.initialize(HamiltonToJPL(INIT_ORIENTATION),
                          Vector3d::Zero(),
                          Vector3d{imu[6], imu[7], imu[8]},
                          Vector3d::Zero(),
                          Vector3d::Zero());
  ekf.setNoiseCov(Matrix3d::Identity() * sigma_ng * sigma_ng,
                  Matrix3d::Identity() * sigma_nwg * sigma_nwg,
                  Matrix3d::Identity() * sigma_na * sigma_na,
                  Matrix3d::Identity() * sigma_nwa * sigma_nwa,
                  sigma_nim * sigma_nim);
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
  has_img = vtracker->NextImage(&true_pose, &img_timestamp);
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

    if (has_img && imu_counter % imu_per_image == 0) {
      /* match features with the last frame and return the FeatureFrame */
      FeatureMatcher::FeatureFrame frame;
      frame = vtracker->ConstructFeatureFrame(true_pose);

      /* perform an update */
      ekf.update(t_second, frame);
      LogTrajectory(ekf, &loc_upd, &ori_upd);

      if (!vtracker->shut_up()) {
        cout << "\t======================================================\n"
             << "\t| IMU: " << setw(6) << imu_counter << " | "
             << "A new frame containing " << setw(4) << frame.size() << " features |\n"
             << "\t======================================================\n";
      } else if (imu_counter / imu_per_image % 100 == 0) {
        cout << "\t======================================================\n"
             << "\t| IMU: " << setw(6) << imu_counter << " | "
             << "A new frame containing " << setw(4) << frame.size() << " features |\n"
             << "\t======================================================\n";
      }
    }

    has_imu = vtracker->NextSensor(&imu, &imu_timestamp);
    has_img = vtracker->NextImage(&true_pose, &img_timestamp);
  }
                                                                                                 
  /****************************************************************************
   *  V. Draw the trajectories.
   ***************************************************************************/
  const list<VirtualFeatureTracker::Pose>& true_poses = vtracker->true_pose_list();
  vector<glm::vec3> loc_gt;
  vector<glm::vec4> ori_gt;
  double quat[4] = {0};
  for (const auto& true_pose : true_poses) {
    /* camera orientation and position in global coordinates */
    Matrix3d R = true_pose.R * vtracker->virtual_camera().R_cam_in_imu;
    Vector3d t = true_pose.R * vtracker->virtual_camera().p_cam_in_imu + true_pose.t;
    utils::RotToJPLQuat(R.data(), quat);
    loc_gt.push_back(vec3(t.x(), t.y(), t.z()));
    ori_gt.push_back(vec4(quat[0], quat[1], quat[2], quat[3]));
  }

  vector<glm::vec3> landmarks;
  for (const auto& landmark : vtracker->landmarks()) {
    landmarks.push_back({landmark.x(), landmark.y(), landmark.z()});
  }

  SLAMTrajectoryDrawer::ReadLandmarksFrom(landmarks);
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

