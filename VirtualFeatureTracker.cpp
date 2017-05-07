#include "VirtualFeatureTracker.h"

#include "Exception.h"
#include "Utils.h"

#include "MSCKF_Simulation/Helpers.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

VirtualFeatureTracker::VirtualFeatureTracker(const string& imu_filename,
                                             const string& gt_filename,
                                             const Camera& camera,
                                                   size_t start,
                                                   size_t landmark_num) throw(FileNotFound)
    : GenericFeatureTracker(Matrix3d::Identity()),
      shut_up_(false),
      virtual_camera_(camera),
      imu_filename_(imu_filename),
      groundtruth_filename_(gt_filename) {
  if (!fs::exists(imu_filename_))
    throw FileNotFound(imu_filename_, "IMU file");
  if (!fs::exists(groundtruth_filename_))
    throw FileNotFound(groundtruth_filename_, "Ground truth file");

  LoadIMUFromFile(start);
  LoadGroundTruthFromFile(start);

  Vector3d upper_left_forward, lower_right_backward;
  Vector3d bbox_margin(virtual_camera_.DOF.y(), virtual_camera_.DOF.y(), virtual_camera_.DOF.y());
  CalculateGTBoundingBox(&upper_left_forward, &lower_right_backward);
  GenerateRandomLandmarks(upper_left_forward + bbox_margin,
                          lower_right_backward - bbox_margin,
                          landmark_num);

  if (!shut_up_) {
    cout << "[ Read\t] Total IMUs: " << imu_list_.size() << endl;
    cout << "[ Read\t] Total true poses: " << true_pose_list_.size() << endl;
  }
}

bool VirtualFeatureTracker::NextSensor(Vector9d* sensor_out, double* timestamp_out) {
  if (imu_cursor_ == imu_list_.cend() ||
      imu_timestamp_cursor_ == imu_timestamp_list_.cend()) {
    return false;
  }

  /* read a new IMU */
  Vector9d imu = *imu_cursor_;

  /* read a new timestamp */
  double timestamp = *imu_timestamp_cursor_;

  *timestamp_out = timestamp;
  *sensor_out = imu;
  ++imu_cursor_;
  ++imu_timestamp_cursor_;

  if (!shut_up_) {
    cout << setprecision(4) << fixed
         << "[ " << timestamp << " ] IMU reading: " << imu.transpose() << endl;
  }
  return true;
}

bool VirtualFeatureTracker::NextImage(Pose* true_pose_out, double* timestamp_out) {
  if (true_pose_cursor_ == true_pose_list_.cend() ||
      true_pose_timestamp_cursor_ == true_pose_timestamp_list_.cend()) {
    return false;
  } else {
    Pose true_pose = *true_pose_cursor_;
    double timestamp = *true_pose_timestamp_cursor_;

    *true_pose_out = true_pose;
    *timestamp_out = timestamp;
    ++true_pose_timestamp_cursor_;
    ++true_pose_cursor_;

    if (!shut_up_) {
      cout << setprecision(4) << fixed
           << "[ " << timestamp << " ] Processing: Image at ( " << true_pose.t.transpose() << " )" << endl;
    }
    return true;
  }
}

/* TODO update feature tracks */
FeatureMatcher::FeatureFrame& VirtualFeatureTracker::ConstructFeatureFrame(const Pose& true_pose) {
  /* match with last frame */
  frame_list_.push_back(FeatureMatcher::FeatureFrame());
  FeatureMatcher::FeatureFrame& frame = frame_list_.back();

  const Matrix3d& C_G_to_imu = true_pose.R;
  const Vector3d& p_imu_in_G = true_pose.t;
  const Matrix3d& C_imu_to_cam = virtual_camera_.R_cam_in_imu;
  const Vector3d& p_cam_in_imu = virtual_camera_.p_cam_in_imu;

  double fx = virtual_camera_.K(0, 0);
  double fy = virtual_camera_.K(1, 1);
  double cx = virtual_camera_.K(0, 2);
  double cy = virtual_camera_.K(1, 2);
  for (size_t feature_id = 0; feature_id < landmarks_.size(); ++feature_id) {
    const Vector3d& landmark = landmarks_[feature_id];
    Vector3d p_in_imu = C_G_to_imu.transpose() * (landmark - p_imu_in_G);
    Vector3d p_in_cam = C_imu_to_cam.transpose() * (p_in_imu - p_cam_in_imu);
    if (p_in_cam.z() >= virtual_camera_.DOF.x() &&
        p_in_cam.z() <= virtual_camera_.DOF.y()) {
      Vector2d u(fx * p_in_cam.x() / p_in_cam.z() + cx + virtual_camera_.noise.value(),
                 fy * p_in_cam.y() / p_in_cam.z() + cy + virtual_camera_.noise.value());
      if (u.x() >= virtual_camera_.FOV_u.x() &&
          u.x() <= virtual_camera_.FOV_u.y() &&
          u.y() >= virtual_camera_.FOV_v.x() &&
          u.y() <= virtual_camera_.FOV_v.y()) {
        Vector3d z = virtual_camera_.K_inv * Vector3d(u.x(), u.y(), 1);
        frame[feature_id] = make_pair(feature_id, Vector2d(z.x() / z.z(), z.y() / z.z()));
      }
    }
  }

  return frame;
}

void VirtualFeatureTracker::LoadIMUFromFile(size_t start) {
  ifstream imu_ifs(imu_filename_);
  string imu_line_str;
  vector<double> imu_line;
  Vector9d imu;
  double timestamp;
  while (!imu_ifs.eof()) {
    utils::SafelyGetLine(imu_ifs, &imu_line_str);
    if (imu_line_str.empty())
      continue;
    imu_line = utils::SplitToVector<double>(imu_line_str);
    imu << imu_line[1], imu_line[2], imu_line[3],  /* gyro_x, gyro_y, gyro_z */
           imu_line[4], imu_line[5], imu_line[6],  /* acce_x, acce_y, acce_z */
           imu_line[7], imu_line[8], imu_line[9];  /* velo_x, velo_y, velo_z */
    timestamp = imu_line.front();
    imu_list_.push_back(imu);
    imu_timestamp_list_.push_back(timestamp);
  }
  imu_ifs.close();
  for (size_t i = 0; i < start; ++i) {
    imu_list_.pop_front();
    imu_timestamp_list_.pop_front();
  }
  imu_cursor_ = imu_list_.cbegin();
  imu_timestamp_cursor_ = imu_timestamp_list_.cbegin();
}

void VirtualFeatureTracker::CalculateGTBoundingBox(Vector3d* upper_left_forward,
                                                   Vector3d* lower_right_backward) {
  *upper_left_forward = true_pose_list_.front().t;
  *lower_right_backward = true_pose_list_.front().t;
  for (const Pose& pose : true_pose_list_) {
    const Vector3d& p = pose.t;
    p.x() > upper_left_forward->x()   ? upper_left_forward->x() = p.x()   : 
    p.x() < lower_right_backward->x() ? lower_right_backward->x() = p.x() : 0;
    p.y() > upper_left_forward->y()   ? upper_left_forward->y() = p.y()   : 
    p.y() < lower_right_backward->y() ? lower_right_backward->y() = p.y() : 0;
    p.z() > upper_left_forward->z()   ? upper_left_forward->z() = p.z()   : 
    p.z() < lower_right_backward->z() ? lower_right_backward->z() = p.z() : 0;
  }
}

void VirtualFeatureTracker::LoadGroundTruthFromFile(size_t start) {
  ifstream gt_ifs(groundtruth_filename_);
  string gt_line_str;
  vector<vector<double>> gt_lines;
  while (!gt_ifs.eof()) {
    utils::SafelyGetLine(gt_ifs, &gt_line_str);
    if (gt_line_str.empty())
      continue;
    gt_lines.push_back(utils::SplitToVector<double>(gt_line_str));
  }
  gt_ifs.close();

  Pose pose, first_pose_inv;
  double timestamp;
  double scale = utils::LatToScale(gt_lines[start][1]);
  first_pose_inv.R = (utils::OXTSEulerToRotationMatrix(gt_lines[start][4],
                                                       gt_lines[start][5],
                                                       gt_lines[start][6])).transpose();
  first_pose_inv.t = -first_pose_inv.R * utils::LatLonToMercator(gt_lines[start][1],
                                                                 gt_lines[start][2],
                                                                 gt_lines[start][3],
                                                                 scale);
  for (auto it = gt_lines.begin() + start; it != gt_lines.end(); ++it) {
    const auto& gt_line = *it;
    pose.R = first_pose_inv.R * utils::OXTSEulerToRotationMatrix(gt_line[4],
                                                                 gt_line[5],
                                                                 gt_line[6]);
    pose.t = first_pose_inv.R * utils::LatLonToMercator(gt_line[1],
                                                        gt_line[2],
                                                        gt_line[3],
                                                        scale) + first_pose_inv.t;
    timestamp = gt_line.front();
    true_pose_list_.push_back(pose);
    true_pose_timestamp_list_.push_back(timestamp);
  }
  true_pose_cursor_ = true_pose_list_.cbegin();
  true_pose_timestamp_cursor_ = true_pose_timestamp_list_.cbegin();
}

void VirtualFeatureTracker::GenerateRandomLandmarks(const Vector3d& upper_left_forward,
                                                    const Vector3d& lower_right_backward,
                                                    const size_t number) {
  Uniform random(0, 1);
  for (size_t i  = 0; i < number; ++i) {
    Vector3d random_vector = random.vector3();
    Matrix3d random_diag = random_vector.asDiagonal();
    Vector3d landmark = random_diag * (upper_left_forward - lower_right_backward);
    landmark += lower_right_backward;
    landmarks_.push_back(landmark);
  }
}
