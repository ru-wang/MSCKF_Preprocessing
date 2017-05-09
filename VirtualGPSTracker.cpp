#include "VirtualGPSTracker.h"

#include "Exception.h"
#include "Utils.h"

#include "MSCKF_Simulation/Helpers.h"

#include <boost/filesystem.hpp>
#include <Eigen/Eigen>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace Eigen;

namespace fs = ::boost::filesystem;

VirtualGPSTracker::VirtualGPSTracker(const string& imu_filename,
                                     const string& gt_filename,
                                     const GPS& gps,
                                           size_t start) throw(FileNotFound)
    : GenericFeatureTracker(Matrix3d::Identity()),
      shut_up_(false),
      virtual_gps_(gps),
      imu_filename_(imu_filename),
      groundtruth_filename_(gt_filename) {
  if (!fs::exists(imu_filename_))
    throw FileNotFound(imu_filename_, "IMU file");
  if (!fs::exists(groundtruth_filename_))
    throw FileNotFound(groundtruth_filename_, "Ground truth file");

  LoadIMUFromFile(start);
  LoadGroundTruthFromFile(start);

  if (!shut_up_) {
    cout << "[ Read\t] Total IMUs: " << imu_list_.size() << endl;
    cout << "[ Read\t] Total true poses: " << true_pose_list_.size() << endl;
  }
}

bool VirtualGPSTracker::NextSensor(Vector9d* sensor_out, double* timestamp_out) {
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

bool VirtualGPSTracker::NextGPS(Vector3d* gps_out, double* timestamp_out) {
  if (true_pose_cursor_ == true_pose_list_.cend() ||
      true_pose_timestamp_cursor_ == true_pose_timestamp_list_.cend()) {
    return false;
  } else {
    Pose true_pose = *true_pose_cursor_;
    double timestamp = *true_pose_timestamp_cursor_;

    *gps_out = true_pose.t + virtual_gps_.noise.vector3();
    *timestamp_out = timestamp;
    ++true_pose_timestamp_cursor_;
    ++true_pose_cursor_;

    if (!shut_up_) {
      cout << setprecision(4) << fixed
           << "[ " << timestamp << " ] GPS reading: " << true_pose.t.transpose() << endl;
    }
    return true;
  }
}

void VirtualGPSTracker::LoadIMUFromFile(size_t start) {
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

void VirtualGPSTracker::LoadGroundTruthFromFile(size_t start) {
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
