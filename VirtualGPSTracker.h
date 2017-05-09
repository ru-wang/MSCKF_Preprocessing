#ifndef MSCKF_PREPROCESSING_VIRTUAL_GPS_TRACKER_H_
#define MSCKF_PREPROCESSING_VIRTUAL_GPS_TRACKER_H_

#include "Exception.h"
#include "GenericFeatureTracker.h"

#include "MSCKF_Simulation/Helpers.h"

#include <Eigen/Eigen>

#include <opencv2/core.hpp>

#include <list>
#include <string>

class VirtualGPSTracker : public GenericFeatureTracker {
 public:
  struct GPS {
    GPS() : R_gps_in_imu(Eigen::Matrix3d::Identity()),
            p_gps_in_imu(0, 0, 0),
            noise(0.05) {}

    Eigen::Matrix3d R_gps_in_imu;
    Eigen::Vector3d p_gps_in_imu;
    Gaussian noise;  /* Gaussian noise for images */
  };

  VirtualGPSTracker(const std::string& imu_filename,
                    const std::string& gt_filename,
                    const GPS& gps,
                          size_t start = 0) throw(FileNotFound);

  virtual ~VirtualGPSTracker() {};

  virtual bool NextSensor(Vector9d* sensor_out, double* timestamp_out) override;
  bool NextGPS(Eigen::Vector3d* gps_out, double* timestamp_out);

  void ShutUp() { shut_up_ = true; }

  bool shut_up() const { return shut_up_; }

  const GPS& virtual_gps() const { return virtual_gps_; }
        GPS& virtual_gps()       { return virtual_gps_; }

  const std::list<Pose>& true_pose_list() const { return true_pose_list_; };

 private:
  /*
   * Suppress the output messages.
   */
  bool shut_up_;

  GPS virtual_gps_;

  void LoadIMUFromFile(size_t start);
  void LoadGroundTruthFromFile(size_t start);

  std::string imu_filename_;
  std::list<Vector9d> imu_list_;
  std::list<Vector9d>::const_iterator imu_cursor_;

  std::list<double> imu_timestamp_list_;
  std::list<double>::const_iterator imu_timestamp_cursor_;

  /* 
   * Ground truth of IMU orientation and position in global coordinates.
   */
  std::string groundtruth_filename_;

  std::list<Pose> true_pose_list_;
  std::list<Pose>::const_iterator true_pose_cursor_;

  std::list<double> true_pose_timestamp_list_;
  std::list<double>::const_iterator true_pose_timestamp_cursor_;

  virtual bool NextImage(cv::Mat*, double*) override { return false; }
};

#endif  // MSCKF_PREPROCESSING_VIRTUAL_GPS_TRACKER_H_
