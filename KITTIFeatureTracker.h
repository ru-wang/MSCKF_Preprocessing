#ifndef MSCKF_PREPROCESSING_KITTI_FEATURE_TRACKER_H_
#define MSCKF_PREPROCESSING_KITTI_FEATURE_TRACKER_H_

#include "GenericFeatureTracker.h"

#include <Eigen/Eigen>

#include <opencv2/core.hpp>

#include <exception>
#include <string>
#include <utility>

class KITTIFeatureTracker : public GenericFeatureTracker {
 public:
  KITTIFeatureTracker(const std::string& path,
                      const std::string& timestamp_filename,
                      const std::string& imu_filename = "") throw(std::runtime_error);
  virtual ~KITTIFeatureTracker() {};

  virtual bool NextImage(cv::Mat* image_out, double* timestamp_out);
  virtual bool NextSensor(Vector9d* sensor_out, double* timestamp_out);

 private:
  std::string image_path_;
  std::vector<std::pair<long, std::string>> image_file_list_;
  std::vector<std::pair<long, std::string>>::const_iterator image_file_cursor_;

  std::string image_timestamp_filename_;
  std::vector<double> image_timestamp_list_;
  std::vector<double>::const_iterator image_timestamp_cursor_;

  std::string imu_filename_;
  std::vector<Vector9d> imu_list_;
  std::vector<Vector9d>::const_iterator imu_cursor_;

  std::vector<double> imu_timestamp_list_;
  std::vector<double>::const_iterator imu_timestamp_cursor_;
};

#endif  // MSCKF_PREPROCESSING_KITTI_FEATURE_TRACKER_H_
