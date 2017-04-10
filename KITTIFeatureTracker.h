#ifndef MSCKF_PREPROCESSING_KITTI_FEATURE_TRACKER_H_
#define MSCKF_PREPROCESSING_KITTI_FEATURE_TRACKER_H_

#include "FeatureMatcher.h"

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <exception>
#include <map>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

class KITTIFeatureTracker {
 public:
  /*
   * FeatureTrackMap:
   *   [<global_feature_id>] --> [<FeatureTrack>]
   *
   * FeatureTrack:
   *
   *   [<feature_id> in frame 1, <x, y>]  |
   *   [<feature_id> in frame 2, <x, y>]  |
   *   [<feature_id> in frame 3, <x, y>]  |
   *                  .                   |
   *                  .                   |
   *                  .                   |
   *   [<feature_id> in frame N, <x, y>]  V
   *
   * where the <feature_id> in frame 1 is the <global_feature_id>.
   */
  typedef std::vector<std::pair<size_t, Eigen::Vector2d>> FeatureTrack;
  typedef std::map<size_t, FeatureTrack> FeatureTrackMap;
  typedef Eigen::Matrix<double, 9, 1> Vector9d;

  KITTIFeatureTracker(const std::string& path,
                      const std::string& timestamp_filename,
                      const std::string& imu_filename = "") throw(std::runtime_error);

  bool NextImage(cv::Mat* image_out, double* timestamp_out);
  bool NextSensor(Vector9d* sensor_out, double* timestamp_out);

  FeatureMatcher::FeatureFrame&
  ConstructFeatureFrame(const std::vector<Eigen::Vector2d>& features, const cv::Mat& descriptors,
                              std::vector<cv::DMatch>* matches_out = nullptr);

  const std::vector<FeatureMatcher::FeatureFrame>& frame_list() const { return frame_list_; }
  const FeatureTrackMap& tracks() const { return tracks_; }
  const FeatureMatcher& fmatcher() const { return fmatcher_; }

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

  std::vector<FeatureMatcher::FeatureFrame> frame_list_;
  FeatureTrackMap tracks_;
  FeatureMatcher fmatcher_;

  /*
   * Feature ID lookup table
   * [<local_feature_id>] --> [<global_feature_id>]
   */
  std::unordered_map<size_t, size_t> feature_ID_TLB_;
};

#endif  // MSCKF_PREPROCESSING_KITTI_FEATURE_TRACKER_H_
