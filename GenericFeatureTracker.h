#ifndef MSCKF_PREPROCESSING_GENERIC_FEATURE_TRACKER_H_
#define MSCKF_PREPROCESSING_GENERIC_FEATURE_TRACKER_H_

#include "FeatureMatcher.h"

#include <Eigen/Eigen>

#include <opencv2/core.hpp>

#include <map>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

class GenericFeatureTracker {
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

  GenericFeatureTracker(const Eigen::Matrix3d& K);
  virtual ~GenericFeatureTracker() = 0;

  virtual bool NextImage(cv::Mat* image_out, double* timestamp_out) = 0;
  virtual bool NextSensor(Vector9d* sensor_out, double* timestamp_out) = 0;

  FeatureMatcher::FeatureFrame&
  ConstructFeatureFrame(const std::vector<Eigen::Vector2d>& features,
                        const cv::Mat& descriptors,
                              std::vector<cv::DMatch>* matches_out = nullptr);

  const std::vector<FeatureMatcher::FeatureFrame>& frame_list() const { return frame_list_; }
  const FeatureTrackMap& tracks() const { return tracks_; }
  const FeatureMatcher& fmatcher() const { return fmatcher_; }

 protected:
  std::vector<FeatureMatcher::FeatureFrame> frame_list_;
  FeatureTrackMap tracks_;
  FeatureMatcher fmatcher_;

  /*
   * Feature ID lookup table.
   * [<local_feature_id>] --> [<global_feature_id>]
   */
  std::unordered_map<size_t, size_t> feature_ID_TLB_;
};

#endif  // MSCKF_PREPROCESSING_GENERIC_FEATURE_TRACKER_H_
