#ifndef MSCKF_PREPROCESSING_FEATURES_MATCHER_H_
#define MSCKF_PREPROCESSING_FEATURES_MATCHER_H_

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

class FeatureMatcher {
 public:
  // [<feature_id> in new frame] --> [<feature_id> in old frame, <x, y> in new frame]
  // where <x, y> are the pixel coordinates with intrinsics eliminated
  typedef std::unordered_map<size_t, std::pair<size_t, Eigen::Vector2d>> FeatureFrame;

  FeatureMatcher(const Eigen::Matrix3d& intrinsics, float hamming_threshold, size_t max_features_per_frame)
      : kIntrinsics(intrinsics), kIntrinsicsInv(intrinsics.inverse()),
        kHammingDistThreshold(hamming_threshold),
        kMaxFeatureNumPerFrame(max_features_per_frame),
        orb_matcher_(cv::NORM_HAMMING) {}

  void SetInitialDescriptors(const cv::Mat& descriptors) {
    orb_matcher_.add(std::vector<cv::Mat> {descriptors});
  }

  std::vector<std::vector<cv::DMatch>>
  MatchORBWithOldFrame(const std::vector<cv::KeyPoint>& features, const cv::Mat& descriptors, FeatureFrame* frame) {
    // Apply Brute-force matching method
    std::vector<std::vector<cv::DMatch>> matches;
    const cv::Mat& old_descriptors = orb_matcher_.getTrainDescriptors().back();
    orb_matcher_.radiusMatch(descriptors, old_descriptors, matches, kHammingDistThreshold);

    // Update feature set & Generate feature frame
    for (size_t local_feature_id = 0; local_feature_id < matches.size(); ++local_feature_id) {
      size_t cam_id = orb_matcher_.getTrainDescriptors().size();
      size_t new_feature_id = cam_id * kMaxFeatureNumPerFrame + local_feature_id;
      size_t old_feature_id;

      const std::vector<cv::DMatch> match_list = matches[local_feature_id];
      if (match_list.empty()) {  // Feature not matched in old frame
        old_feature_id = new_feature_id;
      } else {
        const cv::DMatch& match = match_list.front();
        old_feature_id = (cam_id - 1) * kMaxFeatureNumPerFrame + match.trainIdx;
      }
      const cv::Point2f& pt_2d = features[local_feature_id].pt;

      // Remove intrinsics
      Eigen::Vector3d coord_3d = kIntrinsicsInv * Eigen::Vector3d(pt_2d.x, pt_2d.y, 1);
      Eigen::Vector2d coord_2d(coord_3d.x() / coord_3d.z(), coord_3d.y() / coord_3d.z());
      (*frame)[new_feature_id] = std::make_pair(old_feature_id, coord_2d);
    }

    // Update train descriptors
    orb_matcher_.add(std::vector<cv::Mat> {descriptors});

    return matches;
  }

 private:
  const Eigen::Matrix3d kIntrinsics;
  const Eigen::Matrix3d kIntrinsicsInv;
  const float kHammingDistThreshold;
  const size_t kMaxFeatureNumPerFrame;

  cv::BFMatcher orb_matcher_;
};

#endif  // MSCKF_PREPROCESSING_FEATURES_MATCHER_H_
