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
  /*
   * [<feature_id> in new frame] --> [<feature_id> in old frame, <x, y> in new frame]
   * where <x, y> are the pixel coordinates with intrinsics eliminated
   */
  typedef std::unordered_map<size_t, std::pair<size_t, Eigen::Vector2d>> FeatureFrame;

  FeatureMatcher(const Eigen::Matrix3d& intrinsics, float norm_dist_threshold, size_t max_features_per_frame,
                 int norm_type, bool cross_check = false)
      : kIntrinsics(intrinsics), kIntrinsicsInv(intrinsics.inverse()),
        kNormDistThreshold(norm_dist_threshold),
        kMaxFeatureNumPerFrame(max_features_per_frame),
        matcher_(norm_type, cross_check) {}

  void SetInitialDescriptors(const cv::Mat& descriptors) {
    matcher_.add(std::vector<cv::Mat> {descriptors});
  }

  void MakeFirstFeatureFrame(const std::vector<Eigen::Vector2d>& features, FeatureFrame* frame) {
    for (size_t local_feature_id = 0; local_feature_id < features.size(); ++local_feature_id) {
      const Eigen::Vector2d& pt_2d = features[local_feature_id];

      /* Remove intrinsics */
      Eigen::Vector3d coord_3d = kIntrinsicsInv * Eigen::Vector3d(pt_2d.x(), pt_2d.y(), 1);
      Eigen::Vector2d coord_2d(coord_3d.x() / coord_3d.z(), coord_3d.y() / coord_3d.z());
      (*frame)[local_feature_id] = std::make_pair(local_feature_id, coord_2d);
    }
  }

  std::vector<cv::DMatch>
  MatchFeaturesWithOldFrame(const std::vector<Eigen::Vector2d>& features, const cv::Mat& descriptors, FeatureFrame* frame) {
    /* Apply Brute-force matching method */
    std::vector<std::vector<cv::DMatch>> matches;
    const cv::Mat& old_descriptors = matcher_.getTrainDescriptors().back();
    matcher_.knnMatch(descriptors, old_descriptors, matches, 1);

    std::vector<cv::DMatch> squeezed_matches;

    /* Update feature set & Generate feature frame */
    for (size_t local_feature_id = 0; local_feature_id < matches.size(); ++local_feature_id) {
      size_t cam_id = matcher_.getTrainDescriptors().size();
      size_t new_feature_id = cam_id * kMaxFeatureNumPerFrame + local_feature_id;
      size_t old_feature_id;

      const std::vector<cv::DMatch> match_list = matches[local_feature_id];
      if (match_list.empty() || match_list.front().distance > kNormDistThreshold) {
        /* Feature not matched in old frame */
        old_feature_id = new_feature_id;
      } else {
        /* Feature matched */
        const cv::DMatch& match = match_list.front();
        old_feature_id = (cam_id - 1) * kMaxFeatureNumPerFrame + match.trainIdx;
        squeezed_matches.push_back(match);
      }
      const Eigen::Vector2d& pt_2d = features[local_feature_id];

      /* Remove intrinsics */
      Eigen::Vector3d coord_3d = kIntrinsicsInv * Eigen::Vector3d(pt_2d.x(), pt_2d.y(), 1);
      Eigen::Vector2d coord_2d(coord_3d.x() / coord_3d.z(), coord_3d.y() / coord_3d.z());
      (*frame)[new_feature_id] = std::make_pair(old_feature_id, coord_2d);
    }

    /* Update train descriptors */
    matcher_.add(std::vector<cv::Mat> {descriptors});

    return squeezed_matches;
  }

 private:
  const Eigen::Matrix3d kIntrinsics;
  const Eigen::Matrix3d kIntrinsicsInv;
  const float kNormDistThreshold;
  const size_t kMaxFeatureNumPerFrame;

  cv::BFMatcher matcher_;
};

#endif  // MSCKF_PREPROCESSING_FEATURES_MATCHER_H_
