#include "GenericFeatureTracker.h"

#include "FeatureMatcher.h"
#include "Features.h"

#include <Eigen/Eigen>

#include <opencv2/core.hpp>

#include <map>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace {

const float HAMMING_DIST_THRESHOLD = 16;
const float L2_DIST_THRESHOLD = 50;

}

GenericFeatureTracker::GenericFeatureTracker()
    : fmatcher_(Matrix3d::Identity(),
                HAMMING_DIST_THRESHOLD,
                FeatureUtils::kORBFeatureNum,
                FeatureUtils::kORBNormType,
                true) {}

GenericFeatureTracker::~GenericFeatureTracker() {}

FeatureMatcher::FeatureFrame&
GenericFeatureTracker::ConstructFeatureFrame(const vector<Vector2d>& features,
                                             const Mat& descriptors,
                                                   vector<DMatch>* matches_out) {
  vector<DMatch> matches;

  /* Match with last frame */
  bool is_first_frame = frame_list_.empty();
  frame_list_.push_back(FeatureMatcher::FeatureFrame());
  FeatureMatcher::FeatureFrame& frame = frame_list_.back();
  if (is_first_frame) {
    fmatcher_.SetInitialDescriptors(descriptors);
    fmatcher_.MakeFirstFeatureFrame(features, &frame);
  } else {
    matches = fmatcher_.MatchFeaturesWithOldFrame(features, descriptors, &frame);
  }

  /* Update feature tracks */
  for (auto it = frame.cbegin(); it != frame.cend(); ++it) {
    size_t new_feature_id = it->first;
    size_t old_feature_id = it->second.first;
    const Vector2d& coord_2d = it->second.second;

    /* This feature is seen the first time */
    if (new_feature_id == old_feature_id) {
      size_t global_feature_id = new_feature_id;

      /* Update feature ID TLB */
      auto new_entry = feature_ID_TLB_.find(new_feature_id);
      assert(new_entry == feature_ID_TLB_.end());
      feature_ID_TLB_[new_feature_id] = global_feature_id;

      tracks_[global_feature_id].emplace_back(new_feature_id, coord_2d);
    } else {
      /* Lookup for the corresponding global_feature_id */
      auto old_entry = feature_ID_TLB_.find(old_feature_id);
      assert(old_entry != feature_ID_TLB_.end());
      size_t global_feature_id = old_entry->second;

      /* Update feature ID TLB */
      auto new_entry = feature_ID_TLB_.find(new_feature_id);
      assert(new_entry == feature_ID_TLB_.end());
      feature_ID_TLB_[new_feature_id] = global_feature_id;
      
      tracks_[global_feature_id].emplace_back(new_feature_id, coord_2d);
    }
  }

  if (matches_out)
    *matches_out = matches;
  return frame;
}
