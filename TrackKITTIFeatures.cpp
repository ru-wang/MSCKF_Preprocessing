#include "Features.h"
#include "FeatureMatcher.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>
#include <string>
#include <vector>
#include <utility>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

namespace {
  // FeatureTracks:
  //   [<global_feature_id>] --> [<FeatureTrack>]
  //
  // FeatureTrack:
  //
  //   [<feature_id> in frame 1, <x, y>]  |
  //   [<feature_id> in frame 2, <x, y>]  |
  //   [<feature_id> in frame 3, <x, y>]  |
  //                  .                   |
  //                  .                   |
  //                  .                   |
  //   [<feature_id> in frame N, <x, y>]  V
  //
  // where the <feature_id> in frame 1 is the <global_feature_id>.
  typedef vector<pair<size_t, Vector2d>> FeatureTrack;
  typedef map<size_t, FeatureTrack> FeatureTracks;

  const float HAMMING_DIST_THRESHOLD = 16;
}

bool NextImage(Mat& image_out, const char* path = nullptr) {
  static string prefix;
  static vector<pair<long, string>> image_file_list;
  static vector<pair<long, string>>::const_iterator it_cursor;

  if (path) {
    prefix = path;
    image_file_list.clear();

    /* List all image files ordered by name */
    if (!fs::exists(prefix)) {
      cout << "Error: Path " << prefix << " does not exist!" << endl;
      return false;
    }

    fs::directory_iterator end_it;
    for (fs::directory_iterator it(prefix); it != end_it; ++it) {
      vector<string> strs;
      bs::split(strs, it->path().filename().string(), bs::is_any_of("."));
      long file_id = atol(strs.front().c_str());
      image_file_list.push_back(make_pair(file_id, prefix + it->path().filename().string()));
    }

    /* Sort by name */
    sort(image_file_list.begin(),
         image_file_list.end(),
         [] (const pair<long, string>& a,
             const pair<long, string>& b) {
           return a.first < b.first;
         });

    it_cursor = image_file_list.cbegin();
  }

  if (it_cursor == image_file_list.cend()) {
    cout << "No more images!" << endl;
    return false;
  }

  /* Read a new image */
  const string& filename = it_cursor->second;
  Mat image = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
  if (image.empty()) {
    cout << "Error: Failed to load image: " << filename << "!" << endl;
    return false;
  }

  image_out = image;
  ++it_cursor;

  return true;
}

int main(int, char* argv[]) {
  
  string prefix = argv[1];
  string img_path = prefix + "data/";

  Mat image;
  bool has_img = NextImage(image, img_path.c_str());
  bool is_first_image = true;

  FeatureTracks tracks;
  FeatureMatcher orb_matcher(Eigen::Matrix3d::Identity(), HAMMING_DIST_THRESHOLD, FeatureUtils::kORBFeatureNum);

  // Feature ID lookup table
  // [<local_feature_id>] --> [<global_feature_id>]
  unordered_map<size_t, size_t> feature_id_TLB;

  while (has_img) {
    // Extract features
    Mat descriptors;
    vector<DMatch> matches;
    vector<Vector2d> features = FeatureUtils::ExtractFeaturesWithDescriptors(image, &descriptors, FeatureUtils::ORB);

    // Match with last frame
    FeatureMatcher::FeatureFrame frame;
    if (is_first_image) {
      orb_matcher.SetInitialDescriptors(descriptors);
      orb_matcher.MakeFirstFeatureFrame(features, &frame);
      is_first_image = false;
    } else {
      matches = orb_matcher.MatchORBWithOldFrame(features, descriptors, &frame);
    }

    // Update feature tracks
    for (auto it = frame.begin(); it != frame.end(); ++it) {
      size_t new_feature_id = it->first;
      size_t old_feature_id = it->second.first;
      const Vector2d& coord_2d = it->second.second;

      if (new_feature_id == old_feature_id) {  // This feature is seen the first time
        size_t global_feature_id = new_feature_id;

        // Update feature ID TLB
        auto new_entry = feature_id_TLB.find(new_feature_id);
        assert(new_entry == feature_id_TLB.end());
        feature_id_TLB[new_feature_id] = global_feature_id;

        tracks[global_feature_id].emplace_back(new_feature_id, coord_2d);
      } else {
        // Lookup for the corresponding global_feature_id
        auto old_entry = feature_id_TLB.find(old_feature_id);
        assert(old_entry != feature_id_TLB.end());
        size_t global_feature_id = old_entry->second;

        // Update feature ID TLB
        auto new_entry = feature_id_TLB.find(new_feature_id);
        assert(new_entry == feature_id_TLB.end());
        feature_id_TLB[new_feature_id] = global_feature_id;
        
        assert(new_feature_id != old_feature_id &&
               new_feature_id / FeatureUtils::kORBFeatureNum !=
               old_feature_id / FeatureUtils::kORBFeatureNum);
        tracks[global_feature_id].emplace_back(new_feature_id, coord_2d);
      }
    }

    // For test: Draw the features
    static Mat last_image;
    static vector<KeyPoint> keypoints, last_keypoints;
    static Mat image_out;
    if (!keypoints.empty())
      keypoints.clear();
    for (const auto& feature : features)
      keypoints.emplace_back(feature.x(), feature.y(), 1);

    if (!matches.empty()) {
      drawMatches(image, keypoints, last_image, last_keypoints, matches, image_out);
    } else {
      drawKeypoints(image, keypoints, image_out);
    }
    imshow("Features", image_out);
    last_image = image;
    last_keypoints = keypoints;

    int key = waitKey(0);
    while (key != 'n') {
      if (key == 'q')
        exit(0);
      key = waitKey(0);
    }
    has_img = NextImage(image);
  }

  // Debug information
  size_t tracks_num = 0;
  size_t max_track_len = 0;
  float average_track_len = 0;
  FeatureTracks::iterator longest_track;
  for (auto it = tracks.begin(); it != tracks.end(); ++it) {
    size_t global_feature_id = it->first;
    const FeatureTrack& track = it->second;
    if (track.size() > 2) {
      ++tracks_num;
      average_track_len += track.size();
      if (track.size() > max_track_len) {
        max_track_len = track.size();
        longest_track = it;
      }

      cout << global_feature_id << " (" << global_feature_id / FeatureUtils::kORBFeatureNum << ", "
                                        << global_feature_id % FeatureUtils::kORBFeatureNum << ")" << endl;
      for (const auto& feature : track) {
        size_t local_feature_id = feature.first;
        const Vector2d& coord_2d = feature.second;
        cout << "  [" << local_feature_id << "(" << local_feature_id / FeatureUtils::kORBFeatureNum << ")";
        cout << "\t<" << coord_2d.x() << ", " << coord_2d.y() << ">]" << endl;
      }
      cout << endl;
    }
  }
  cout << "\tTotal tracks: "<< tracks_num;
  cout << "\tAverage track length: " << average_track_len / tracks_num;
  cout << "\tMax track length: "<< max_track_len << endl;
  cout << "\t" << longest_track->first << " (" << longest_track->first / FeatureUtils::kORBFeatureNum << ", "
       << longest_track->first % FeatureUtils::kORBFeatureNum << ")" << endl;
  for (const auto& feature : longest_track->second) {
    size_t local_feature_id = feature.first;
    const Vector2d& coord_2d = feature.second;
    cout << "\t  [" << local_feature_id << "(" << local_feature_id / FeatureUtils::kORBFeatureNum << ")";
    cout << "\t<" << coord_2d.x() << ", " << coord_2d.y() << ">]" << endl;
  }
  cout << endl;

  return 0;
}
