#include "KITTIFeatureTracker.h"

#include "FeatureMatcher.h"
#include "Features.h"
#include "Utils.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <exception>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

namespace {

const float HAMMING_DIST_THRESHOLD = 16;
const float L2_DIST_THRESHOLD = 50;

}

KITTIFeatureTracker::KITTIFeatureTracker(const string& path, 
                                         const string& timestamp_filename,
                                         const string& imu_filename) throw(runtime_error)
    : image_path_(path), image_timestamp_filename_(timestamp_filename), imu_filename_(imu_filename),
      fmatcher_(Matrix3d::Identity(),
                HAMMING_DIST_THRESHOLD,
                FeatureUtils::kORBFeatureNum,
                FeatureUtils::kORBNormType,
                true) {
  /* List all image files ordered by name */
  if (!fs::exists(image_path_))
    throw runtime_error("Error: Path '" + image_path_ + "' does not exist!");
  if (!fs::exists(image_timestamp_filename_))
    throw runtime_error("Error: Timestamp file '" + timestamp_filename + "' does not exist!");
  if (!imu_filename_.empty() && !fs::exists(imu_filename_))
    throw runtime_error("Error: IMU file '" + imu_filename_ + "' does not exist!");

  fs::directory_iterator end_it;
  for (fs::directory_iterator it(image_path_); it != end_it; ++it) {
    vector<string> strs;
    bs::split(strs, it->path().filename().string(), bs::is_any_of("."));
    long file_id = stol(strs.front().c_str());
    const string& extension = strs.back();
    if (extension == "png" || extension == "jpg")
      image_file_list_.push_back(make_pair(file_id, image_path_ + it->path().filename().string()));
  }

  /* Sort by name */
  sort(image_file_list_.begin(),
       image_file_list_.end(),
       [] (const pair<long, string>& a,
           const pair<long, string>& b) {
         return a.first < b.first;
       });
  image_file_cursor_ = image_file_list_.cbegin();

  ifstream timestamp_ifs(image_timestamp_filename_);
  string timestamp_str;
  double timestamp = 0;
  while (!timestamp_ifs.eof()) {
    utils::SafelyGetLine(timestamp_ifs, &timestamp_str);
    if (timestamp_str.empty())
      continue;
    timestamp = utils::TimestampSub(timestamp_str, "1970-01-01 00:00:00");
    image_timestamp_list_.push_back(timestamp);
  }
  timestamp_ifs.close();
  image_timestamp_cursor_ = image_timestamp_list_.cbegin();
  cout << "[ Read\t] Total images: " << image_file_list_.size() << endl;

  if (!imu_filename.empty()) {
    ifstream imu_ifs(imu_filename_);
    string imu_line;
    Vector9d imu;
    double gx, gy, gz, ax, ay, az, vx, vy, vz;
    while (!imu_ifs.eof()) {
      utils::SafelyGetLine(imu_ifs, &imu_line);
      if (imu_line.empty())
        continue;
      vector<string> imu_strs;
      bs::split(imu_strs, imu_line, bs::is_any_of(" "), bs::token_compress_on);
      gx = stod(imu_strs[1]); gy = stod(imu_strs[2]); gz = stod(imu_strs[3]);
      ax = stod(imu_strs[4]); ay = stod(imu_strs[5]); az = stod(imu_strs[6]);
      vx = stod(imu_strs[7]); vy = stod(imu_strs[8]); vz = stod(imu_strs[9]);
      imu << gx, gy, gz, ax, ay, az, vx, vy, vz;
      timestamp = stod(imu_strs.front());
      imu_list_.push_back(imu);
      imu_timestamp_list_.push_back(timestamp);
    }
    imu_ifs.close();
    imu_cursor_ = imu_list_.cbegin();
    imu_timestamp_cursor_ = imu_timestamp_list_.cbegin();
    cout << "[ Read\t] Total IMUs: " << imu_list_.size() << endl;
  }
}

bool KITTIFeatureTracker::NextImage(Mat* image_out, double* timestamp_out) {
  if (image_file_cursor_ == image_file_list_.cend() ||
      image_timestamp_cursor_ == image_timestamp_list_.cend()) {
    return false;
  }

  /* Read a new image */
  const string& filename = image_file_cursor_->second;
  Mat image = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
  if (image.empty()) {
    cerr << "Error: Failed to load image: " << filename << "!" << endl;
    return false;
  }

  /* Read a new timestamp */
  double timestamp = *image_timestamp_cursor_;

  *image_out = image;
  *timestamp_out = timestamp;
  ++image_file_cursor_;
  ++image_timestamp_cursor_;

  cout << "[ " << (int)timestamp << "\t] Processing: " << filename << endl;
  return true;
}

bool KITTIFeatureTracker::NextSensor(Vector9d* sensor_out, double* timestamp_out) {
  if (imu_cursor_ == imu_list_.cend() ||
      imu_timestamp_cursor_ == imu_timestamp_list_.cend()) {
    return false;
  }

  /* Read a new IMU */
  Vector9d imu = *imu_cursor_;

  /* Read a new timestamp */
  double timestamp = *imu_timestamp_cursor_;

  *timestamp_out = timestamp;
  *sensor_out = imu;
  ++imu_cursor_;
  ++imu_timestamp_cursor_;

  cout << "[ " << (int)timestamp << "\t] IMU reading: " << imu.transpose() << endl;
  return true;
}

FeatureMatcher::FeatureFrame&
KITTIFeatureTracker::ConstructFeatureFrame(const vector<Vector2d>& features, const Mat& descriptors,
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
