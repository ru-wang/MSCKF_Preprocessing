#include "Features.h"
#include "TrackKITTIFeatures.h"
#include "Utils.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <eigen3/Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <exception>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

namespace {

const float HAMMING_DIST_THRESHOLD = 16;
const float L2_DIST_THRESHOLD = 50;
const size_t MIN_TRACK_LENGTH = 5;

const bool CONVERT_TO_TANGO_TRACKS = true;
const bool PRINT_TANGO_TRACK_MATRIX = false;
const bool WRITE_TANGO_TRACK_MATRIX = true;

bool DRAW_MATCHES = false;
bool PRINT_TRACKS = false;

void PrintHelp() {
  cout << "\nNAME\n"
       << "    kitti_track - track 2D features from KITTI dataset\n\n"

       << "SYNOPSIS\n"
         << "    " << "[-h, --help]\n"
         << "    " << "[-v, --verbose]\n"
         << "    " << "[-d, --draw]\n"
         << "    " << "[-o, --output PATH]\n"
         << "    " << "<IMAGE PATH>\n\n"

       << "DESCRIPTION\n"
         << "    -h, --help\n\tprint this help information\n"
         << "    -v, --verbose\n\tprint track information\n"
         << "    -d, --draw\n\tdraw the keypoint matches\n"
         << "    -o, --output PATH\n\tthe output path\n"
         << "    <IMAGE PATH>\n\twhere the '/data' directory should locate\n" << endl;
}

}

KITTIFeatureTracker::KITTIFeatureTracker(const string& path, 
                                         const string& timestamp_filename) throw(runtime_error)
    : image_path_(path), timestamp_filename_(timestamp_filename),
      fmatcher_(Matrix3d::Identity(),
                HAMMING_DIST_THRESHOLD,
                FeatureUtils::kORBFeatureNum,
                FeatureUtils::kORBNormType,
                true) {
  /* List all image files ordered by name */
  if (!fs::exists(image_path_))
    throw runtime_error("Error: Path '" + image_path_ + "' does not exist!");
  if (!fs::exists(timestamp_filename_))
    throw runtime_error("Error: Timestamp file '" + timestamp_filename + "' does not exist!");

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

  ifstream timestamp_ifs(timestamp_filename_);
  string timestamp_str;
  double timestamp = 0;
  while (!timestamp_ifs.eof()) {
    utils::SafelyGetLine(timestamp_ifs, &timestamp_str);
    if (timestamp_str.empty())
      continue;
    timestamp = utils::TimestampSub(timestamp_str, "1970-01-01 00:00:00");
    timestamp_list_.push_back(timestamp);
  }
  timestamp_ifs.close();
  timestamp_cursor_ = timestamp_list_.cbegin();
}

bool KITTIFeatureTracker::NextImage(Mat* image_out, double* timestamp_out) {
  if (image_file_cursor_ == image_file_list_.cend() ||
      timestamp_cursor_ == timestamp_list_.cend()) {
    cout << "No more images!" << endl;
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
  double timestamp = *timestamp_cursor_;

  *image_out = image;
  *timestamp_out = timestamp;
  ++image_file_cursor_;
  ++timestamp_cursor_;

  cout << "[ " << (int)timestamp << "\t] Processing: " << filename << endl;
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

int main(int argc, char* argv[]) {
  if (argc == 1) {
    PrintHelp();
    return 0;
  }

  string opt, img_path, out_path;
  for (int i = 1; i < argc; ++i) {
    opt = argv[i];
    if (opt == "-h" or opt == "--help")
      PrintHelp();
    else if (opt == "-v" || opt == "--verbose")
      PRINT_TRACKS = true;
    else if (opt == "-d" || opt == "--draw")
      DRAW_MATCHES = true;
    else if (opt == "-o" || opt == "--output")
      out_path = argv[++i];
    else
      img_path = opt;
  }

  if (img_path.length() == 0) {
    PrintHelp();
    return 0;
  } else if (img_path.back() != '/') {
      img_path += "/";
  }

  if (out_path.length() == 0)
    out_path = img_path;
  else if (out_path.back() != '/')
    out_path += '/';

  KITTIFeatureTracker* tracker = nullptr;
  try {
    tracker = new KITTIFeatureTracker(img_path + "data/", img_path + "timestamps.txt");
  } catch (runtime_error e) {
    cerr << e.what() << endl;
    return 0;
  }

  Mat image;
  double img_timestamp;
  bool has_img = tracker->NextImage(&image, &img_timestamp);
  size_t max_feature_count = 0;

  while (has_img) {
    /* Extract features */
    Mat descriptors;
    vector<DMatch> matches;
    vector<Vector2d> features = FeatureUtils::ExtractFeaturesWithDescriptors(image, &descriptors, FeatureUtils::ORB);

    /* Match features extracted with last frame and return the matches */
    tracker->ConstructFeatureFrame(features, descriptors, &matches);

    if (DRAW_MATCHES) {
      /* For test: Draw the feature matches */
      static vector<Point> keypoints, last_keypoints;
      static Mat last_image, image_out;

      if (!keypoints.empty())
        keypoints.clear();
      for (const auto& feature : features)
        keypoints.emplace_back((int)feature.x(), (int)feature.y());

      if (!matches.empty()) {
        image_out.create(image.rows * 2, image.cols, CV_8UC3);
        Mat old_roi = image_out(Rect(0, 0, image.cols, image.rows));
        Mat new_roi = image_out(Rect(0, image.rows, image.cols, image.rows));
        cvtColor(last_image, old_roi, CV_GRAY2RGB);
        cvtColor(image, new_roi, CV_GRAY2RGB);

        for (const auto& keypoint : last_keypoints)
          circle(old_roi, keypoint, 5, Scalar(255, 0, 0));
        for (const auto& keypoint : keypoints)
          circle(new_roi, keypoint, 5, Scalar(0, 0, 255));

        for (const auto& match : matches) {
          const Point& old_pt = last_keypoints[match.trainIdx];
          const Point& new_pt = keypoints[match.queryIdx];

          /* Optical flows */
          line(new_roi, old_pt, new_pt, Scalar(0, 255, 0), 2);
          /* Matching lines */
          line(image_out, old_pt, new_pt + Point(0, image.rows), Scalar(0, 255, 0));
        }
      } else {
        image_out.create(image.rows, image.cols, CV_8UC3);
        cvtColor(image, image_out, CV_GRAY2RGB);
        for (const auto& keypoint : keypoints)
          circle(image_out, keypoint, 5, Scalar(0, 0, 255));
      }

      imshow("Features", image_out);
      last_keypoints = keypoints;
      last_image = image;

      int key = waitKey(100);
      if (key == 'q')
        exit(0);
    }

    has_img = tracker->NextImage(&image, &img_timestamp);
  }

  const KITTIFeatureTracker::FeatureTrackMap& tracks = tracker->tracks();

  if (CONVERT_TO_TANGO_TRACKS) {
    /* Convert feature tracks to Tango format */
    vector<vector<Vector3d>> tango_tracks(tracker->frame_list().size());
    size_t tango_tracks_id = 0;
    for (auto it = tracks.cbegin(); it != tracks.cend(); ++it) {
      const auto& track = it->second;
      if (track.size() >= MIN_TRACK_LENGTH) {
        for (const auto& feature : track) {
          size_t frame_id = feature.first / FeatureUtils::kORBFeatureNum;
          double u = feature.second.x();
          double v = feature.second.y();
          double w = tango_tracks_id + 1;
          tango_tracks[frame_id].emplace_back(u, v, w);
        }
        ++tango_tracks_id;
      }
    }

    const size_t rows = tracker->frame_list().size();
    const size_t cols = tango_tracks_id;
    MatrixXd tango_track_mat = MatrixXd::Constant(rows * 3, cols, -1);
    for (size_t row = 0; row < rows; ++row) {
      const auto& this_row = tango_tracks[row];
      for (auto it = this_row.cbegin(); it != this_row.cend(); ++it) {
        size_t col = it - this_row.cbegin();
        tango_track_mat(row * 3 + 0, col) = it->x();
        tango_track_mat(row * 3 + 1, col) = it->y();
        tango_track_mat(row * 3 + 2, col) = it->z();
      }
    }

    if (PRINT_TANGO_TRACK_MATRIX) {
      cout << "Tango track matrix: [" << rows << "x" << cols << "]" << endl;
      cout << tango_track_mat << endl;
    }

    if (WRITE_TANGO_TRACK_MATRIX) {
      ofstream ofs(out_path + "featuretracks.txt");
      ofs << tango_track_mat << endl;
      ofs.close();
    }
  }

  if (PRINT_TRACKS) {
    /* Debug information */
    size_t tracks_num = 0;
    size_t max_track_len = 0;
    float average_track_len = 0;
    KITTIFeatureTracker::FeatureTrackMap::const_iterator longest_track;
    for (auto it = tracks.cbegin(); it != tracks.cend(); ++it) {
      size_t global_feature_id = it->first;
      const KITTIFeatureTracker::FeatureTrack& track = it->second;
      if (track.size() >= MIN_TRACK_LENGTH) {
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
    cout << "\tMax feature count per frame: "<< max_feature_count;
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
  }

  delete tracker;
  return 0;
}
