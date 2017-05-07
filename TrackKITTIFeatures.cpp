#include "Features.h"
#include "KITTIFeatureTracker.h"

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <string>
#include <vector>
#include <exception>

using namespace cv;
using namespace std;
using namespace Eigen;

using namespace utils;

namespace {

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
    /* extract features */
    Mat descriptors;
    vector<DMatch> matches;
    vector<Vector2d> features = ExtractFeaturesWithDescriptors(image, &descriptors, utils::ORB);

    /* match features extracted with last frame and return the matches */
    tracker->ConstructFeatureFrame(features, descriptors, &matches);

    if (DRAW_MATCHES) {
      /* for test: draw the feature matches */
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

          /* optical flows */
          line(new_roi, old_pt, new_pt, Scalar(0, 255, 0), 2);
          /* matching lines */
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
    /* convert feature tracks to Tango format */
    vector<vector<Vector3d>> tango_tracks(tracker->frame_list().size());
    size_t tango_tracks_id = 0;
    for (auto it = tracks.cbegin(); it != tracks.cend(); ++it) {
      const auto& track = it->second;
      if (track.size() >= MIN_TRACK_LENGTH) {
        for (const auto& feature : track) {
          size_t frame_id = feature.first / kORBFeatureNum;
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
    /* debug information */
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

        cout << global_feature_id << " (" << global_feature_id / kORBFeatureNum << ", "
                                          << global_feature_id % kORBFeatureNum << ")" << endl;
        for (const auto& feature : track) {
          size_t local_feature_id = feature.first;
          const Vector2d& coord_2d = feature.second;
          cout << "  [" << local_feature_id << "(" << local_feature_id / kORBFeatureNum << ")";
          cout << "\t<" << coord_2d.x() << ", " << coord_2d.y() << ">]" << endl;
        }
        cout << endl;
      }
    }
    cout << "\tMax feature count per frame: "<< max_feature_count;
    cout << "\tTotal tracks: "<< tracks_num;
    cout << "\tAverage track length: " << average_track_len / tracks_num;
    cout << "\tMax track length: "<< max_track_len << endl;
    cout << "\t" << longest_track->first << " (" << longest_track->first / kORBFeatureNum << ", "
         << longest_track->first % kORBFeatureNum << ")" << endl;
    for (const auto& feature : longest_track->second) {
      size_t local_feature_id = feature.first;
      const Vector2d& coord_2d = feature.second;
      cout << "\t  [" << local_feature_id << "(" << local_feature_id / kORBFeatureNum << ")";
      cout << "\t<" << coord_2d.x() << ", " << coord_2d.y() << ">]" << endl;
    }
    cout << endl;
  }

  delete tracker;
  return 0;
}
