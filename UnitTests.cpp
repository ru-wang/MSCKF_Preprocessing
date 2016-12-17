#include "Features.h"
#include "FeatureMatcher.h"

#include "MSCKF/MSCKF.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

namespace {
  typedef Matrix<double, 6, 1> Vector6d;

  const Quaterniond INIT_ORIENTATION((Matrix3d() << Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()).finished());

  const float HAMMING_DIST_THRESHOLD = 16;

  const double COV_NG = 0.002;
  const double COV_NWG = 0.0005;
  const double COV_NA = 0.05;
  const double COV_NWA = 0.005;
  const double SIGMA_NIM = 0.005;
}

bool NextSensor(Vector3d& gyro_out, Vector3d& acce_out, long& timestamp_out, const char* filename = nullptr) {
  static ifstream ifs;
  static vector<pair<Vector6d, long>> imu_data_list;
  static vector<pair<Vector6d, long>>::const_iterator it_cursor;

  /* Load all imu data at once */
  if (filename) {
    ifs.open(filename);
    if (!ifs) {
      cout << "Error: Failed to open " << filename << "!" << endl;
      return false;
    }
  }

  string a_line;
  while (getline(ifs, a_line)) {
    cout << "a_line" << endl;
    stringstream ss(a_line);
    Vector6d imu;
    double gx, gy, gz, ax, ay, az;
    long timestamp;
    ss >> gx >> gy >> gz >> ax >> ay >> az;
    ss >> timestamp;

    /*
     * Should invert the coordinate system first. See the API Guides:
     * https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-accel
     */
    imu << -gx, -gy, -gz, -ax, -ay, -az;
    imu_data_list.push_back(make_pair(imu, timestamp));
    it_cursor = imu_data_list.cbegin();
  }

  if (it_cursor == imu_data_list.cend()) {
    cout << "No more imu data!" << endl;
    return false;
  }

  /* Read a tuple */
  timestamp_out = it_cursor->second;
  const Vector6d& imu = it_cursor->first;
  gyro_out[0] = imu[0 + 0]; gyro_out[1] = imu[0 + 1]; gyro_out[2] = imu[0 + 2];
  acce_out[0] = imu[3 + 0]; acce_out[1] = imu[3 + 1]; acce_out[2] = imu[3 + 2];
  ++it_cursor;

  return true;
}

bool NextImage(Mat& image_out, long& timestamp_out, const char* path = nullptr) {
  static string prefix;
  static vector<pair<long, string>> image_file_list;
  static vector<pair<long, string>>::const_iterator it_cursor;

  if (path) {
    prefix = path;
    image_file_list.clear();

    /* List all image files ordered by name(timestamp) */
    if (!fs::exists(prefix)) {
      cout << "Error: Path " << prefix << " does not exist!" << endl;
      return false;
    }

    fs::directory_iterator end_it;
    for (fs::directory_iterator it(prefix); it != end_it; ++it) {
      vector<string> strs;
      bs::split(strs, it->path().filename().string(), bs::is_any_of("."));
      long timestamp = atol(strs.front().c_str());
      image_file_list.push_back(make_pair(timestamp, it->path().filename().string()));
    }

    /* Sort by name(timestamp) */
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
  timestamp_out = it_cursor->first;
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
  string img_path = prefix + "IMG/";
  string imu_file = prefix + "imu.txt";

  Vector3d gyro, acce;
  Mat image;
  long imu_timestamp, img_timestamp;

  bool has_imu = NextSensor(gyro, acce, imu_timestamp, imu_file.c_str());
  bool has_img = NextImage(image, img_timestamp, img_path.c_str());

  /* Initialize the MSCKFs */
  MSCKF ekf, ekf_propagate_only;
  ekf.setNoiseCov(Matrix3d::Identity() * COV_NG,
                  Matrix3d::Identity() * COV_NWG,
                  Matrix3d::Identity() * COV_NA,
                  Matrix3d::Identity() * COV_NWA, SIGMA_NIM);
  ekf.initialize(HamiltonToJPL(INIT_ORIENTATION),
                 Vector3d::Zero(),
                 Vector3d::Zero(),
                 Vector3d::Zero(),
                 Vector3d::Zero());
  ekf_propagate_only.setNoiseCov(Matrix3d::Identity() * COV_NG,
                                 Matrix3d::Identity() * COV_NWG,
                                 Matrix3d::Identity() * COV_NA,
                                 Matrix3d::Identity() * COV_NWA, SIGMA_NIM);
  ekf_propagate_only.initialize(HamiltonToJPL(INIT_ORIENTATION),
                                Vector3d::Zero(),
                                Vector3d::Zero(),
                                Vector3d::Zero(),
                                Vector3d::Zero());

  FeatureMatcher orb_matcher(Eigen::Matrix3d::Identity(), HAMMING_DIST_THRESHOLD, FeatureUtils::kORBFeatureNum);
  
  bool is_first_image = true;
  while (has_imu || has_img) {
    double t_second = 0;
    if (!has_imu) {         // has_imu == false && has_img == true
      t_second = img_timestamp / 1.0e9;

      // Extract features
      Mat descriptors;
      vector<Vector2d> features = FeatureUtils::ExtractFeaturesWithDescriptors(image, &descriptors, FeatureUtils::ORB);
      FeatureMatcher::FeatureFrame frame;
      if (is_first_image) {
        orb_matcher.SetInitialDescriptors(descriptors);
        orb_matcher.MakeFirstFeatureFrame(features, &frame);
        is_first_image = false;
      } else {
        orb_matcher.MatchORBWithOldFrame(features, descriptors, &frame);
      }
      ekf.update(t_second, frame);

      has_img = NextImage(image, img_timestamp);
    } else if (!has_img) {  // has_imu == true  && has_img == false
      t_second = imu_timestamp / 1.0e9;
      ekf.propagate(t_second, gyro, acce);
      ekf_propagate_only.propagate(t_second, gyro, acce);

      has_imu = NextSensor(gyro, acce, imu_timestamp);
    } else {                // has_imu == true  && has_img == true
      /* Compare the timestamp */
      if (imu_timestamp < img_timestamp) {
        // Pass in IMU data first
        t_second = imu_timestamp / 1.0e9;
        ekf.propagate(t_second, gyro, acce);
        ekf_propagate_only.propagate(t_second, gyro, acce);

        has_imu = NextSensor(gyro, acce, imu_timestamp);
      } else if (img_timestamp < imu_timestamp) {
        // Pass in Image data first
        t_second = img_timestamp / 1.0e9;

        // Extract features
        Mat descriptors;
        vector<Vector2d> features = FeatureUtils::ExtractFeaturesWithDescriptors(image, &descriptors, FeatureUtils::ORB);
        FeatureMatcher::FeatureFrame frame;
        if (is_first_image) {
          orb_matcher.SetInitialDescriptors(descriptors);
          orb_matcher.MakeFirstFeatureFrame(features, &frame);
          is_first_image = false;
        } else {
          orb_matcher.MatchORBWithOldFrame(features, descriptors, &frame);
        }
        ekf.update(t_second, frame);

        has_img = NextImage(image, img_timestamp);
      } else /* (img_timestamp == imu_timestamp) */ {
        t_second = imu_timestamp / 1.0e9;
        ekf.propagate(t_second, gyro, acce);
        ekf_propagate_only.propagate(t_second, gyro, acce);

        // Extract features
        Mat descriptors;
        vector<Vector2d> features = FeatureUtils::ExtractFeaturesWithDescriptors(image, &descriptors, FeatureUtils::ORB);
        FeatureMatcher::FeatureFrame frame;
        if (is_first_image) {
          orb_matcher.SetInitialDescriptors(descriptors);
          orb_matcher.MakeFirstFeatureFrame(features, &frame);
          is_first_image = false;
        } else {
          orb_matcher.MatchORBWithOldFrame(features, descriptors, &frame);
        }
        ekf.update(t_second, frame);

        has_imu = NextSensor(gyro, acce, imu_timestamp);
        has_img = NextImage(image, img_timestamp);
      }
    }

    stringstream ss;
    ss << t_second << ", " << ekf_propagate_only.position().norm() << ", "
                           << ekf.position().norm() << ", "
                           << ekf.positionCovariance().diagonal().sum();
    cout << ss.str() << endl;
  }

  return 0;
}
