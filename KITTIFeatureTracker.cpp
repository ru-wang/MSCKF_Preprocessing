#include "KITTIFeatureTracker.h"

#include "GenericFeatureTracker.h"
#include "Utils.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

KITTIFeatureTracker::KITTIFeatureTracker(const string& path,
                                         const string& timestamp_filename,
                                         const Eigen::Matrix3d& K,
                                         const string& imu_filename) throw(FileNotFound)
    : GenericFeatureTracker(K),
      image_path_(path),
      image_timestamp_filename_(timestamp_filename),
      imu_filename_(imu_filename) {
  /* list all image files ordered by name */
  if (!fs::exists(image_path_))
    throw FileNotFound(image_path_, "Path");
  if (!fs::exists(image_timestamp_filename_))
    throw FileNotFound(timestamp_filename, "Timestamp file");
  if (!imu_filename_.empty() && !fs::exists(imu_filename_))
    throw FileNotFound(imu_filename_, "IMU file");

  fs::directory_iterator end_it;
  for (fs::directory_iterator it(image_path_); it != end_it; ++it) {
    vector<string> strs;
    bs::split(strs, it->path().filename().string(), bs::is_any_of("."));
    long file_id = stol(strs.front().c_str());
    const string& extension = strs.back();
    if (extension == "png" || extension == "jpg")
      image_file_list_.push_back(make_pair(file_id, image_path_ + it->path().filename().string()));
  }

  /* sort by name */
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

  /* read a new image */
  const string& filename = image_file_cursor_->second;
  Mat image = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
  if (image.empty()) {
    cerr << "Error: Failed to load image: " << filename << "!" << endl;
    return false;
  }

  /* read a new timestamp */
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

  /* read a new IMU */
  Vector9d imu = *imu_cursor_;

  /* read a new timestamp */
  double timestamp = *imu_timestamp_cursor_;

  *timestamp_out = timestamp;
  *sensor_out = imu;
  ++imu_cursor_;
  ++imu_timestamp_cursor_;

  cout << "[ " << (int)timestamp << "\t] IMU reading: " << imu.transpose() << endl;
  return true;
}
