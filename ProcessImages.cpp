#include "Features.h"
#include "FeatureMatcher.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

using namespace cv;
using namespace std;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

namespace {
  const float HAMMING_DIST_THRESHOLD = 16;
}

int main(int, char* argv[]) {

  const char* path = argv[1];

  /* list all image_file_list by ordered by name(timestamp) */
  if (!fs::exists(path)) {
    cout << "Error: Path " << path << " does not exist!" << endl;
    return -1;
  }

  vector<pair<long, string>> image_file_list;
  fs::directory_iterator end_it;
  for (fs::directory_iterator it(path); it != end_it; ++it) {
    vector<string> strs;
    bs::split(strs, it->path().filename().string(), bs::is_any_of("."));
    long timestamp = atol(strs.front().c_str());
    image_file_list.push_back(make_pair(timestamp, it->path().filename().string()));
  }

  /* sort by name(timestamp) */
  sort(image_file_list.begin(),
       image_file_list.end(),
       [] (const pair<long, string>& a,
           const pair<long, string>& b) {
         return a.first < b.first;
       });

  /* extract features & descriptors */
  Mat last_img;
  Mat last_descriptors;
  vector<KeyPoint> last_keypoints;
  FeatureMatcher orb_matcher(Eigen::Matrix3d::Identity(), HAMMING_DIST_THRESHOLD, FeatureUtils::kORBFeatureNum);
  for (auto& image_file : image_file_list) {
    string filename = path + image_file.second;

    Mat descriptors;
    vector<KeyPoint> features = FeatureUtils::ExtractKeypointsWithDescriptors(filename.c_str(), &descriptors, FeatureUtils::ORB);
    if (0 == features.size()) {
      cout << "Failed to load image_file " << filename << "!" << endl;
      return -1;
    } else {
      Mat img_in = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
      Mat img_out;
      if (!last_img.empty()) {
        // Match descriptors
        FeatureMatcher::FeatureFrame frame;
        vector<vector<DMatch>> matches = orb_matcher.MatchORBWithOldFrame(features, descriptors, &frame);

        // Draw the results
        drawMatches(img_in, features, last_img, last_keypoints, matches, img_out);
      } else {
        orb_matcher.SetInitialDescriptors(descriptors);
        drawKeypoints(img_in, features, img_out);
      }
      imshow("Features", img_out);
      last_img = img_in;
      last_descriptors = descriptors;
      last_keypoints = features;
    }

    while (true) {
      int key = waitKey(1);
      if (key == 'q' || key == 'Q')
        return 0;
      else if (key == 'n' || key == 'N')
        break;
    }
  }

  destroyAllWindows();
  return 0;
}
