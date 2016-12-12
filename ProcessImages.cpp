#include "Features.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

using namespace cv;
using namespace std;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

bool comp(const pair<long, string>& a, const pair<long, string>& b) { return a.first < b.first; }

int main(int, char* argv[]) {

  const char* path = argv[1];

  /* List all images by ordered by name(timestamp) */
  if (!fs::exists(path)) {
    cout << "Error: Path " << path << " does not exist!" << endl;
    return -1;
  }

  vector<pair<long, string>> images;
  fs::directory_iterator end_it;
  for (fs::directory_iterator it(path); it != end_it; ++it) {
    vector<string> strs;
    bs::split(strs, it->path().filename().string(), bs::is_any_of("."));
    long timestamp = atol(strs.front().c_str());
    images.push_back(make_pair(timestamp, it->path().filename().string()));
  }

  /* Sort by name(timestamp) */
  sort(images.begin(), images.end(), comp);

  /* Extract features & descriptors */
  for (auto& image : images) {
    string filename = path + image.second;

    Mat descriptors;
    vector<KeyPoint> features = FeatureUtils::ExtractKeypointsWithDescriptors(filename.c_str(), &descriptors, FeatureUtils::ORB);
    if (0 == features.size()) {
      cout << "Failed to load image " << filename << "!" << endl;
      return -1;
    } else {
      Mat img_in = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
      Mat img_out;
      drawKeypoints(img_in, features, img_out);
      imshow("Features", img_out);
    }

    int key = waitKey(1);
    if (key == 'q' || key == 'Q')
      break;
  }

  destroyAllWindows();
  return 0;
}
