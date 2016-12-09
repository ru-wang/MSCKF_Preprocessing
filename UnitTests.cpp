#include "Features.h"

#include <cstdio>
#include <iostream>

using namespace cv;
using namespace std;

namespace {
  constexpr size_t MAX_FILENAME_LEN = 100;
}

int main(int, char* argv[]) {

  const char* file_format = argv[1];
  char filename[MAX_FILENAME_LEN];
  int img_index = 0;
  int key = 0;

  do {
    memset(filename, 0, MAX_FILENAME_LEN);
    sprintf(filename, file_format, img_index++);

    //vector<Eigen::Vector2d> features = FeatureUtils::ExtractFeatures(filename, FeatureUtils::ORB);
    Mat descriptors;
    vector<KeyPoint> features = FeatureUtils::ExtractKeypointsWithDescriptors(filename, &descriptors, FeatureUtils::ORB);
    if (0 == features.size()) {
      printf("Failed to load image %s\n", filename);
    } else {
      Mat img_in = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
      Mat img_out;
      drawKeypoints(img_in, features, img_out);

      cout << descriptors << "\n" << endl;
      imshow("ORB FeatureUtils", img_out);
    }

    key = waitKey(0);
  } while (key != 'q' && key != 'Q' && key != 27);

  return 0;
}
