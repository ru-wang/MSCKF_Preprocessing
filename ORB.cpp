#include <cstdio>
#include <cstring>
#include <string>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace cv;

namespace {
  constexpr size_t MAX_FILENAME_LEN = 100;
}

int main(int, char** argv) {

  const char* file_format = argv[1];
  char filename1[MAX_FILENAME_LEN];
  char filename2[MAX_FILENAME_LEN];
  int img_index = 0;
  int key = 0;

  do {
    memset(filename1, 0, MAX_FILENAME_LEN);
    memset(filename2, 0, MAX_FILENAME_LEN);
    sprintf(filename1, file_format, img_index++);
    sprintf(filename2, file_format, img_index);

    Mat img1 = imread(filename1, CV_LOAD_IMAGE_UNCHANGED);
    Mat img2 = imread(filename2, CV_LOAD_IMAGE_UNCHANGED);
    resize(img1, img1, Size(0, 0), 0.5, 0.5);
    resize(img2, img2, Size(0, 0), 0.5, 0.5);

    if(img1.empty() || img2.empty()) {
      printf("Can't read one of the images\n");
      return -1;
    }

    // detecting keypoints
    OrbFeatureDetector detector(1000);
    vector<KeyPoint> keypoints1, keypoints2;
    detector.detect(img1, keypoints1);
    detector.detect(img2, keypoints2);

    // computing descriptors
    OrbDescriptorExtractor extractor;
    Mat descriptors1, descriptors2;
    extractor.compute(img1, keypoints1, descriptors1);
    extractor.compute(img2, keypoints2, descriptors2);

    // matching descriptors
    BruteForceMatcher<L2<float>> matcher;
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // drawing the results
    Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    imshow("ORB Matches", img_matches);

    key = waitKey(0);
  } while (key != 'q' && key != 'Q' && key != 27);

  return 0;
}
