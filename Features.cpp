#include "Features.h"
using namespace cv;
using namespace std;

OrbFeatureDetector FeatureUtils::orb_detector_(kORBFeatureNum);
SiftFeatureDetector FeatureUtils::sift_detector_(kSIFTFeatureNum);
SurfFeatureDetector FeatureUtils::surf_detector_(kSURFHessianThreshold);

OrbDescriptorExtractor FeatureUtils::orb_extractor_{};
SiftDescriptorExtractor FeatureUtils::sift_extractor_{};
SurfDescriptorExtractor FeatureUtils::surf_extractor_{};

int FeatureUtils::extract_orb(const Mat& img_in,
                              vector<KeyPoint>* keypoints_out,
                              Mat* descriptors_out) {
  if (keypoints_out != nullptr) {
    orb_detector_.detect(img_in, *keypoints_out);
    if (descriptors_out != nullptr)
      orb_extractor_.compute(img_in, *keypoints_out, *descriptors_out);
    return keypoints_out->size();
  }
  return 0;
}

int FeatureUtils::extract_sift(const Mat& img_in,
                               vector<KeyPoint>* keypoints_out,
                               Mat* descriptors_out) {
  if (keypoints_out != nullptr) {
    sift_detector_.detect(img_in, *keypoints_out);
    if (descriptors_out != nullptr)
      sift_extractor_.compute(img_in, *keypoints_out, *descriptors_out);
    return keypoints_out->size();
  }
  return 0;
}

int FeatureUtils::extract_surf(const Mat& img_in,
                               vector<KeyPoint>* keypoints_out,
                               Mat* descriptors_out) {
  if (keypoints_out != nullptr) {
    surf_detector_.detect(img_in, *keypoints_out);
    if (descriptors_out != nullptr)
      surf_extractor_.compute(img_in, *keypoints_out, *descriptors_out);
    return keypoints_out->size();
  }
  return 0;
}
