#include "Features.h"

#include <opencv2/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

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
    if (keypoints_out->size() > kSIFTFeatureNum)
      keypoints_out->resize(kSIFTFeatureNum);
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

std::vector<Eigen::Vector2d>
(*utils::ExtractFeatures)(const cv::Mat& img, FeatureUtils::Type type)
    = FeatureUtils::ExtractFeatures;

std::vector<Eigen::Vector2d>
(*utils::ExtractFeaturesWithDescriptors)
(const cv::Mat& img, cv::Mat* descriptors, FeatureUtils::Type type)
    = FeatureUtils::ExtractFeaturesWithDescriptors;

std::vector<cv::KeyPoint>
(*utils::ExtractKeypointsWithDescriptors)
(const char* filename, cv::Mat* descriptors, FeatureUtils::Type type)
    = FeatureUtils::ExtractKeypointsWithDescriptors;
