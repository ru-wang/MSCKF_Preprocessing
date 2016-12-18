#ifndef MSCKF_PREPROCESSING_FEATURES_H_
#define MSCKF_PREPROCESSING_FEATURES_H_

#include <eigen3/Eigen/Eigen>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <unordered_map>
#include <vector>

class FeatureUtils {
 public:
  enum Type { ORB, SIFT, SURF };

  static const int kORBFeatureNum        = 200;
  static const int kSIFTFeatureNum       = 0;
  static const int kSURFHessianThreshold = 1000;

  static std::vector<Eigen::Vector2d> ExtractFeatures(const cv::Mat& img, Type type) {
    using namespace cv;
    using namespace std;

    if (type != ORB && type != SIFT && type != SURF)
      return vector<Eigen::Vector2d>{};
    if (true == img.empty())
      return vector<Eigen::Vector2d>{};

    int feature_num = 0;
    vector<KeyPoint> keypoints;
    switch (type) {
      case ORB:  feature_num = extract_orb (img, &keypoints); break;
      case SIFT: feature_num = extract_sift(img, &keypoints); break;
      case SURF: feature_num = extract_surf(img, &keypoints); break;
      default: return vector<Eigen::Vector2d>{};
    }

    vector<Eigen::Vector2d> features;
    for (auto it = keypoints.cbegin(); it != keypoints.cend(); ++it)
      features.push_back(Eigen::Vector2d(it->pt.x, it->pt.y));
    return features;
  }

  static std::vector<Eigen::Vector2d> ExtractFeaturesWithDescriptors(const cv::Mat& img,
                                                                     cv::Mat* descriptors,
                                                                     Type type) {
    using namespace cv;
    using namespace std;

    if (type != ORB && type != SIFT && type != SURF)
      return vector<Eigen::Vector2d>{};
    if (true == img.empty())
      return vector<Eigen::Vector2d>{};

    int feature_num = 0;
    vector<KeyPoint> keypoints;
    switch (type) {
      case ORB:  feature_num = extract_orb (img, &keypoints, descriptors); break;
      case SIFT: feature_num = extract_sift(img, &keypoints, descriptors); break;
      case SURF: feature_num = extract_surf(img, &keypoints, descriptors); break;
      default: return vector<Eigen::Vector2d>{};
    }

    vector<Eigen::Vector2d> features;
    for (auto it = keypoints.cbegin(); it != keypoints.cend(); ++it)
      features.push_back(Eigen::Vector2d(it->pt.x, it->pt.y));
    return features;
  }

  // For unit test only
  static std::vector<cv::KeyPoint> ExtractKeypointsWithDescriptors(const char* filename,
                                                                   cv::Mat* descriptors,
                                                                   Type type) {
    using namespace cv;
    using namespace std;

    if (type != ORB && type != SIFT && type != SURF)
      return vector<KeyPoint>{};

    Mat img = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
    if (true == img.empty())
      return vector<KeyPoint>{};

    int feature_num = 0;
    vector<KeyPoint> keypoints;
    switch (type) {
      case ORB:  feature_num = extract_orb (img, &keypoints, descriptors); break;
      case SIFT: feature_num = extract_sift(img, &keypoints, descriptors); break;
      case SURF: feature_num = extract_surf(img, &keypoints, descriptors); break;
      default: return vector<KeyPoint>{};
    }

    return keypoints;
  }

 private:
  static int extract_orb(const cv::Mat& img_in,
                         std::vector<cv::KeyPoint>* keypoints_out,
                         cv::Mat* descriptors_out = nullptr);

  static int extract_sift(const cv::Mat& img_in,
                          std::vector<cv::KeyPoint>* keypoints_out,
                          cv::Mat* descriptors_out = nullptr);

  static int extract_surf(const cv::Mat& img_in,
                          std::vector<cv::KeyPoint>* keypoints_out,
                          cv::Mat* descriptors_out = nullptr);

  static cv::OrbFeatureDetector orb_detector_;
  static cv::SiftFeatureDetector sift_detector_;
  static cv::SurfFeatureDetector surf_detector_;

  static cv::OrbDescriptorExtractor orb_extractor_;
  static cv::SiftDescriptorExtractor sift_extractor_;
  static cv::SurfDescriptorExtractor surf_extractor_;
};

#endif  // MSCKF_PREPROCESSING_FEATURES_H_
