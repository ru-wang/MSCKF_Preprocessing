#ifndef MSCKF_PREPROCESSING_VIRTUAL_FEATURE_TRACKER_H_
#define MSCKF_PREPROCESSING_VIRTUAL_FEATURE_TRACKER_H_

#include "FeatureMatcher.h"
#include "GenericFeatureTracker.h"
#include "Exception.h"

#include "MSCKF_Simulation/Helpers.h"

#include <Eigen/Eigen>

#include <opencv2/core.hpp>

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

class VirtualFeatureTracker : public GenericFeatureTracker {
 public:
  /*
   * Virtual pinhole camera with Gaussian noise.
   */
  struct Camera {
    Camera() : R_cam_in_imu((Eigen::Matrix3d() <<  0,  0,  1,
                                                  -1,  0,  0,
                                                   0, -1,  0).finished()),
               p_cam_in_imu(0, 0, 0),
               K((Eigen::Matrix3d() <<   1,   0, 0,
                                         0,   1, 0,
                                         0,   0, 1).finished()),
               K_inv(K.inverse()),
               FOV_u(-2, 2),
               FOV_v(-1, 1),
               DOF(0.5, 10),
               noise(0.05) {}

    Eigen::Matrix3d R_cam_in_imu;
    Eigen::Vector3d p_cam_in_imu;

    Eigen::Matrix3d K;      /* intrinsic matrix K */
    Eigen::Matrix3d K_inv;  /* inverse of K */
    Eigen::Vector2d FOV_u;  /* horizontal field of view in pixel, [FOV_u.x, FOV_u.y] */
    Eigen::Vector2d FOV_v;  /* vertical field of view in pixel,   [FOV_v.x, FOV_v.y] */
    Eigen::Vector2d DOF;    /* depth of field in meter */
    Gaussian noise;         /* Gaussian noise for images */
  };

  VirtualFeatureTracker(const std::string& imu_filename,
                        const std::string& gt_filename,
                        const Camera& camera,
                              size_t start = 0,
                              size_t landmark_num = 10000) throw(FileNotFound);

  virtual ~VirtualFeatureTracker() override {};

  virtual bool NextSensor(Vector9d* sensor_out, double* timestamp_out) override;
  bool NextImage(Pose* true_pose_out, double* timestamp_out);
  FeatureMatcher::FeatureFrame& ConstructFeatureFrame(const Pose& true_pose);

  void ShutUp() { shut_up_ = true; }

  bool shut_up() const { return shut_up_; }

  const Camera& virtual_camera() const { return virtual_camera_; }
        Camera& virtual_camera()       { return virtual_camera_; }

  const std::list<Pose>& true_pose_list() const { return true_pose_list_; };

  const std::vector<Eigen::Vector3d>& landmarks() const { return landmarks_; }

 private:
  /*
   * Suppress the output messages.
   */
  bool shut_up_;

  Camera virtual_camera_;

  void LoadIMUFromFile(size_t start);
  void LoadGroundTruthFromFile(size_t start);

  /*
   * Calculate the bounding box of the ground truth trajectory.
   * Must be called after `LoadGroundTruthFromFile()` is called.
   */
  void CalculateGTBoundingBox(Eigen::Vector3d* upper_left_forward,
                              Eigen::Vector3d* lower_right_backward);

  /*
   * Generate a bunch of landmarks uniformly distributed in the given bounding box.
   */
  void GenerateRandomLandmarks(const Eigen::Vector3d& upper_left_forward,
                               const Eigen::Vector3d& lower_right_backward,
                               const size_t number);

  std::string imu_filename_;
  std::list<Vector9d> imu_list_;
  std::list<Vector9d>::const_iterator imu_cursor_;

  std::list<double> imu_timestamp_list_;
  std::list<double>::const_iterator imu_timestamp_cursor_;

  /* 
   * Ground truth of IMU orientation and position in global coordinates.
   */
  std::string groundtruth_filename_;

  std::list<Pose> true_pose_list_;
  std::list<Pose>::const_iterator true_pose_cursor_;

  std::list<double> true_pose_timestamp_list_;
  std::list<double>::const_iterator true_pose_timestamp_cursor_;

  /*
   * Landmarks generated randomly.
   * [<landmark_id>] --> [<x, y, z> in global coordinates]
   * where <landmark_id> is also the global feature ID.
   */
  std::vector<Eigen::Vector3d> landmarks_;

  virtual bool NextImage(cv::Mat*, double*) override { return false; }
};

#endif  // MSCKF_PREPROCESSING_VIRTUAL_FEATURE_TRACKER_H_
