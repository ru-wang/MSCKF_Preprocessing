#include "Features.h"
#include "KITTIFeatureTracker.h"
#include "SLAMTrajectoryDrawer.h"

#include "MSCKF/JPL.h"
#include "MSCKF/MSCKF.h"

#include <Eigen/Eigen>
#include <glm/glm.hpp>
#include <opencv2/highgui.hpp>

#include <cassert>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
using namespace Eigen;

using namespace utils;

namespace {

class NoImuError : public runtime_error {
 public:
  NoImuError() : runtime_error("Runtime: No more IMU data!") {}
};

class NoImageError : public runtime_error {
 public:
  NoImageError() : runtime_error("Runtime: No more images!") {}
};

const Quaterniond INIT_ORIENTATION((Matrix3d() << Vector3d::UnitX(),
                                                  Vector3d::UnitY(),
                                                  Vector3d::UnitZ()).finished());

const double COV_NG = 0.002;
const double COV_NWG = 0.0005;
const double COV_NA = 0.05;
const double COV_NWA = 0.005;
const double SIGMA_NIM = 0.005;

void LogTrajectory(const MSCKF& ekf, vector<glm::vec3>* loc, vector<glm::vec4>* ori) {
  loc->push_back(glm::vec3(ekf.position().x(),
                           ekf.position().y(),
                           ekf.position().z()));
  JPL_Quaternion q_I_to_G = HamiltonToJPL(ekf.orientation());
  ori->push_back(glm::vec4(q_I_to_G.x(),
                           q_I_to_G.y(),
                           q_I_to_G.z(),
                           q_I_to_G.w()));
}

}

int main(int argc, char* argv[]) {
  assert(argc > 2);
  string img_path = string(argv[1]) + "data/";
  string img_timestamp_filename = string(argv[1]) + "timestamps.txt";
  string imu_file = string(argv[2]);

  /****************************************************************************
   * I. Create a feature tracker.
   ****************************************************************************/
  Mat image;
  double img_timestamp = 0;
  bool has_img = false;
  KITTIFeatureTracker* tracker = nullptr;
  try {
    tracker = new KITTIFeatureTracker(img_path, img_timestamp_filename, imu_file);
  } catch (runtime_error e) {
    cerr << e.what() << endl;
    return 0;
  }

  /****************************************************************************
   * II. Read the first IMU in order to get the initial velocity.
   ****************************************************************************/
  KITTIFeatureTracker::Vector9d imu;
  double imu_timestamp = 0;
  bool has_imu = false;
  has_imu = tracker->NextSensor(&imu, &imu_timestamp);

  /****************************************************************************
   * III. Initialize two MSCKFs: one is for propagation only.
   ****************************************************************************/
  MSCKF ekf_ppg_only, ekf;
  ekf_ppg_only.setNoiseCov(Matrix3d::Identity() * COV_NG,
                           Matrix3d::Identity() * COV_NWG,
                           Matrix3d::Identity() * COV_NA,
                           Matrix3d::Identity() * COV_NWA,
                           SIGMA_NIM);
  ekf_ppg_only.initialize(HamiltonToJPL(INIT_ORIENTATION),
                          Vector3d::Zero(),
                          Vector3d{imu[6], imu[7], imu[8]},
                          Vector3d::Zero(),
                          Vector3d::Zero());
  ekf.setNoiseCov(Matrix3d::Identity() * COV_NG,
                  Matrix3d::Identity() * COV_NWG,
                  Matrix3d::Identity() * COV_NA,
                  Matrix3d::Identity() * COV_NWA,
                  SIGMA_NIM);
  ekf.initialize(HamiltonToJPL(INIT_ORIENTATION),
                 Vector3d::Zero(),
                 Vector3d{imu[6], imu[7], imu[8]},
                 Vector3d::Zero(),
                 Vector3d::Zero());

  /****************************************************************************
   * IV. Prepare for the main loop.
   ****************************************************************************/
  vector<glm::vec3> loc_ppg, loc_upd;
  vector<glm::vec4> ori_ppg, ori_upd;
  has_img = tracker->NextImage(&image, &img_timestamp);
  has_imu = tracker->NextSensor(&imu, &imu_timestamp);
  double t_second = 0;


  /****************************************************************************
   * V. Main loop.
   *    - On reading a new IMU, integrate it to both EFKs;
   *    - On reading a new image, extract the features and let the EFK
   *      decide whether to perform a update;
   *    - If there is no IMU or image any more, throw an exception.
   ****************************************************************************/
  try {

    while (has_imu || has_img) {
      /*
       * 1. We don't have IMU any more, but still have some images.
       *    Update for the last time then quit.
       *    (has_imu == false && has_img == true)
       */
      if (!has_imu) {
        t_second = img_timestamp;

        /* Extract features */
        Mat descriptors;
        vector<DMatch> matches;
        vector<Vector2d> features;
        features = ExtractFeaturesWithDescriptors(image, &descriptors, utils::ORB);

        /* Match features with the last frame and return the FeatureFrame */
        FeatureMatcher::FeatureFrame frame;
        frame = tracker->ConstructFeatureFrame(features, descriptors, &matches);

        /* Perform an update */
        ekf.update(t_second, frame);

        throw NoImuError();
      }

      /*
       * 2. We don't have images any more, but still have some IMUs to integrate.
       *    Integrate them then quit.
       *    (has_imu == true && has_img == false)
       */
      else if (!has_img) {
        t_second = imu_timestamp;

        /* Integrate to both EKFs */
        Vector3d gyro {imu[0], imu[1], imu[2]};
        Vector3d acce {imu[3], imu[4], imu[5]};
        ekf.propagate(t_second, gyro, acce);
        ekf_ppg_only.propagate(t_second, gyro, acce);

        throw NoImageError();
      }

      /*
       * 3. Now we have both new images and new IMU(which is the normal case).
       *    Compare their timestamps and decide whether to propagate or update.
       *    (has_imu == true && has_img == true)
       */
      else {
        bool has_propagated = false;
        bool has_updated = false;

        /*
         * (A) If the IMU comes first, integrate.
         */
        if (imu_timestamp < img_timestamp) {
          t_second = imu_timestamp;

          Vector3d gyro {imu[0], imu[1], imu[2]};
          Vector3d acce {imu[3], imu[4], imu[5]};
          ekf.propagate(t_second, gyro, acce);
          ekf_ppg_only.propagate(t_second, gyro, acce);

          has_propagated = true;
          has_imu = tracker->NextSensor(&imu, &imu_timestamp);
        }

        /*
         * (B) Here comes a new image, let the EFK decide wether to update.
         */
        else if (img_timestamp < imu_timestamp) {
          t_second = img_timestamp;

          /* Extract features */
          Mat descriptors;
          vector<DMatch> matches;
          vector<Vector2d> features;
          features = ExtractFeaturesWithDescriptors(image, &descriptors, utils::ORB);

          /* Match features with the last frame and return the FeatureFrame */
          FeatureMatcher::FeatureFrame frame;
          frame = tracker->ConstructFeatureFrame(features, descriptors, &matches);

          /* Perform an update */
          ekf.update(t_second, frame);

          has_updated = true;
          has_img = tracker->NextImage(&image, &img_timestamp);
        }

        /*
         * (C) If the IMU and the image come at the same time, integrate then update.
         */
        else /* (img_timestamp == imu_timestamp) */ {
          t_second = imu_timestamp;

          Vector3d gyro {imu[0], imu[1], imu[2]};
          Vector3d acce {imu[3], imu[4], imu[5]};
          ekf.propagate(t_second, gyro, acce);
          ekf_ppg_only.propagate(t_second, gyro, acce);

          /* Extract features */
          Mat descriptors;
          vector<DMatch> matches;
          vector<Vector2d> features;
          features = ExtractFeaturesWithDescriptors(image, &descriptors, utils::ORB);

          /* Match features with the last frame and return the FeatureFrame */
          FeatureMatcher::FeatureFrame frame;
          frame = tracker->ConstructFeatureFrame(features, descriptors, &matches);

          /* Perform an update */
          ekf.update(t_second, frame);

          has_propagated = has_updated = true;
          has_imu = tracker->NextSensor(&imu, &imu_timestamp);
          has_img = tracker->NextImage(&image, &img_timestamp);
        }

        if (has_propagated) LogTrajectory(ekf_ppg_only, &loc_ppg, &ori_ppg);
        if (has_updated)    LogTrajectory(ekf,          &loc_upd, &ori_upd);
      }
    }

  } catch (NoImuError e) {
    cout << e.what() << endl;
    LogTrajectory(ekf, &loc_upd, &ori_upd);
  } catch (NoImageError e) {
    cout << e.what() << endl;
    LogTrajectory(ekf_ppg_only, &loc_ppg, &ori_ppg);

    /* Perform propagations utill there is no more IMU */
    while ((has_imu = tracker->NextSensor(&imu, &imu_timestamp))) {
      t_second = imu_timestamp;
      Vector3d gyro {imu[0], imu[1], imu[2]};
      Vector3d acce {imu[3], imu[4], imu[5]};
      ekf.propagate(t_second, gyro, acce);
      ekf_ppg_only.propagate(t_second, gyro, acce);
      LogTrajectory(ekf_ppg_only, &loc_ppg, &ori_ppg);
    }
  }

  /****************************************************************************
   * VI. Draw the trajectories.
   *     - Draw the propagation-only trajectory in red;
   *     - Draw the updated trajectory in green.
   ***************************************************************************/
  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_ppg, ori_ppg, loc_upd, ori_upd);
  SLAMTrajectoryDrawer::SetupGLUT(argc, argv);
  SLAMTrajectoryDrawer::SetupGLEW();
  SLAMTrajectoryDrawer::SetupGLSL();
  SLAMTrajectoryDrawer::StartDrawing();
  SLAMTrajectoryDrawer::FreeTrajectory();

  delete tracker;
  return 0;
}
