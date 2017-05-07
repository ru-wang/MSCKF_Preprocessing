/*
 * Unit test for MSCKF in simulation environments.
 * Propagate and update.
 */

#include "SLAMTrajectoryDrawer.h"

#include "MSCKF/MSCKF.h"
#include "MSCKF_Simulation/Helpers.h"

#include <glm/glm.hpp>

#include <iostream>

using namespace std;
using namespace glm;
using namespace Eigen;

namespace {

const double R = 5.0;
const double T = 10.0;
const double w = 2.0 * M_PI / T;

const double NG = 0.002;
const double NWG = 0.005;
const double NA = 0.05;
const double NWA = 0.005;
const double NIM = 0.005;

Gaussian noise(1.0, 10);

vector<Vector3d> MakeFeatures() {
  vector<Vector3d> features;
  Uniform rnd(-1.0, 1.0);
  for (int i = 0; i < 5000; ++i) {
    Vector3d p = rnd.vector3() * R * 3;
    if (Vector3d(p.x(), p.y(), p.z()).norm() < R * 3) {
      features.push_back(p);
    }
  }
  return features;
}

unordered_map<size_t, pair<size_t, Vector2d>> FeatureFrame(const vector<Vector3d>& features,
                                                           const Quaterniond& orientation,
                                                           const Vector3d& position,
                                                                 double NIM) {
  unordered_map<size_t, pair<size_t, Vector2d>> frame;
  Matrix3d R = orientation.toRotationMatrix().transpose();
  for (size_t i = 0; i < features.size(); ++i) {
    const Vector3d &p = features[i];
    Vector3d q = R * (p - position);
    Vector2d z(q.x() / q.z(), q.y() / q.z());
    if (q.z() > 0.5 && q.z() < 5.0 && abs(z.x()) < 1.0 && abs(z.y()) < 1.0)
      frame[i] = make_pair(i, z + noise.vector2() * sqrt(NIM));
  }
  return frame;
}

void LogTrajectory(const MSCKF& ekf, vector<vec3>* loc, vector<vec4>* ori) {
  loc->push_back(vec3(ekf.cameraPosition().x(),
                      ekf.cameraPosition().y(),
                      ekf.cameraPosition().z()));
  JPL_Quaternion q_C_to_G = HamiltonToJPL(ekf.cameraOrientation());
  ori->push_back(vec4(q_C_to_G.x(),
                      q_C_to_G.y(),
                      q_C_to_G.z(),
                      q_C_to_G.w()));
}

}

int main(int argc, char* argv[]) {
  /****************************************************************************
   * I. Generate the first IMU and the first feature frame.
   ****************************************************************************/
  vector<Vector3d> features = MakeFeatures();

  Quaterniond init_ori(Matrix3d::Identity());
  Vector3d gyro(0.0, 0.0, w);
  Vector3d acce(0.0, R*w*w, 0.0);

  /****************************************************************************
   * II. Set up the MSCKFs.
   ****************************************************************************/
  vector<vec3> loc_ppg, loc_upd, loc_gt;
  vector<vec4> ori_ppg, ori_upd, ori_gt;

  Matrix3d R_cam_in_imu((Matrix3d() <<  0,  0,  1,
                                       -1,  0,  0,
                                        0, -1,  0).finished());
  Vector3d p_cam_in_imu(0, 0, 0);
  MSCKF ekf(R_cam_in_imu.transpose(), p_cam_in_imu);
  MSCKF ekf_ppg_only(R_cam_in_imu.transpose(), p_cam_in_imu);
  MSCKF ekf_true(R_cam_in_imu.transpose(), p_cam_in_imu);
  ekf.setNoiseCov(Matrix3d::Identity()*NG, Matrix3d::Identity()*NWG,
                  Matrix3d::Identity()*NA, Matrix3d::Identity()*NWA,
                  NIM);
  ekf_ppg_only.setNoiseCov(Matrix3d::Identity()*NG, Matrix3d::Identity()*NWG,
                           Matrix3d::Identity()*NA, Matrix3d::Identity()*NWA,
                           NIM);
  ekf_true.setNoiseCov(Matrix3d::Identity()*NG, Matrix3d::Identity()*NWG,
                       Matrix3d::Identity()*NA, Matrix3d::Identity()*NWA,
                       NIM);
  ekf.initialize(HamiltonToJPL(init_ori), Vector3d::Zero(),
                 Vector3d(R*w, 0.0, 0.0), Vector3d::Zero(),
                 Vector3d(0, -R, 0), 0.0);
  ekf_ppg_only.initialize(HamiltonToJPL(init_ori), Vector3d::Zero(),
                          Vector3d(R*w, 0.0, 0.0), Vector3d::Zero(),
                          Vector3d(0, -R, 0), 0.0);
  ekf_true.initialize(HamiltonToJPL(init_ori), Vector3d::Zero(),
                      Vector3d(R*w, 0.0, 0.0), Vector3d::Zero(),
                      Vector3d(0, -R, 0), 0.0);

  /****************************************************************************
   * III. Main loop.
   *      - On reading a new IMU, integrate it to both EFKs;
   *      - On reading a new feature frame, let the EFK decide
   *        whether to perform a update.
   ****************************************************************************/
  for (double t = 0.0; t <= 5.0; t += 0.01) {
    Vector3d noisy_gyro = gyro + noise.vector3()*sqrt(NG);
    Vector3d noisy_acce = acce + noise.vector3()*sqrt(NG) + Vector3d(0.02, 0.2, 0.001);

    ekf.propagate(t, noisy_gyro, noisy_acce);
    ekf_ppg_only.propagate(t, noisy_gyro, noisy_acce);
    ekf_true.propagate(t, gyro, acce);

    unordered_map<size_t, pair<size_t, Vector2d>> frame;
    frame = FeatureFrame(features, ekf_true.cameraOrientation(), ekf_true.cameraPosition(), NIM);
    ekf.update(t, frame);

    LogTrajectory(ekf_ppg_only, &loc_ppg, &ori_ppg);
    LogTrajectory(ekf, &loc_upd, &ori_upd);
    LogTrajectory(ekf_true, &loc_gt, &ori_gt);

    cout << "[ " << t << "\t] IMU reading: " << gyro.transpose() << acce.transpose() << endl;
    cout << "[ " << t << "\t] Frame feature number: " << frame.size() << endl;
  }

  /****************************************************************************
   *  IV. Draw the trajectories.
   *      - Draw the updated trajectory in red;
   *      - Draw the ground truth in green.
   ***************************************************************************/

  vector<glm::vec3> landmarks;
  for (const auto& feature : features)
    landmarks.push_back({feature.x(), feature.y(), feature.z()});

  SLAMTrajectoryDrawer::ReadLandmarksFrom(landmarks);
  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_ppg, ori_ppg);
  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_upd, ori_upd);
  SLAMTrajectoryDrawer::ReadTrajectoryFrom(loc_gt, ori_gt);
  SLAMTrajectoryDrawer::SetupGLUT(argc, argv);
  SLAMTrajectoryDrawer::SetupGLEW();
  SLAMTrajectoryDrawer::SetupGLSL();
  SLAMTrajectoryDrawer::StartDrawing();
  SLAMTrajectoryDrawer::FreeTrajectory();

  return 0;
}
