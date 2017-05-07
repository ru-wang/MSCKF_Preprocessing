/*
 * Unit test for KITTI dataset.
 * Propagate IMU only.
 */

#include "MSCKF/JPL.h"
#include "MSCKF/MSCKF.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

/*
 * Print some verbose information.
 */ 
#define PRINT_VERBOSE {                                            \
  stringstream ss;                                                 \
  ss << imu_timestamp << ":\n"                                     \
     << "gyro:\t[ " << gyro.transpose() << " ]\n"                  \
     << "acce:\t[ " << acce.transpose() << " ]\n"                  \
     << "velo:\t[ " << velo.transpose() << " ]\n"                  \
     << "q:\t[ " << ekf_propagate_only.orientation().x()           \
     << ", "     << ekf_propagate_only.orientation().y()           \
     << ", "     << ekf_propagate_only.orientation().z()           \
     << ", "     << ekf_propagate_only.orientation().w() << " ]\n" \
     << "p:\t[ " << ekf_propagate_only.position().x()              \
     << ", "     << ekf_propagate_only.position().y()              \
     << ", "     << ekf_propagate_only.position().z() << " ]\n"    \
     << "v_I:\t[ " << v_I.x()                                      \
     << ", "       << v_I.y()                                      \
     << ", "       << v_I.z() << " ]\n"                            \
     << "v_G:\t[ " << v_G.x()                                      \
     << ", "       << v_G.y()                                      \
     << ", "       << v_G.z() << " ]\n";                           \
  /*cv::waitKey(1000);*/                                               \
  cout << ss.str() << endl;                                        \
}

using namespace std;
using namespace Eigen;

namespace bs = ::boost;
namespace fs = ::boost::filesystem;

namespace {
  typedef Matrix<double, 9, 1> Vector9d;

  const Quaterniond INIT_ORIENTATION((Matrix3d() << Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()).finished());

  const double COV_NG = 0.002;
  const double COV_NWG = 0.0005;
  const double COV_NA = 0.05;
  const double COV_NWA = 0.005;
  const double SIGMA_NIM = 0.005;
}

bool NextSensor(Vector3d& gyro_out, Vector3d& acce_out, Vector3d& velo_out, double& timestamp_out, const char* filename = nullptr) {
  static ifstream ifs;
  static vector<pair<Vector9d, double>> imu_data_list;
  static vector<pair<Vector9d, double>>::const_iterator it_cursor = imu_data_list.cend();

  /* load all imu data at once */
  if (filename) {
    ifs.open(filename);
    if (!ifs) {
      cout << "Error: Failed to open " << filename << "!" << endl;
      return false;
    }

    string a_line;
    while (getline(ifs, a_line)) {
      stringstream ss(a_line);
      double timestamp, gx, gy, gz, ax, ay, az, vx, vy, vz;
      ss >> timestamp;
      ss >> gx >> gy >> gz >> ax >> ay >> az >> vx >> vy >> vz;

      Vector9d imu;
      imu << gx, gy, gz, ax, ay, az, vx, vy, vz;
      imu_data_list.push_back(make_pair(imu, timestamp));
      it_cursor = imu_data_list.cbegin();
    }
    ifs.close();
  }

  if (it_cursor == imu_data_list.cend()) {
    cout << "No more imu data!" << endl;
    return false;
  }

  /* read a tuple */
  timestamp_out = it_cursor->second;
  const Vector9d& imu = it_cursor->first;
  gyro_out[0] = imu[0]; gyro_out[1] = imu[1]; gyro_out[2] = imu[2];
  acce_out[0] = imu[3]; acce_out[1] = imu[4]; acce_out[2] = imu[5];
  velo_out[0] = imu[6]; velo_out[1] = imu[7]; velo_out[2] = imu[8];
  ++it_cursor;

  return true;
}

int main(int argc, char* argv[]) {
  assert(argc > 1);
  string imu_file = argv[1];
  double imu_timestamp;
  Vector3d gyro, acce, velo;
  NextSensor(gyro, acce, velo, imu_timestamp, imu_file.c_str());

  /* initialize the MSCKF */
  MSCKF ekf_propagate_only;
  ekf_propagate_only.setNoiseCov(Matrix3d::Identity() * COV_NG,
                                 Matrix3d::Identity() * COV_NWG,
                                 Matrix3d::Identity() * COV_NA,
                                 Matrix3d::Identity() * COV_NWA, SIGMA_NIM);
  ekf_propagate_only.initialize(HamiltonToJPL(INIT_ORIENTATION),
                                Vector3d::Zero(),
                                velo,
                                Vector3d::Zero(),
                                Vector3d::Zero());
  
  ofstream ofs("states.txt");
  do {
    ekf_propagate_only.propagate(imu_timestamp, gyro, acce);

    Vector3d p_I_in_G = ekf_propagate_only.position();
    JPL_Quaternion q_I_to_G = HamiltonToJPL(ekf_propagate_only.orientation());
    Matrix3d R_G_to_I = JPL_CT(q_I_to_G);
    Vector3d v_G = ekf_propagate_only.velocity();
    Vector3d v_I = R_G_to_I * v_G;
    PRINT_VERBOSE

    stringstream statestream;
    statestream << p_I_in_G.x() << " " << p_I_in_G.y() << " " << p_I_in_G.z() << " "
                << q_I_to_G.x() << " " << q_I_to_G.y() << " " << q_I_to_G.z() << " "
                << q_I_to_G.w() << endl;
    ofs << statestream.str();
  } while (NextSensor(gyro, acce, velo, imu_timestamp));
  ofs.close();

  return 0;
}
