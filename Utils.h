#ifndef MSCKF_PREPROCESSING_UTILS_H_
#define MSCKF_PREPROCESSING_UTILS_H_

#include <boost/algorithm/string.hpp>

#include <Eigen/Eigen>

#include <cmath>
#include <istream>
#include <string>
#include <vector>

namespace utils {

/* The Earth's radius in meter: 6378137(m) */
extern const double EARTH_RADIUS;

/* PI */
extern const double PI;

template<typename T>
std::vector<T> SplitToVector(const std::string& str,
                             const std::string& delimiters = " ",
                             const bool token_compress = true);

std::istream& SafelyGetLine(std::istream& is, std::string* str);

/*
 * Calculate the time interval between `timestamp` and `last_timestamp`.
 *
 * Args:
 *   last_timestamp (string): The last timestamp in the format "YYYY-MM-DD hh:mm:ss.XXXX", where XXXX is the fraction.
 *   timestamp      (string): The current timestamp in the format  "YYYY-MM-DD hh:mm:ss.XXXX", where XXXX is the fraction.
 *
 * Return:
 *   time_interval (double): The time interval between last timestamp and current timestamp
 *                           in second.
 *
 * Example:
 *   t_interval = TimestampSub("2011-09-26 13:02:25.604340603",
 *                             "2011-09-26 13:02:25.594360375");
 *
 * Note:
 *   Subtracting a timestamp by a larger timestamp is undefined.
 */
template<typename T = double>
T TimestampSub(const std::string& timestamp, const std::string& last_timestamp);

/*
 * Convert a JPL quanternion to a rotation matrix.
 *                                  | q1 |
 * Assume the input quanternion q = | q2 |, where q4 is the real part.
 *                                  | q3 |
 *                                  | q4 |
 * The output matrix is stored in coloumn major order.
 *
 * See the reference:
 *   Trawny, N., & Roumeliotis, S. I. (2005). Indirect Kalman filter for 3D
 *   attitude estimation. University of Minnesota, Dept. of Comp. Sci. & Eng.,
 *   Tech. Rep, 2, 2005.
 */
template<typename T = double> void JPLQuatToMat(const T quanternion[], T matrix[]);

/*
 * Convert a rotation matrix to a JPL quanternion.
 * Assume the input matrix is stored in coloumn major order.
 *                                    | q1 |
 * The output quanternion is like q = | q2 |, where q4 is the real part.
 *                                    | q3 |
 *                                    | q4 |
 *
 * See the reference:
 *   Trawny, N., & Roumeliotis, S. I. (2005). Indirect Kalman filter for 3D
 *   attitude estimation. University of Minnesota, Dept. of Comp. Sci. & Eng.,
 *   Tech. Rep, 2, 2005.
 */
template<typename T = double> void RotToJPLQuat(const T matrix[], T quanternion[]);

/* 
 * Compute mercator scale from latitude.
 */
template<typename T = double> T LatToScale(const T latitude);

/*
 * Convert latitude/longitude coordinates to mercator coordinates using mercator scale.
 */
template<typename T = double>
Eigen::Matrix<T, 3, 1> LatLonToMercator(const T latitude,
                                        const T longitude,
                                        const T altitude,
                                        const T scale);

/*
 * Convert the OXTS euler angles to a rotation matrix.
 *
 * See the reference:
 *   OXTS RT3000 user manual, page 71/92.
 */
template<typename T = double>
Eigen::Matrix<T, 3, 3> OXTSEulerToRotationMatrix(const T roll,
                                                 const T pitch,
                                                 const T heading);
}

template<typename T>
T utils::TimestampSub(const std::string& timestamp, const std::string& last_timestamp) {
  double interval = 0.0;
  if (last_timestamp == timestamp)
    return interval;

  std::vector<std::string> strs;
  boost::split(strs, last_timestamp, boost::is_any_of("\t "), boost::token_compress_on);
  std::vector<std::string> last_date {strs.front()};
  std::vector<std::string> last_time {strs.back()};

  strs.clear();
  boost::split(strs, timestamp, boost::is_any_of("\t "), boost::token_compress_on);
  std::vector<std::string> curr_date {strs.front()};
  std::vector<std::string> curr_time {strs.back()};

  boost::split(last_time, last_time.front(), boost::is_any_of(":"));
  boost::split(curr_time, curr_time.front(), boost::is_any_of(":"));

  boost::split(last_date, last_date.front(), boost::is_any_of("-"));
  boost::split(curr_date, last_time.front(), boost::is_any_of("-"));

  /*
   * TODO Currently we only consider the situation
   *      where all the timestamps are of the same date.
   */

  interval = std::stod(curr_time[2]) - std::stod(last_time[2]);
  interval += (std::stoi(curr_time[0]) - std::stoi(last_time[0])) * 60 * 60;
  interval += (std::stoi(curr_time[1]) - std::stoi(last_time[1])) * 60;

  return (T)interval;
}

template<typename T>
void utils::JPLQuatToMat(const T quanternion[], T matrix[]) {
  Eigen::Matrix<T, 4, 1> q_bar;
  q_bar << quanternion[0],
           quanternion[1],
           quanternion[2],
           quanternion[3];
  q_bar.normalize();
  const T q1 = q_bar.x(),
          q2 = q_bar.y(),
          q3 = q_bar.z(),
          q4 = q_bar.w();

  Eigen::Matrix<T, 4, 3> Xi, Psi;
  Xi <<  q4, -q3,  q2,
         q3,  q4, -q1,
        -q2,  q1,  q4,
        -q1, -q2, -q3;
  Psi <<  q4,  q3, -q2,
         -q3,  q4,  q1,
          q2, -q1,  q4,
         -q1, -q2, -q3;
  Eigen::Matrix<T, 3, 3> rot = Xi.transpose() * Psi;

  memcpy(matrix, rot.data(), sizeof(T) * 9);
}

template<typename T>
void utils::RotToJPLQuat(const T matrix[], T quanternion[]) {
  T c11 = matrix[0], c12 = matrix[3], c13 = matrix[6],
    c21 = matrix[1], c22 = matrix[4], c23 = matrix[7],
    c31 = matrix[2], c32 = matrix[5], c33 = matrix[8];

  T trace = c11 + c22 + c33;
  Eigen::Matrix<T, 4, 1> q_bar;
  q_bar.w() = sqrt((trace + 1)) / 2;
  q_bar.x() = (c23 - c32) / (4 * q_bar.w());
  q_bar.y() = (c31 - c13) / (4 * q_bar.w());
  q_bar.z() = (c12 - c21) / (4 * q_bar.w());
  q_bar.normalize();

  quanternion[0] = q_bar.x();
  quanternion[1] = q_bar.y();
  quanternion[2] = q_bar.z();
  quanternion[3] = q_bar.w();
}

template<typename T>
T utils::LatToScale(const T latitude) {
  return std::cos(latitude * PI / 180);
}

template<typename T>
Eigen::Matrix<T, 3, 1> utils::LatLonToMercator(const T latitude,
                                               const T longitude,
                                               const T altitude,
                                               const T scale) {
  T x = scale * longitude * PI * EARTH_RADIUS / 180;
  T y = scale * EARTH_RADIUS * std::log(std::tan((90 + latitude) * PI / 360));
  T z = altitude;
  return Eigen::Matrix<T, 3, 1>(x, y, z);
}

template<typename T>
Eigen::Matrix<T, 3, 3> utils::OXTSEulerToRotationMatrix(const T roll,
                                                        const T pitch,
                                                        const T heading) {
  T psi = heading;
  T theta = pitch;
  T phi = roll;
  Eigen::Matrix<T, 3, 3> R_psi, R_theta, R_phi;
  R_psi << std::cos(psi), -std::sin(psi), 0,
           std::sin(psi),  std::cos(psi), 0,
                       0,              0, 1;
  R_theta <<  std::cos(theta), 0, std::sin(theta),
                            0, 1,               0,
             -std::sin(theta), 0, std::cos(theta);
  R_phi << 1,             0,              0,
           0, std::cos(phi), -std::sin(phi),
           0, std::sin(phi),  std::cos(phi);
  return R_psi * R_theta * R_phi;
}

#endif  // MSCKF_PREPROCESSING_UTILS_H_
