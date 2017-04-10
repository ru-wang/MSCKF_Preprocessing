#ifndef MSCKF_PREPROCESSING_UTILS_H_
#define MSCKF_PREPROCESSING_UTILS_H_

#include <boost/algorithm/string.hpp>

#include <cstdlib>
#include <fstream>
#include <istream>
#include <streambuf>
#include <string>
#include <vector>

namespace utils {

std::istream& SafelyGetLine(std::istream& is, std::string* str) {
  str->clear();
  std::streambuf* sb = is.rdbuf();
  while (true) {
    char ch = sb->sbumpc();
    switch (ch) {
      case '\n': return is;
      case '\r': if (sb->sgetc() == '\n')
                   sb->sbumpc();
                 return is;
      case EOF:  if (str->empty())
                   is.setstate(std::ifstream::eofbit);
                 return is;
      default:   (*str) += ch;
    }
  }
}

/*
 * Calculate the time interval between `timestamp` and `last_timestamp`.
 *
 * Args:
 *   last_timestamp (string): The last timestamp in the format "YYYY-MM-DD hh:mm:ss.XXXX", where XXXX is the fraction
 *   timestamp      (string): The current timestamp in the format  "YYYY-MM-DD hh:mm:ss.XXXX", where XXXX is the fraction
 *
 * Return:
 *   time_interval (double): The time interval between last timestamp and current timestamp
 *                           in second
 *
 * Example:
 *   t_interval = TimestampSub("2011-09-26 13:02:25.604340603",
 *                             "2011-09-26 13:02:25.594360375");
 *
 * Note:
 *   Subtracting a timestamp by a larger timestamp is undefined
 */
double TimestampSub(const std::string& timestamp, const std::string& last_timestamp) {
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
   *      where all the timestamps are of the same date
   */

  interval = std::stod(curr_time[2]) - std::stod(last_time[2]);
  interval += (std::stoi(curr_time[0]) - std::stoi(last_time[0])) * 60 * 60;
  interval += (std::stoi(curr_time[1]) - std::stoi(last_time[1])) * 60;

  return interval;
}

}

#endif  // MSCKF_PREPROCESSING_UTILS_H_
