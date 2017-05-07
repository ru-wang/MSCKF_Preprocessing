#ifndef MSCKF_PREPROCESSING_EXCEPTION_H_
#define MSCKF_PREPROCESSING_EXCEPTION_H_

#include <stdexcept>
#include <string>

class FileNotFound : public std::runtime_error {
 public:
  FileNotFound(const std::string& filename, const std::string& type = "File")
      : runtime_error(type + " '" + filename + "' does not exist!") {}
};

class UnknownException : public std::runtime_error {
 public:
  UnknownException() : runtime_error("Unknown exception") {}
};

class IncompleteCablirationFile : public std::runtime_error {
 public:
  IncompleteCablirationFile()
      : runtime_error("Incomplete calibration file!") {}
};

class NoImuWarning : public std::runtime_error {
 public:
  NoImuWarning() : runtime_error("No more IMU data!") {}
};

class NoImageWarning : public std::runtime_error {
 public:
  NoImageWarning() : runtime_error("No more images!") {}
};

class YAMLParseError : public std::runtime_error {
 public:
   YAMLParseError() : runtime_error("YAML parse error!") {}
};

#endif  // MSCKF_PREPROCESSING_EXCEPTION_H_