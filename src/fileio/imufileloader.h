#pragma once

#include <cmath>
#include <cstring>
#include <stdexcept>

#include "common/types.h"
#include "fileloader.h"

// Binary rows: time [s], angular velocity [rad/s], specific force [m/s^2].
class ImuFileLoader : public FileLoader {
 public:
  ImuFileLoader(const string& filename, int columns = 7, int rate = 200)
      : nominal_dt_(1.0 / rate), filename_(filename) {
    if (columns != 7 || rate <= 0) throw std::runtime_error("Invalid IMU record layout or rate");
    if (!open(filename, columns)) throw std::runtime_error("Cannot open IMU input: " + filename);
  }

  const IMU& next() {
    const auto data = load();
    const double previous = imu_.timestamp;
    imu_.timestamp = data[0];
    std::memcpy(imu_.angular_velocity.data(), &data[1], 3 * sizeof(double));
    std::memcpy(imu_.acceleration.data(), &data[4], 3 * sizeof(double));
    if (!std::isfinite(imu_.timestamp) || !imu_.angular_velocity.allFinite() ||
        !imu_.acceleration.allFinite())
      throw std::runtime_error("Non-finite IMU record: " + filename_);
    imu_.dt = initialized_ ? imu_.timestamp - previous : nominal_dt_;
    if (imu_.dt <= 0 || (initialized_ && std::abs(imu_.dt - nominal_dt_) > 1e-6))
      throw std::runtime_error(
          "IMU timestamps must be strictly increasing at the configured rate: " + filename_);
    initialized_ = true;
    return imu_;
  }

 private:
  double nominal_dt_;
  string filename_;
  bool initialized_ = false;
  IMU imu_;
};
