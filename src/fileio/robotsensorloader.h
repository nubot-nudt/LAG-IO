#pragma once

#include <cmath>
#include <cstring>
#include <stdexcept>

#include "common/types.h"
#include "fileloader.h"

// Binary rows: time, 12 joint angles, 12 joint rates, four foot forces.
class RobotSensorLoader : public FileLoader {
 public:
  RobotSensorLoader(const string& filename, int columns = 29, int rate = 200)
      : nominal_dt_(1.0 / rate), filename_(filename) {
    if (columns != 29 || rate <= 0) throw std::runtime_error("Invalid robot record layout or rate");
    if (!open(filename, columns)) throw std::runtime_error("Cannot open robot input: " + filename);
  }

  const RobotSensor& next() {
    const auto data = load();
    const double previous = sensor_.timestamp;
    sensor_.timestamp = data[0];
    std::memcpy(sensor_.joint_angular_position.data(), &data[1], 12 * sizeof(double));
    std::memcpy(sensor_.joint_angular_velocity.data(), &data[13], 12 * sizeof(double));
    std::memcpy(sensor_.footforce.data(), &data[25], 4 * sizeof(double));
    if (!std::isfinite(sensor_.timestamp) || !sensor_.joint_angular_position.allFinite() ||
        !sensor_.joint_angular_velocity.allFinite() || !sensor_.footforce.allFinite())
      throw std::runtime_error("Non-finite robot record: " + filename_);
    if (initialized_ && std::abs(sensor_.timestamp - previous - nominal_dt_) > 1e-6)
      throw std::runtime_error("Robot timestamps must match the configured sample rate: " +
                               filename_);
    initialized_ = true;
    return sensor_;
  }

 private:
  double nominal_dt_;
  string filename_;
  bool initialized_ = false;
  RobotSensor sensor_;
};
