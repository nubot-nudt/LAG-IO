#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Dense>
#include <boost/optional.hpp>

// A stationary contact point implies v_imu + R_world_imu * (omega x lever_arm) = 0.
// The caller supplies the lever-arm velocity in the world frame.
class ZUPTFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  ZUPTFactor(gtsam::Key vel_key, const gtsam::noiseModel::Base::shared_ptr& noise,
             gtsam::Vector3 measurement)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(noise, vel_key), measurement_(measurement) {}

  gtsam::Vector evaluateError(const gtsam::Vector3& foot_velocity,
                              boost::optional<gtsam::Matrix&> H1 = boost::none) const override {
    gtsam::Vector3 residual = foot_velocity + measurement_;

    if (H1) {
      *H1 = gtsam::Matrix3::Identity();
    }

    return residual;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<ZUPTFactor>(*this);
  }

 private:
  gtsam::Vector3 measurement_;
};
