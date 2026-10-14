#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// Couples the leg IMU position to an encoder-derived mounting point in body axes.
// Leg orientation is estimated by the inertial chain, not this position residual.
class LegPositionFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
  gtsam::Point3 position_in_body_;

 public:
  typedef boost::shared_ptr<LegPositionFactor> shared_ptr;

  LegPositionFactor(gtsam::Key bodyKey, gtsam::Key legKey, const gtsam::Point3& position_in_body,
                    gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, bodyKey, legKey),
        position_in_body_(position_in_body) {}

  gtsam::Vector errorFunc(const gtsam::Pose3& X_body, const gtsam::Pose3& X_leg) const {
    gtsam::Point3 expected_position = X_body.transformFrom(position_in_body_);

    gtsam::Point3 actual_position = X_leg.translation();

    return actual_position - expected_position;
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& X_body, const gtsam::Pose3& X_leg,
                              boost::optional<gtsam::Matrix&> H_body = boost::none,
                              boost::optional<gtsam::Matrix&> H_leg = boost::none) const override {
    if (H_body) {
      *H_body = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(
          [this](const gtsam::Pose3& b, const gtsam::Pose3& l) { return this->errorFunc(b, l); },
          X_body, X_leg, 1e-5);
    }

    if (H_leg) {
      *H_leg = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(
          [this](const gtsam::Pose3& b, const gtsam::Pose3& l) { return this->errorFunc(b, l); },
          X_body, X_leg, 1e-5);
    }

    return errorFunc(X_body, X_leg);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<LegPositionFactor>(*this);
  }
};
