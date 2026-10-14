#ifndef IMUTRACK_H
#define IMUTRACK_H

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <iostream>
#include <map>
#include <vector>

#include "common/rotation.h"
#include "common/types.h"
#include "fileio/filesaver.h"
#include "fileio/imufileloader.h"

// Owns one sensor stream, its inertial state, preintegration, and contact history.
// The centered contact window determines the processing timestamp of every track.
class IMUTrack {
 public:
  FileSaver navfile_, imuerrfile_;

  explicit IMUTrack(Paras& options, std::string label);
  ~IMUTrack() = default;

  ::NavState getNavState();
  inline void setInitGyroBias(const Vector3d& gyro_bias) { imuerror_.gyrbias = gyro_bias; }
  inline void setInitAccBias(const Vector3d& acc_bias) { imuerror_.accbias = acc_bias; }
  inline void setInitAttitude(const double& roll, const double& pitch) {
    pvacur_.att.euler(0) = roll;
    pvacur_.att.euler(1) = pitch;
    pvacur_.att.euler(2) = 0.0;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);
    pvapre_ = pvacur_;
  }
  double timestamp() const { return imucur_.timestamp; }
  void setInitPos(const Eigen::Vector3d& pos) {
    pvacur_.pos = pos;
    pvapre_.pos = pos;
  }

  bool getZuptFlag() const { return if_ZUPT_available_; }
  void setZuptFlag(bool flag) { if_ZUPT_available_ = flag; }
  void setZuptcount(int count) { zuptcount_ = count; }
  IMU getImuCur() { return imucur_; }
  void updatePreState() {
    pvapre_ = pvacur_;
    imupre_ = imucur_;
  }

  void writeState(double timestamp, std::shared_ptr<::NavState> navstate);
  void closeOutputFiles() {
    navfile_.close();
    imuerrfile_.close();
  }

  double getContactProbability() const { return kde_contact_prob_; }

  void initialize();
  void newImuProcess();
  void initStaticAlignment();
  inline double stdNormalCDF(double x) { return 0.5 * (1.0 + std::erf(x * 0.70710678118)); }

  void initPreintegration(Paras& paras);
  void updatePreintegration();
  void resetPreintegration();
  void resetPreintegration(const gtsam::imuBias::ConstantBias& new_bias);
  void setNavState(const ::NavState& navstate);
  void setBias(const gtsam::imuBias::ConstantBias& optimized_bias) {
    imuerror_.accbias = optimized_bias.accelerometer();
    imuerror_.gyrbias = optimized_bias.gyroscope();
  };

  void computeKDE();

  gtsam::PreintegratedCombinedMeasurements& getCurrentPreintegration() {
    return *imu_preintegration_;
  }

 private:
  void openOutputFiles();

  void detectZUPT();

  void insMech();
  void imuCompensate();

  bool addImuData() {
    IMU imu = imufile_.next();

    if (imuBuff_.size() < IMU_win_length) {
      imuBuff_.push_back(imu);
      return false;
    } else {
      imuBuff_.pop_front();
      imuBuff_.push_back(imu);

      imucur_ = imuBuff_[halfIMU_win_length];
      imupre_ = imuBuff_[halfIMU_win_length - 1];
      return true;
    }
  }

  Paras paras_;

  std::string label_;

  ImuFileLoader imufile_;

  IMU imupre_;
  IMU imucur_;

  PVA pvacur_;
  PVA pvapre_;
  ImuError imuerror_;

  std::deque<IMU> imuBuff_;

  int zuptcount_ = 0;

  bool if_ZUPT_available_ = false;

  double kde_contact_prob_ = 0.0;

  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_preintegration_params_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegration_;

  Eigen::Vector3d init_acc_std = Eigen::Vector3d::Zero();
};
#endif
