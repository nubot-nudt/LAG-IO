#include "IMUTrack.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <vector>

#include "common/rotation.h"
#include "common/types.h"
#include "fileio/filesaver.h"
#include "fileio/imufileloader.h"
#include "fileio/trajectorywriter.h"

IMUTrack::IMUTrack(Paras& paras, std::string label)
    : paras_(paras), imufile_(paras.imupaths.at(label), paras.imudatalen, paras.imudatarate) {
  label_ = label;
  initialize();
  openOutputFiles();
}

void IMUTrack::initStaticAlignment() {
  do {
    imucur_ = imufile_.next();
  } while (imucur_.timestamp < paras_.starttime);

  int k = 0;
  Vector3d init_gyro_mean = Vector3d::Zero();
  Vector3d init_acc_mean = Vector3d::Zero();

  std::vector<Eigen::Vector3d> acc_buffer;

  while (imucur_.timestamp < paras_.starttime + paras_.initAlignmentTime) {
    if (!addImuData()) continue;

    init_gyro_mean += imucur_.angular_velocity;
    init_acc_mean += imucur_.acceleration;
    acc_buffer.push_back(imucur_.acceleration);

    k++;
  }
  if (k < 2) throw std::runtime_error(label_ + ": insufficient static alignment samples");
  init_gyro_mean /= k;
  init_acc_mean /= k;

  Vector3d acc_sq_diff_sum = Vector3d::Zero();

  for (int i = 0; i < k; ++i) {
    Vector3d acc_diff = acc_buffer[i] - init_acc_mean;

    acc_sq_diff_sum += acc_diff.array().square().matrix();
  }

  // A floor avoids division by zero on noiseless or quantized stationary input.
  init_acc_std = (acc_sq_diff_sum.array() / (k - 1)).sqrt().max(paras_.kde_bandwidth_floor);
  if (label_ != "Body") {
    double maximum_probability = 1.0;
    for (int axis = 0; axis < 3; ++axis)
      maximum_probability *= std::erf(paras_.kde_acc_delta / (std::sqrt(2.0) * init_acc_std[axis]));
    if (maximum_probability <= paras_.kde_contact_threshold)
      throw std::runtime_error(label_ +
                               ": alignment variance makes the contact threshold unreachable; use "
                               "a stationary alignment interval");
  }

  double init_roll = atan2(-init_acc_mean[1], -init_acc_mean[2]);
  double init_pitch = atan2(init_acc_mean[0], sqrt(init_acc_mean[1] * init_acc_mean[1] +
                                                   init_acc_mean[2] * init_acc_mean[2]));
  setInitAttitude(init_roll, init_pitch);
  setInitGyroBias(init_gyro_mean);
}

void IMUTrack::newImuProcess() {
  addImuData();

  updatePreintegration();
  imuCompensate();

  insMech();

  if (label_ != "Body") {
    detectZUPT();
  }
}

void IMUTrack::imuCompensate() {
  imucur_.angular_velocity -= imuerror_.gyrbias;
  imucur_.acceleration -= imuerror_.accbias;
  imupre_.angular_velocity -= imuerror_.gyrbias;
  imupre_.acceleration -= imuerror_.accbias;
}

void IMUTrack::insMech() {
  Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl;
  Eigen::Vector3d temp1, temp2, temp3;
  Eigen::Vector3d imucur_dvel, imucur_dtheta, imupre_dvel, imupre_dtheta;

  imucur_dvel = imucur_.acceleration * imucur_.dt;
  imucur_dtheta = imucur_.angular_velocity * imucur_.dt;
  imupre_dvel = imupre_.acceleration * imupre_.dt;
  imupre_dtheta = imupre_.angular_velocity * imupre_.dt;

  temp1 = imucur_dtheta.cross(imucur_dvel) / 2;
  temp2 = imupre_dtheta.cross(imucur_dvel) / 12;
  temp3 = imupre_dvel.cross(imucur_dtheta) / 12;

  d_vfb = imucur_dvel + temp1 + temp2 + temp3;

  d_vfn = pvapre_.att.cbn * d_vfb;

  gl << 0, 0, NormG;
  d_vgn = gl * imucur_.dt;

  pvacur_.vel = pvapre_.vel + d_vfn + d_vgn;

  Eigen::Vector3d midvel;

  midvel = (pvacur_.vel + pvapre_.vel) / 2;
  pvacur_.pos = pvapre_.pos + midvel * imucur_.dt;

  Eigen::Quaterniond qbb;
  Eigen::Vector3d rot_bframe;

  rot_bframe = imucur_dtheta + imupre_dtheta.cross(imucur_dtheta) / 12;
  qbb = Rotation::rotvec2quaternion(rot_bframe);

  pvacur_.att.qbn = pvapre_.att.qbn * qbb;
  pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
  pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);
}

void IMUTrack::initialize() {
  pvacur_.pos = paras_.initstate.pva.pos;
  pvacur_.vel = paras_.initstate.pva.vel;
  pvacur_.att.euler = paras_.initstate.pva.att.euler;
  pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
  pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);
  imuerror_ = paras_.initstate.imuerror;
  pvapre_ = pvacur_;
}

void IMUTrack::initPreintegration(Paras& paras) {
  imu_preintegration_params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(NormG);

  Eigen::Vector3d sigma_g = paras.graph_imunoise.sigma_g;
  Eigen::Vector3d sigma_a = paras.graph_imunoise.sigma_a;
  Eigen::Vector3d sigma_bg = paras.graph_imunoise.sigma_bg;
  Eigen::Vector3d sigma_ba = paras.graph_imunoise.sigma_ba;

  if (label_ != "Body") {
    sigma_a *= paras.leg_acc_noise_scale;

    sigma_ba *= paras.leg_acc_bias_noise_scale;
  }
  Eigen::Vector3d var_g = sigma_g.cwiseProduct(sigma_g);
  Eigen::Vector3d var_a = sigma_a.cwiseProduct(sigma_a);
  Eigen::Vector3d var_bg = sigma_bg.cwiseProduct(sigma_bg);
  Eigen::Vector3d var_ba = sigma_ba.cwiseProduct(sigma_ba);

  imu_preintegration_params_->setGyroscopeCovariance(var_g.asDiagonal());
  imu_preintegration_params_->setAccelerometerCovariance(var_a.asDiagonal());

  imu_preintegration_params_->setBiasOmegaCovariance(var_bg.asDiagonal());
  imu_preintegration_params_->setBiasAccCovariance(var_ba.asDiagonal());

  imu_preintegration_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      imu_preintegration_params_,
      gtsam::imuBias::ConstantBias(imuerror_.accbias, imuerror_.gyrbias));
}

void IMUTrack::updatePreintegration() {
  const IMU& imu = imucur_;

  imu_preintegration_->integrateMeasurement(imu.acceleration, imu.angular_velocity, imu.dt);
}

void IMUTrack::resetPreintegration() {
  imu_preintegration_->resetIntegrationAndSetBias(
      gtsam::imuBias::ConstantBias(imuerror_.accbias, imuerror_.gyrbias));
}

void IMUTrack::resetPreintegration(const gtsam::imuBias::ConstantBias& new_bias) {
  if (imu_preintegration_) {
    imu_preintegration_->resetIntegrationAndSetBias(new_bias);
  }
}

NavState IMUTrack::getNavState() {
  NavState state;
  state.pva = pvacur_;
  state.imuerror = imuerror_;
  return state;
}

void IMUTrack::detectZUPT() {
  if (imuBuff_.size() < IMU_win_length) {
    return;
  }
  computeKDE();

  if (kde_contact_prob_ > paras_.kde_contact_threshold) {
    zuptcount_++;

    int zupt_dt_int =
        std::max(1, static_cast<int>(std::lround(paras_.zupt_dt * paras_.imudatarate)));
    if (zuptcount_ % zupt_dt_int == 0) {
      if_ZUPT_available_ = true;
    }

  } else {
    zuptcount_ = 0;

    if_ZUPT_available_ = false;
  }
}

void IMUTrack::setNavState(const NavState& navstate) {
  pvapre_ = pvacur_;
  pvacur_ = navstate.pva;
  imuerror_ = navstate.imuerror;
}

void IMUTrack::openOutputFiles() {
  navfile_.open(paras_.outputpath + "/" + label_ + "_traj.txt", FileSaver::TEXT);
  imuerrfile_.open(paras_.outputpath + "/" + label_ + "_imuerror.txt", FileSaver::TEXT);
}

void IMUTrack::writeState(double timestamp, std::shared_ptr<NavState> navstate) {
  writeNavResult(timestamp, navstate, navfile_, imuerrfile_);
}

// Acceleration-only contact probability, integrated over a centered sample window.
void IMUTrack::computeKDE() {
  kde_contact_prob_ = 0.0;
  if (imuBuff_.size() < IMU_win_length) return;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& imu : imuBuff_) mean += imu.acceleration;
  mean /= imuBuff_.size();
  if (std::abs(mean.norm() - NormG) > paras_.kde_gravity_tolerance) return;

  Eigen::Vector3d probability = Eigen::Vector3d::Zero();
  for (const auto& imu : imuBuff_) {
    const Eigen::Vector3d residual = imu.acceleration - mean;
    for (int axis = 0; axis < 3; ++axis) {
      probability[axis] +=
          stdNormalCDF((paras_.kde_acc_delta - residual[axis]) / init_acc_std[axis]) -
          stdNormalCDF((-paras_.kde_acc_delta - residual[axis]) / init_acc_std[axis]);
    }
  }
  kde_contact_prob_ = (probability / imuBuff_.size()).prod();
}
