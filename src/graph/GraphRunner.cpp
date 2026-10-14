#include "GraphRunner.h"

#include <absl/time/clock.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <iomanip>
#include <iostream>
#include <vector>

#include "IMUTrack.h"
#include "common/types.h"
#include "factors/ZUPTFactor.h"
#include "fileio/filesaver.h"

GraphRunner::GraphRunner(Paras& Paras) : paras_(Paras) {
  gtsam::ISAM2Params params;
  params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
  params.relinearizeThreshold = paras_.relinearizeThreshold;
  params.relinearizeSkip = paras_.relinearizeSkip;
  params.enableRelinearization = paras_.if_enableRelinearization;
  params.evaluateNonlinearError = paras_.if_evaluateNonlinearError;
  params.cacheLinearizedFactors = paras_.if_cacheLinearizedFactors;
  params.factorization = gtsam::ISAM2Params::QR;

  lag_ = paras_.smoother_lag;
  smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(lag_, params);
}

Keys GraphRunner::addKeyFrameToGraph(std::shared_ptr<KeyFrame> keyframe) {
  last_added_kf_id_ = keyframe->kf_id;
  Keys keys_ = addKey(keyframe);
  InsertKeyFrameValues(keyframe, keys_);

  size_t kf_id = keyframe->kf_id;

  frame_timestamps_[kf_id] = keyframe->timestamp;

  if (!is_initialized_) {
    addPriorFactor(keys_);
    is_initialized_ = true;
  }
  return keys_;
}

gtsam::Symbol GraphRunner::generateKey(char type_char, size_t frame_id, IMUID imu_id) {
  const size_t idx = frame_id * IMU_NUM + static_cast<size_t>(imu_id);
  gtsam::Symbol sym(type_char, idx);
  key2fi_[sym.key()] = {frame_id, imu_id};
  return sym;
}

bool GraphRunner::parseKey(gtsam::Key k, char& tag, size_t& frame, IMUID& imu) {
  tag = gtsam::Symbol(k).chr();
  auto it = key2fi_.find(k);
  if (it != key2fi_.end()) {
    frame = it->second.first;
    imu = it->second.second;
    return true;
  }
  return false;
}

Keys GraphRunner::addKey(std::shared_ptr<KeyFrame> keyframe) {
  Keys keys_;
  const size_t frame_id = keyframe->kf_id;

  for (int i = 0; i < IMU_NUM; ++i) {
    IMUID imu_id = static_cast<IMUID>(i);
    keys_.pose_keys[imu_id] = generateKey('x', frame_id, imu_id);
    keys_.vel_keys[imu_id] = generateKey('v', frame_id, imu_id);
    keys_.bias_keys[imu_id] = generateKey('b', frame_id, imu_id);
  }
  return keys_;
}

void GraphRunner::InsertKeyFrameValues(std::shared_ptr<KeyFrame> keyframe, Keys& keys) {
  values_.clear();

  for (int i = 0; i < IMU_NUM; ++i) {
    IMUID imu_id = static_cast<IMUID>(i);
    const auto& navstate = keyframe->navstates.at(imu_id);

    values_.insert(
        keys.pose_keys.at(imu_id),
        gtsam::Pose3(gtsam::Rot3(navstate.pva.att.cbn),
                     gtsam::Point3(navstate.pva.pos(0), navstate.pva.pos(1), navstate.pva.pos(2))));

    values_.insert(keys.vel_keys.at(imu_id),
                   gtsam::Vector3(navstate.pva.vel(0), navstate.pva.vel(1), navstate.pva.vel(2)));

    values_.insert(
        keys.bias_keys.at(imu_id),
        gtsam::imuBias::ConstantBias(navstate.imuerror.accbias, navstate.imuerror.gyrbias));
  }
}

void GraphRunner::addPriorFactor(Keys& keys) {
  for (int i = 0; i < IMU_NUM; ++i) {
    IMUID imu_id = static_cast<IMUID>(i);

    gtsam::Vector6 pose_sigmas;
    // GTSAM Pose3 tangent coordinates place rotation before translation.
    pose_sigmas << paras_.initstate_std.pva.att.euler, paras_.initstate_std.pva.pos;

    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas);
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(keys.pose_keys.at(imu_id),
                                                values_.at<gtsam::Pose3>(keys.pose_keys.at(imu_id)),
                                                pose_noise));

    const gtsam::Vector3 vel_sigmas = paras_.initstate_std.pva.vel;
    auto vel_noise = gtsam::noiseModel::Diagonal::Sigmas(vel_sigmas);
    graph_.add(gtsam::PriorFactor<gtsam::Vector3>(
        keys.vel_keys.at(imu_id), values_.at<gtsam::Vector3>(keys.vel_keys.at(imu_id)), vel_noise));

    gtsam::Vector6 bias_sigmas;
    bias_sigmas << paras_.initstate_std.imuerror.accbias, paras_.initstate_std.imuerror.gyrbias;
    auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas);
    graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        keys.bias_keys.at(imu_id),
        values_.at<gtsam::imuBias::ConstantBias>(keys.bias_keys.at(imu_id)), bias_noise));
  }
}

void GraphRunner::addIMUFactorToGraph(const std::shared_ptr<KeyFrame>& keyframe,
                                      IMUTrack* imutrack[IMU_NUM]) {
  const size_t frame = keyframe->kf_id;
  if (frame == 0) return;
  for (int i = 0; i < IMU_NUM; ++i) {
    const auto imu = static_cast<IMUID>(i);
    const auto& preintegration = imutrack[i]->getCurrentPreintegration();
    graph_.emplace_shared<gtsam::CombinedImuFactor>(
        generateKey('x', frame - 1, imu), generateKey('v', frame - 1, imu),
        generateKey('x', frame, imu), generateKey('v', frame, imu),
        generateKey('b', frame - 1, imu), generateKey('b', frame, imu), preintegration);
  }
}

void GraphRunner::addZUPTFactorToGraph(std::shared_ptr<KeyFrame> keyframe,
                                       gtsam::Vector3 measurement, int i) {
  IMUID imu_id = static_cast<IMUID>(i);
  gtsam::Key vel_key = generateKey('v', keyframe->kf_id, imu_id);

  Eigen::Vector3d zupt_std_scale = paras_.zupt_std;

  auto noise = gtsam::noiseModel::Diagonal::Sigmas(zupt_std_scale);

  auto factor = boost::make_shared<ZUPTFactor>(vel_key, noise, measurement);
  graph_.add(factor);
}

double GraphRunner::optimize() {
  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps;

  const size_t current_frame_id = last_added_kf_id_;
  const double current_timestamp = frame_timestamps_.at(current_frame_id);

  for (const auto& key_value : values_) {
    new_timestamps[key_value.key] = current_timestamp;
  }

  auto t_opt_start = absl::Now();

  // Insert one synchronized keyframe, then marginalize states outside the time window.
  smoother_->update(graph_, values_, new_timestamps);
  result_ = smoother_->calculateEstimate();

  auto t_opt_end = absl::Now();
  double pure_opt_marg_time = absl::ToDoubleMilliseconds(t_opt_end - t_opt_start);

  // Retain active-state estimates for trajectory export before the next marginalization.
  updateOptimizedStatesCache();

  graph_.resize(0);
  values_.clear();
  return pure_opt_marg_time;
}

void GraphRunner::updateOptimizedStatesCache() {
  std::map<size_t, std::map<IMUID, gtsam::Pose3>> frame_poses;
  std::map<size_t, std::map<IMUID, gtsam::Vector3>> frame_vels;
  std::map<size_t, std::map<IMUID, gtsam::imuBias::ConstantBias>> frame_biases;

  for (const auto& key_value : result_) {
    char tag;
    size_t frame_id;
    IMUID imu_id;

    if (!parseKey(key_value.key, tag, frame_id, imu_id)) continue;

    if (tag == 'x') {
      frame_poses[frame_id][imu_id] = result_.at<gtsam::Pose3>(key_value.key);
    } else if (tag == 'v') {
      frame_vels[frame_id][imu_id] = result_.at<gtsam::Vector3>(key_value.key);
    } else if (tag == 'b') {
      frame_biases[frame_id][imu_id] = result_.at<gtsam::imuBias::ConstantBias>(key_value.key);
    }
  }

  for (const auto& [frame_id, poses] : frame_poses) {
    OptimizedFrame optimized_state;
    optimized_state.kf_id = frame_id;

    auto ts_it = frame_timestamps_.find(frame_id);
    optimized_state.timestamp = (ts_it != frame_timestamps_.end()) ? ts_it->second : -1.0;

    for (const auto& [imu_id, pose] : poses) {
      optimized_state.poses[imu_id] = pose;

      if (frame_vels.count(frame_id) && frame_vels[frame_id].count(imu_id)) {
        optimized_state.vel[imu_id] = frame_vels[frame_id][imu_id];
      }

      if (frame_biases.count(frame_id) && frame_biases[frame_id].count(imu_id)) {
        optimized_state.bias[imu_id] = frame_biases[frame_id][imu_id];
      }
    }

    all_optimized_states_[frame_id] = optimized_state;
  }
}

void GraphRunner::addLegPositionFactor(size_t frame_id, int i,
                                       const gtsam::Point3& position_in_body, double sigma_relpos) {
  IMUID imu_id = static_cast<IMUID>(i);

  gtsam::Key body_key = generateKey('x', frame_id, BODY);
  gtsam::Key leg_key = generateKey('x', frame_id, imu_id);

  auto model =
      gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(3),
                                        gtsam::noiseModel::Isotropic::Sigma(3, sigma_relpos));

  graph_.add(boost::make_shared<LegPositionFactor>(body_key, leg_key, position_in_body, model));
}
