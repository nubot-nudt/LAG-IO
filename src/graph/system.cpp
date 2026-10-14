#include "system.h"

#include <absl/time/clock.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <iomanip>
#include <numeric>

#include "GraphRunner.h"
#include "common/types.h"

System::System(Paras& paras)
    : paras_(paras),
      robotsensorfile_(paras.robotsensorpath, 29, paras.imudatarate),
      body_imu_track_(paras, "Body"),
      fl_imu_track_(paras, "FL"),
      fr_imu_track_(paras, "FR"),
      rl_imu_track_(paras, "RL"),
      rr_imu_track_(paras, "RR"),
      grunner_(paras) {
  imutrack_[0] = &body_imu_track_;
  imutrack_[1] = &fl_imu_track_;
  imutrack_[2] = &fr_imu_track_;
  imutrack_[3] = &rl_imu_track_;
  imutrack_[4] = &rr_imu_track_;

  sequence_interval = paras_.endtime - (paras_.starttime + paras_.initAlignmentTime);
}

void System::run() {
  initialization();
  initializeGraph();
  optimizeGraph();
  for (auto* track : imutrack_) track->initPreintegration(paras_);
  const auto started = absl::Now();
  int last_percent = -1;
  size_t contact_frames = 0, inertial_frames = 0;

  while (gettimestamp() + 0.5 / paras_.imudatarate < paras_.endtime) {
    for (auto* track : imutrack_) track->newImuProcess();
    const double timestamp = gettimestamp();
    for (auto* track : imutrack_) {
      if (std::abs(track->timestamp() - timestamp) > 1e-6)
        throw std::runtime_error("IMU streams must have matching timestamps");
    }
    // Match the delayed center of the KDE window, not the newest input sample.
    while (robotsensor_.timestamp < timestamp - 1e-6) robotsensor_ = robotsensorfile_.next();
    if (std::abs(robotsensor_.timestamp - timestamp) > 1e-6)
      throw std::runtime_error("Robot and IMU timestamps do not match");

    std::vector<int> contacts;
    for (int i = 1; i < IMU_NUM; ++i)
      if (imutrack_[i]->getZuptFlag()) contacts.push_back(i);

    const bool interval_elapsed =
        timestamp - framecur_->timestamp >= paras_.max_keyframe_interval - 1e-6;
    const bool final_sample = timestamp + 0.5 / paras_.imudatarate >= paras_.endtime;
    // Contact adds velocity and kinematic observations. Timeout frames add only inertial links.
    if (!contacts.empty() || interval_elapsed || final_sample) {
      if (contacts.empty())
        ++inertial_frames;
      else
        ++contact_frames;
      addNewKeyframeToGraph();
      addIMUFactorToGraph();
      addZUPTFactorToGraph(contacts);
      addMultiPosFactorToGraph(contacts);
      optimizeGraph();
      updateStatesFromGraph();
      for (int i : contacts) {
        imutrack_[i]->setZuptFlag(false);
        imutrack_[i]->setZuptcount(0);
      }
    }
    for (auto* track : imutrack_) track->updatePreState();
    const int percent = static_cast<int>(
        100 * (timestamp - paras_.starttime - paras_.initAlignmentTime) / sequence_interval);
    if (percent > last_percent) {
      std::cout << "Processing: " << percent << "%\r" << std::flush;
      last_percent = percent;
    }
  }
  OutputAllKeyFramesFromGraph();
  for (auto* track : imutrack_) track->closeOutputFiles();
  std::cout << "Keyframes with contact: " << contact_frames
            << "; inertial-only keyframes: " << inertial_frames << '\n';
  std::cout << "LAG-IO finished in " << absl::ToDoubleSeconds(absl::Now() - started) << " s\n";
}

void System::initialization() {
  for (int i = 0; i < IMU_NUM; i++) {
    imutrack_[i]->initStaticAlignment();
  }

  do {
    robotsensor_ = robotsensorfile_.next();
  } while (robotsensor_.timestamp < paras_.starttime);

  while (robotsensor_.timestamp < gettimestamp() - 1e-6) robotsensor_ = robotsensorfile_.next();
  if (std::abs(robotsensor_.timestamp - gettimestamp()) > 1e-6)
    throw std::runtime_error("Robot data does not cover the alignment timestamp");
  leg_position_in_body_.setZero();

  for (int leg = 0; leg < IMU_NUM - 1; ++leg) computeLegImuPosition(robotsensor_, leg);

  Eigen::Matrix3d R_wb = body_imu_track_.getNavState().pva.att.cbn;
  Eigen::Vector3d p_wb = body_imu_track_.getNavState().pva.pos;

  for (int i = 1; i < IMU_NUM; ++i) {
    int legid = i - 1;
    Eigen::Vector3d p_leg_body = leg_position_in_body_.col(legid) + paras_.base_in_bodyimu;

    // Kinematics already returns the IMU mounting point in body coordinates.
    Eigen::Vector3d p_leg_world = p_wb + R_wb * p_leg_body;

    auto nav = imutrack_[i]->getNavState();
    nav.pva.pos = p_leg_world;

    imutrack_[i]->setNavState(nav);

    imutrack_[i]->setInitPos(p_leg_world);
  }
}

void System::computeLegImuPosition(RobotSensor& robotsensor, int legid) {
  int lfoot = -1, ffoot = -1;
  if (legid < 2) ffoot = 1;
  if (legid == 0 || legid == 2) lfoot = 1;

  double ox = paras_.robotpara.ox;
  double oy = paras_.robotpara.oy;
  double ot = paras_.robotpara.ot;
  // The longitudinal mounting offset shortens the calf segment to the IMU point.
  // Lateral lever-arm components are used by the contact velocity correction.
  double lc = paras_.robotpara.lc - paras_.legimus_leverarm[legid].z();
  double lt = paras_.robotpara.lt;

  Eigen::Vector3d joint_angular_position = robotsensor.joint_angular_position.segment(legid * 3, 3);

  double s1 = sin(joint_angular_position(0));
  double s2 = sin(joint_angular_position(1));
  double s23 = sin(joint_angular_position(1) + joint_angular_position(2));

  double c1 = cos(joint_angular_position(0));
  double c2 = cos(joint_angular_position(1));
  double c23 = cos(joint_angular_position(1) + joint_angular_position(2));

  leg_position_in_body_(0, legid) = -lt * s2 - lc * s23 + ffoot * ox;
  leg_position_in_body_(1, legid) = lfoot * ot * c1 + lc * s1 * c23 + lt * c2 * s1 + lfoot * oy;
  leg_position_in_body_(2, legid) = lfoot * ot * s1 - lc * c1 * c23 - lt * c1 * c2;

  leg_position_in_body_.block(0, legid, 3, 1) =
      paras_.robotbody_rotmat * leg_position_in_body_.block(0, legid, 3, 1);
}

void System::initializeGraph() {
  framepre_ = std::make_shared<KeyFrame>();
  framepre_->kf_id = 0;
  framepre_->timestamp = imutrack_[0]->timestamp();
  for (int i = 0; i < IMU_NUM; i++) {
    framepre_->navstates[i].pva = imutrack_[i]->getNavState().pva;
    framepre_->navstates[i].imuerror = imutrack_[i]->getNavState().imuerror;
  }
  framepre_->robotsensor = robotsensor_;
  framecur_ = framepre_;

  grunner_.addKeyFrameToGraph(framecur_);
}

void System::addNewKeyframeToGraph() {
  std::shared_ptr<KeyFrame> new_keyframe = std::make_shared<KeyFrame>();
  new_keyframe->kf_id = framecur_->kf_id + 1;
  new_keyframe->timestamp = gettimestamp();

  for (int i = 0; i < IMU_NUM; i++) {
    const NavState& prev_navstate = framecur_->navstates[i];
    gtsam::Pose3 prev_pose(gtsam::Rot3(prev_navstate.pva.att.cbn), prev_navstate.pva.pos);
    gtsam::Vector3 prev_vel = prev_navstate.pva.vel;
    gtsam::imuBias::ConstantBias prev_bias(prev_navstate.imuerror.accbias,
                                           prev_navstate.imuerror.gyrbias);

    gtsam::PreintegratedCombinedMeasurements& preint = imutrack_[i]->getCurrentPreintegration();
    gtsam::NavState predicted_navstate =
        preint.predict(gtsam::NavState(prev_pose, prev_vel), prev_bias);

    new_keyframe->navstates[i].pva.att.cbn = predicted_navstate.attitude().matrix();
    new_keyframe->navstates[i].pva.att.qbn = predicted_navstate.attitude().toQuaternion();
    new_keyframe->navstates[i].pva.att.euler =
        Rotation::matrix2euler(predicted_navstate.attitude().matrix());
    new_keyframe->navstates[i].imuerror = prev_navstate.imuerror;

    new_keyframe->navstates[i].pva.pos = predicted_navstate.position();
    new_keyframe->navstates[i].pva.vel = predicted_navstate.velocity();
  }
  new_keyframe->robotsensor = robotsensor_;
  framepre_ = framecur_;
  framecur_ = new_keyframe;

  grunner_.addKeyFrameToGraph(framecur_);
}

void System::addIMUFactorToGraph() { grunner_.addIMUFactorToGraph(framecur_, imutrack_); }

void System::addZUPTFactorToGraph(std::vector<int> legs_triggered) {
  for (int i : legs_triggered) {
    gtsam::Vector3 measurement;
    Eigen::Vector3d vel_leverarm =
        framecur_->navstates[i].pva.att.cbn *
        Rotation::skewSymmetric(imutrack_[i]->getImuCur().angular_velocity) *
        paras_.legimus_leverarm[i - 1];

    measurement << vel_leverarm(0), vel_leverarm(1), vel_leverarm(2);

    grunner_.addZUPTFactorToGraph(framecur_, measurement, i);
  }
}

double System::optimizeGraph() { return grunner_.optimize(); }

void System::updateStatesFromGraph() {
  size_t id = framecur_->kf_id;

  for (int i = 0; i < IMU_NUM; i++) {
    IMUID imu_id = static_cast<IMUID>(i);

    gtsam::Symbol pose_key = grunner_.generateKey('x', id, imu_id);
    gtsam::Symbol vel_key = grunner_.generateKey('v', id, imu_id);
    gtsam::Symbol bias_key = grunner_.generateKey('b', id, imu_id);

    if (!grunner_.getResult().exists(pose_key)) {
      std::cout << "[Warning] Pose key not found for IMU " << i << std::endl;
      continue;
    }

    gtsam::Pose3 optimized_pose = grunner_.getResult().at<gtsam::Pose3>(pose_key);
    gtsam::Vector3 optimized_vel = grunner_.getResult().at<gtsam::Vector3>(vel_key);
    gtsam::imuBias::ConstantBias optimized_bias =
        grunner_.getResult().at<gtsam::imuBias::ConstantBias>(bias_key);

    NavState updated_navstate;
    updated_navstate.pva.pos =
        Eigen::Vector3d(optimized_pose.translation().x(), optimized_pose.translation().y(),
                        optimized_pose.translation().z());
    updated_navstate.pva.att.cbn = optimized_pose.rotation().matrix();
    updated_navstate.pva.att.qbn = Rotation::matrix2quaternion(updated_navstate.pva.att.cbn);
    updated_navstate.pva.att.euler = Rotation::matrix2euler(updated_navstate.pva.att.cbn);
    updated_navstate.pva.vel =
        Eigen::Vector3d(optimized_vel(0), optimized_vel(1), optimized_vel(2));
    updated_navstate.imuerror.accbias =
        Eigen::Vector3d(optimized_bias.accelerometer()(0), optimized_bias.accelerometer()(1),
                        optimized_bias.accelerometer()(2));
    updated_navstate.imuerror.gyrbias =
        Eigen::Vector3d(optimized_bias.gyroscope()(0), optimized_bias.gyroscope()(1),
                        optimized_bias.gyroscope()(2));

    imutrack_[i]->setNavState(updated_navstate);

    framecur_->navstates[i] = updated_navstate;
  }

  for (int i = 0; i < IMU_NUM; i++) {
    imutrack_[i]->resetPreintegration();
  }
}

void System::OutputAllKeyFramesFromGraph() {
  const auto& all_states = grunner_.getAllOptimizedStates();

  std::cout << "[Output] Writing " << all_states.size() << " frames" << std::endl;

  for (const auto& [frame_id, optimized_state] : all_states) {
    double timestamp = optimized_state.timestamp;

    if (timestamp < 0) {
      std::cerr << "[Output] Invalid timestamp for frame " << frame_id << std::endl;
      continue;
    }

    for (const auto& [imu_id, pose] : optimized_state.poses) {
      int i = static_cast<int>(imu_id);

      auto nav = std::make_shared<NavState>();
      nav->pva.pos = pose.translation();
      nav->pva.att.cbn = pose.rotation().matrix();
      nav->pva.att.qbn = Rotation::matrix2quaternion(nav->pva.att.cbn);
      nav->pva.att.euler = Rotation::matrix2euler(nav->pva.att.cbn);

      if (optimized_state.vel.count(imu_id)) {
        nav->pva.vel = optimized_state.vel.at(imu_id);
      }
      if (optimized_state.bias.count(imu_id)) {
        nav->imuerror.accbias = optimized_state.bias.at(imu_id).accelerometer();
        nav->imuerror.gyrbias = optimized_state.bias.at(imu_id).gyroscope();
      }

      imutrack_[i]->writeState(timestamp, nav);
    }
  }

  std::cout << "[Output] Finished writing trajectory" << std::endl;
}

void System::addMultiPosFactorToGraph(std::vector<int> legs_triggered) {
  if (!paras_.if_use_multi_imu_constraint) return;
  for (int i : legs_triggered) {
    int legid = i - 1;

    computeLegImuPosition(robotsensor_, legid);
    Eigen::Vector3d position_in_body = leg_position_in_body_.col(legid) + paras_.base_in_bodyimu;

    gtsam::Point3 measurement(position_in_body.x(), position_in_body.y(), position_in_body.z());

    grunner_.addLegPositionFactor(framecur_->kf_id, i, measurement, paras_.relpos_std);
  }
}
