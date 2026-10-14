#ifndef SYSTEM_H
#define SYSTEM_H

#pragma once
#include <gtsam/geometry/Pose3.h>

#include <Eigen/Dense>
#include <queue>
#include <vector>

#include "GraphRunner.h"
#include "IMUTrack.h"
#include "common/rotation.h"
#include "common/types.h"
#include "fileio/robotsensorloader.h"

// Coordinates synchronized sensor processing, keyframe construction, and state feedback.
class System {
 public:
  System(Paras& options);
  ~System() = default;

  void run();

  double gettimestamp() const { return imutrack_[1]->timestamp(); }

 private:
  void initialization();

  void initializeGraph();

  void addNewKeyframeToGraph();

  void addIMUFactorToGraph();

  void addZUPTFactorToGraph(std::vector<int> legs_triggered);

  void addMultiPosFactorToGraph(std::vector<int> legs_triggered);

  void computeLegImuPosition(RobotSensor& robotsensor, int legid);

  double optimizeGraph();

  void updateStatesFromGraph();

  void OutputAllKeyFramesFromGraph();

 private:
  Paras paras_;
  RobotSensorLoader robotsensorfile_;
  RobotSensor robotsensor_;

  IMUTrack body_imu_track_;

  IMUTrack fl_imu_track_;
  IMUTrack fr_imu_track_;
  IMUTrack rl_imu_track_;
  IMUTrack rr_imu_track_;

  GraphRunner grunner_;

  IMUTrack* imutrack_[IMU_NUM];

  std::shared_ptr<KeyFrame> framepre_;
  std::shared_ptr<KeyFrame> framecur_;

  double sequence_interval = 0.0;

  Eigen::Matrix<double, 3, IMU_NUM - 1> leg_position_in_body_;
};

#endif
