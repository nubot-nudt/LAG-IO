#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <math.h>

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <string>
#include <vector>

const double D2R = (M_PI / 180.0);
const double R2D = (180.0 / M_PI);

const int IMU_NUM = 5;

enum IMUID { BODY = 0, FL = 1, FR = 2, RL = 3, RR = 4 };

const int IMU_win_length = 11;
const int halfIMU_win_length = (int)(IMU_win_length / 2);

static constexpr double NormG = 9.782940329221166;

using JointVector = Eigen::Matrix<double, 12, 1>;

struct RobotSensor {
  double timestamp = 0.0;
  JointVector joint_angular_position = JointVector::Zero();
  JointVector joint_angular_velocity = JointVector::Zero();
  Eigen::Vector4d footforce = Eigen::Vector4d::Zero();
};

struct IMU {
  double timestamp = 0.0;
  double dt = 0.0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

struct Attitude {
  Eigen::Quaterniond qbn = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d cbn = Eigen::Matrix3d::Identity();
  Eigen::Vector3d euler = Eigen::Vector3d::Zero();
};

struct PVA {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Attitude att;
};

struct ImuError {
  Eigen::Vector3d gyrbias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accbias = Eigen::Vector3d::Zero();
};

struct NavState {
  PVA pva;
  ImuError imuerror;
};

struct GraphImuNoise {
  Eigen::Vector3d sigma_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d sigma_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d sigma_bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d sigma_ba = Eigen::Vector3d::Zero();
};

struct RobotPara {
  double ox = 0.0;
  double oy = 0.0;
  double ot = 0.0;
  double lc = 0.0;
  double lt = 0.0;
};

struct KeyFrame {
  size_t kf_id = static_cast<size_t>(-1);
  double timestamp = 0.0;
  std::map<int, ::NavState> navstates;
  RobotSensor robotsensor;
};

struct OptimizedFrame {
  size_t kf_id = static_cast<size_t>(-1);
  double timestamp = 0.0;
  std::map<int, gtsam::Pose3> poses;
  std::map<int, gtsam::Vector3> vel;
  std::map<int, gtsam::imuBias::ConstantBias> bias;
};

struct Keys {
  std::map<IMUID, gtsam::Symbol> pose_keys;
  std::map<IMUID, gtsam::Symbol> vel_keys;
  std::map<IMUID, gtsam::Symbol> bias_keys;
};

struct Paras {
  ::NavState initstate;
  ::NavState initstate_std;

  std::vector<Eigen::Vector3d> legimus_leverarm;

  Eigen::Vector3d zupt_std = Eigen::Vector3d::Zero();

  double zupt_dt = 0.0;

  double starttime = 0.0, endtime = 0.0;

  int imudatarate = 0, imudatalen = 0;

  int initAlignmentTime = 0;

  double relpos_std = 0.1;

  std::map<std::string, std::string> imupaths;

  std::string robotsensorpath, outputpath;

  Eigen::Matrix3d robotbody_rotmat = Eigen::Matrix3d::Identity();

  Eigen::Vector3d base_in_bodyimu = Eigen::Vector3d::Zero();

  bool if_use_multi_imu_constraint = false;

  RobotPara robotpara;

  double relinearizeThreshold = 0;
  int relinearizeSkip = 1;
  bool if_enableRelinearization = true;
  bool if_evaluateNonlinearError = false;
  bool if_cacheLinearizedFactors = true;

  GraphImuNoise graph_imunoise;

  double smoother_lag = 0.2;
  double max_keyframe_interval = 0.1;
  double kde_acc_delta = 1.5;
  double kde_bandwidth_floor = 1e-6;
  double kde_gravity_tolerance = 4.0;
  double kde_contact_threshold = 0.1;
  double leg_acc_noise_scale = 15.0;
  double leg_acc_bias_noise_scale = 10.0;
};
