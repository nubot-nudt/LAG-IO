#include "configuration.h"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>

#include "common/types.h"

namespace {
void validateKeys(const YAML::Node& node, const std::vector<std::string>& allowed,
                  const std::string& section) {
  if (!node.IsMap()) throw std::runtime_error(section + " must be a mapping");
  std::set<std::string> seen;
  for (const auto& entry : node) {
    const auto key = entry.first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end())
      throw std::runtime_error("Unsupported " + section + " option: " + key);
    if (!seen.insert(key).second)
      throw std::runtime_error("Duplicate " + section + " option: " + key);
  }
}

inline double positive(const YAML::Node& node, const std::string& key) {
  const double value = node[key].as<double>();
  if (!std::isfinite(value) || value <= 0)
    throw std::runtime_error(key + " must be finite and positive");
  return value;
}

inline Eigen::Vector3d vector3(const YAML::Node& node, const std::string& key,
                               bool require_positive = false) {
  const auto values = node[key].as<std::vector<double>>();
  if (values.size() != 3) throw std::runtime_error(key + " requires three values");
  Eigen::Vector3d result(values.data());
  if (!result.allFinite() || (require_positive && (result.array() <= 0).any()))
    throw std::runtime_error("Invalid vector: " + key);
  return result;
}

}  // namespace

Paras loadConfig(const std::filesystem::path& filename) {
  const auto sequence = YAML::LoadFile(filename.string());

  const std::vector<std::string> allowed = {"estimator",   "datapath",  "outputpath", "imu_Body",
                                            "imu_FL",      "imu_FR",    "imu_RL",     "imu_RR",
                                            "robotsensor", "starttime", "endtime"};
  validateKeys(sequence, allowed, "sequence");
  const auto resolve = [&](const std::string& path) {
    return std::filesystem::absolute(filename.parent_path() / path).lexically_normal();
  };
  const auto config = YAML::LoadFile(resolve(sequence["estimator"].as<std::string>()).string());
  validateKeys(config,
               {"imudatalen",
                "imudatarate",
                "initAlignmentTime",
                "initposstd",
                "initvelstd",
                "initattstd",
                "init_acc_bias_std",
                "init_gyro_bias_std",
                "fl_imu_leverarm",
                "fr_imu_leverarm",
                "rl_imu_leverarm",
                "rr_imu_leverarm",
                "zupt_std",
                "zupt_dt",
                "relpos_std",
                "if_use_multi_imu_constraint",
                "rotmat",
                "base_in_bodyimu",
                "robotpara",
                "relinearizeThreshold",
                "relinearizeSkip",
                "if_enableRelinearization",
                "if_evaluateNonlinearError",
                "if_cacheLinearizedFactors",
                "graph_imunoise",
                "smoother_lag",
                "kde",
                "leg_preintegration",
                "max_keyframe_interval"},
               "estimator");
  validateKeys(config["kde"],
               {"acc_delta", "acc_bandwidth_floor", "gravity_tolerance", "contact_threshold"},
               "kde");
  validateKeys(config["graph_imunoise"], {"sigma_g", "sigma_a", "sigma_bg", "sigma_ba"},
               "graph_imunoise");
  validateKeys(config["robotpara"], {"ox", "oy", "ot", "lc", "lt"}, "robotpara");
  validateKeys(config["leg_preintegration"], {"acc_noise_scale", "acc_bias_noise_scale"},
               "leg_preintegration");
  Paras paras;
  const auto datapath = resolve(sequence["datapath"].as<std::string>());
  paras.outputpath = resolve(sequence["outputpath"].as<std::string>()).string();
  for (const auto* label : {"Body", "FL", "FR", "RL", "RR"}) {
    const auto path = datapath / sequence[std::string("imu_") + label].as<std::string>();
    if (!std::filesystem::is_regular_file(path))
      throw std::runtime_error("Missing IMU input: " + path.string());
    paras.imupaths[label] = path.string();
  }
  paras.robotsensorpath = (datapath / sequence["robotsensor"].as<std::string>()).string();
  if (!std::filesystem::is_regular_file(paras.robotsensorpath))
    throw std::runtime_error("Missing robot input: " + paras.robotsensorpath);
  paras.starttime = sequence["starttime"].as<double>();
  paras.endtime = sequence["endtime"].as<double>();
  paras.initAlignmentTime = config["initAlignmentTime"].as<int>();
  if (!std::isfinite(paras.starttime) || !std::isfinite(paras.endtime) ||
      paras.initAlignmentTime <= 0 || paras.endtime <= paras.starttime + paras.initAlignmentTime)
    throw std::runtime_error("Time range must include alignment and estimation");
  paras.imudatalen = config["imudatalen"].as<int>();
  paras.imudatarate = config["imudatarate"].as<int>();
  if (paras.imudatalen != 7 || paras.imudatarate <= 0)
    throw std::runtime_error("IMU input requires seven columns and a positive sample rate");
  paras.initstate_std.pva.pos = vector3(config, "initposstd", true);
  paras.initstate_std.pva.vel = vector3(config, "initvelstd", true);
  paras.initstate_std.pva.att.euler = vector3(config, "initattstd", true) * D2R;
  paras.initstate_std.imuerror.gyrbias = vector3(config, "init_gyro_bias_std", true);
  paras.initstate_std.imuerror.accbias = vector3(config, "init_acc_bias_std", true);
  for (const auto* leg : {"fl", "fr", "rl", "rr"}) {
    paras.legimus_leverarm.push_back(vector3(config, std::string(leg) + "_imu_leverarm"));
  }
  paras.zupt_std = vector3(config, "zupt_std", true);
  paras.zupt_dt = positive(config, "zupt_dt");
  if (paras.zupt_dt * paras.imudatarate > std::numeric_limits<int>::max())
    throw std::runtime_error("zupt_dt exceeds the supported sample-count range");
  paras.relpos_std = positive(config, "relpos_std");
  paras.if_use_multi_imu_constraint = config["if_use_multi_imu_constraint"].as<bool>();
  paras.relinearizeThreshold = positive(config, "relinearizeThreshold");
  paras.relinearizeSkip = config["relinearizeSkip"].as<int>();
  if (paras.relinearizeSkip < 1) throw std::runtime_error("relinearizeSkip must be positive");
  paras.if_enableRelinearization = config["if_enableRelinearization"].as<bool>();
  paras.if_evaluateNonlinearError = config["if_evaluateNonlinearError"].as<bool>();
  paras.if_cacheLinearizedFactors = config["if_cacheLinearizedFactors"].as<bool>();
  const auto rotation = config["rotmat"].as<std::vector<double>>();
  if (rotation.size() != 9) throw std::runtime_error("rotmat requires nine values");
  paras.robotbody_rotmat =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
  if (!paras.robotbody_rotmat.allFinite() ||
      !(paras.robotbody_rotmat.transpose() * paras.robotbody_rotmat)
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(paras.robotbody_rotmat.determinant() - 1) > 1e-6)
    throw std::runtime_error("rotmat must be a proper rotation");
  paras.base_in_bodyimu = vector3(config, "base_in_bodyimu");
  const auto robot = config["robotpara"];
  paras.robotpara.ox = positive(robot, "ox");
  paras.robotpara.oy = positive(robot, "oy");
  paras.robotpara.ot = positive(robot, "ot");
  paras.robotpara.lc = positive(robot, "lc");
  paras.robotpara.lt = positive(robot, "lt");
  for (const auto& lever_arm : paras.legimus_leverarm) {
    if (paras.robotpara.lc - lever_arm.z() <= 0)
      throw std::runtime_error("Leg IMU mounting offset must leave a positive calf segment");
  }
  const auto graph = config["graph_imunoise"];
  paras.graph_imunoise.sigma_g = vector3(graph, "sigma_g", true);
  paras.graph_imunoise.sigma_a = vector3(graph, "sigma_a", true);
  paras.graph_imunoise.sigma_bg = vector3(graph, "sigma_bg", true);
  paras.graph_imunoise.sigma_ba = vector3(graph, "sigma_ba", true);
  paras.smoother_lag = positive(config, "smoother_lag");
  paras.max_keyframe_interval = positive(config, "max_keyframe_interval");
  if (paras.max_keyframe_interval >= paras.smoother_lag)
    throw std::runtime_error("max_keyframe_interval must be shorter than smoother_lag");
  const auto kde = config["kde"];
  paras.kde_acc_delta = positive(kde, "acc_delta");
  paras.kde_bandwidth_floor = positive(kde, "acc_bandwidth_floor");
  paras.kde_gravity_tolerance = positive(kde, "gravity_tolerance");
  paras.kde_contact_threshold = positive(kde, "contact_threshold");
  if (paras.kde_contact_threshold >= 1)
    throw std::runtime_error("contact_threshold must be below one");
  paras.leg_acc_noise_scale = positive(config["leg_preintegration"], "acc_noise_scale");
  paras.leg_acc_bias_noise_scale = positive(config["leg_preintegration"], "acc_bias_noise_scale");
  std::filesystem::create_directories(paras.outputpath);
  // Save the exact settings used for this run alongside the outputs.
  std::ofstream settings(std::filesystem::path(paras.outputpath) / "estimator.yaml");
  if (!settings) throw std::runtime_error("Cannot write estimator configuration snapshot");
  settings << config << '\n';
  settings.close();
  if (!settings) throw std::runtime_error("Cannot save estimator configuration snapshot");
  return paras;
}
