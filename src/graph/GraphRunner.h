#ifndef GRAPHRUNNER_H
#define GRAPHRUNNER_H

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "common/types.h"
#include "factors/LegPositionFactor.h"
#include "fileio/filesaver.h"

class IMUTrack;
struct KeyFrame;

// Builds the joint body/leg graph and maintains the fixed-lag posterior.
// A separate state cache retains the last estimate of marginalized keyframes.
class GraphRunner {
 public:
  explicit GraphRunner(Paras& options);
  ~GraphRunner() = default;

  Keys addKeyFrameToGraph(std::shared_ptr<KeyFrame> keyframe);
  Keys addKey(std::shared_ptr<KeyFrame> keyframe);

  void addPriorFactor(Keys& keys);
  void InsertKeyFrameValues(std::shared_ptr<KeyFrame> keyframe, Keys& keys);
  void addIMUFactorToGraph(const std::shared_ptr<KeyFrame>& keyframe, IMUTrack* imutrack[IMU_NUM]);
  void addZUPTFactorToGraph(std::shared_ptr<KeyFrame> keyframe, gtsam::Vector3 measurement, int i);
  void addLegPositionFactor(size_t frame_id, int leg_imu_id, const gtsam::Point3& position_in_body,
                            double sigma_relpos);
  double optimize();
  const gtsam::Values& getResult() const { return result_; }
  gtsam::Symbol generateKey(char type_char, size_t frame_id, IMUID imu_id);
  bool parseKey(gtsam::Key k, char& tag, size_t& frame, IMUID& imu);

  double getFrameTimestamp(size_t frame_id) const {
    auto it = frame_timestamps_.find(frame_id);
    return (it != frame_timestamps_.end()) ? it->second : -1.0;
  }

  const std::map<size_t, double>& getFrameTimestamps() const { return frame_timestamps_; }

  const std::map<size_t, OptimizedFrame>& getAllOptimizedStates() const {
    return all_optimized_states_;
  }

 private:
  std::unordered_map<gtsam::Key, std::pair<size_t, IMUID>> key2fi_;
  std::map<size_t, double> frame_timestamps_;

  void updateOptimizedStatesCache();

  Paras paras_;

  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  double lag_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values result_;
  gtsam::Values values_;

  bool is_initialized_ = false;
  size_t last_added_kf_id_ = 0;

  std::map<size_t, OptimizedFrame> all_optimized_states_;
};

#endif
