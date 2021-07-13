#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_
#include <memory>
#include <vector>
#include "Eigen/Geometry"
#include "ivcommon/common/math.h"
#include "ivcommon/common/port.h"
#include "ivcommon/transform/rigid_transform.h"
#include "iv_slam_mapping/mapping/probability_values.h"
#include "iv_slam_mapping/mapping/proto/serialization.pb.h"
#include "iv_slam_mapping/mapping_3d/proto/submap.pb.h" 
#include "glog/logging.h"

namespace iv_slam_mapping {
namespace mapping {
/// 
/// Converts the given probability to log odds.
  /// 
inline float Logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = Logit(kMaxProbability);/// 最大概率对应的ｌｏｇ
const float kMinLogOdds = Logit(kMinProbability);/// 最小概率嘴硬的ｌｏｇ
/// 
/// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
/// kMaxLogOdds] is mapped to [1, 255].
/// 
inline uint8 ProbabilityToLogOddsInteger(const float probability) {
  const int value = ::ivcommon::RoundToInt((Logit(probability) - kMinLogOdds) * 254.f / (kMaxLogOdds - kMinLogOdds)) + 1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}
/// 
/// An individual submap, which has a 'local_pose' in the local SLAM frame, keeps
/// track of how many range data were inserted into it, and sets the
/// 'finished_probability_grid' to be used for loop closing once the map no
/// longer changes.
/// 
class Submap {
 public:
  Submap(const ivcommon::transform::Rigid3d& local_pose) : local_pose_(local_pose) {
    gps_local_pose = ivcommon::transform::Rigid3d::Identity();
  }
  virtual ~Submap() {}

  virtual void ToProto(mapping_3d::proto::Submap3D* proto) const = 0;
/// 
  /// Local SLAM pose of this submap.
  /// 
  ivcommon::transform::Rigid3d local_pose() const { return local_pose_; }
/// 
  /// Number of RangeData inserted.
  /// 
  int num_range_data() const { return num_range_data_; }
/// 
  /// Fills data into the 'response'.
///   virtual void ToResponseProto(
///       const ivcommon::transform::Rigid3d& global_submap_pose,
///       proto::SubmapQuery::Response* response) const = 0;
/// 
ivcommon::transform::Rigid3d gps_local_pose;
 protected:
  void SetNumRangeData(const int num_range_data) {
    num_range_data_ = num_range_data;
  }
  const ivcommon::transform::Rigid3d local_pose_;
 private:

  int num_range_data_ = 0;
};
}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
