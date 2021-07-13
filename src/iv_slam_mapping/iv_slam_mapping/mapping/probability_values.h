#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#include <cmath>
#include <vector>
#include "ivcommon/common/math.h"
#include "ivcommon/common/port.h"
#include "glog/logging.h"

namespace iv_slam_mapping {
namespace mapping {
///
///ｂａｙｅｓ概率更新预处理
///
inline float Odds(float probability) {
  return probability / (1.f - probability);
}
///
///ｂａｙｅｓ概率更新预处理
///
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;///最小概率
constexpr float kMaxProbability = 1.f - kMinProbability;///最大概率
///
/// Clamps probability to be in the range [kMinProbability, kMaxProbability].
///
inline float ClampProbability(const float probability) {
  return ::ivcommon::Clamp(probability, kMinProbability, kMaxProbability);
}

constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;
///
/// Converts a probability to a uint16 in the [1, 32767] range.
///
inline uint16 ProbabilityToValue(const float probability) {
  const int value = ::ivcommon::RoundToInt((ClampProbability(probability) - kMinProbability) * (32766.f / (kMaxProbability - kMinProbability))) +1;
  ///
  /// DCHECK for performance.
  ///
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

extern const std::vector<float>* const kValueToProbability;
///
/// Converts a uint16 (which may or may not have the update marker set) to a
/// probability in the range [kMinProbability, kMaxProbability].
///
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}
///
///计算概率更新表
///
std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);

}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
