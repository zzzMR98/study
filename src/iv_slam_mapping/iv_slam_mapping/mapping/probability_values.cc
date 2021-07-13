#include "iv_slam_mapping/mapping/probability_values.h"
namespace iv_slam_mapping {
namespace mapping {
namespace {
///
/// 0 is unknown, [1, 32767] maps to [kMinProbability, kMaxProbability].
  ///
float SlowValueToProbability(const uint16 value) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == kUnknownProbabilityValue) {
    ///
    /// Unknown cells have kMinProbability.
    ///
    return kMinProbability;
  }
  const float kScale = (kMaxProbability - kMinProbability) / 32766.f;
  return value * kScale + (kMinProbability - kScale);///线性关系转为概率值
}

const std::vector<float>* PrecomputeValueToProbability() {
  std::vector<float>* result = new std::vector<float>;
  ///
  /// Repeat two times, so that both values with and without the update marker
  /// can be converted to a probability.
  ///
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToProbability(value));
    }
  }
  return result;
}
}  // namespace

const std::vector<float>* const kValueToProbability = PrecomputeValueToProbability();///计算概率表
///
///计算概率更新表
///
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) + kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(  odds * Odds((*kValueToProbability)[cell]))) + kUpdateMarker);
  }
  return result;
}

}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping
