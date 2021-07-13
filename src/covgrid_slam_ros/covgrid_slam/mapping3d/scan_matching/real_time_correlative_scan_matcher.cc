/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "covgrid_slam/mapping3d/scan_matching/real_time_correlative_scan_matcher.h"

#include <cmath>

#include "Eigen/Geometry"
#include "ivcommon/common/math.h"
#include "ivcommon/transform/transform.h"
#include "glog/logging.h"

namespace mapping3d {
namespace scan_matching {

RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher(
    const mapping::scan_matching::proto::
        RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

float RealTimeCorrelativeScanMatcher::Match(
    const ivcommon::transform::Rigid3d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const HybridGrid& hybrid_grid,
    ivcommon::transform::Rigid3d* pose_estimate) const {
  CHECK_NOTNULL(pose_estimate);
  float best_score = -1.f;
  for (const ivcommon::transform::Rigid3d& transform : GenerateExhaustiveSearchTransforms(
           hybrid_grid.resolution(), point_cloud)) {
    const ivcommon::transform::Rigid3d candidate =
        initial_pose_estimate * transform;
    const float score = ScoreCandidate(
        hybrid_grid, sensor::TransformPointCloud(point_cloud, candidate),
        transform);
    if (score > best_score) {
      best_score = score;
      *pose_estimate = candidate.cast<double>();
    }
  }
  return best_score;
}

std::vector<::ivcommon::transform::Rigid3d>
RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchTransforms(
    const float resolution, const sensor::PointCloud& point_cloud) const {
  std::vector<::ivcommon::transform::Rigid3d> result;
  const int linear_window_size =
      ::ivcommon::RoundToInt(options_.linear_search_window() / resolution);
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;
  for (const Eigen::Vector3d& point : point_cloud) {
    const float range = point.norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const float kSafetyMargin = 1.f - 1e-3f;
  const float angular_step_size =
      kSafetyMargin * std::acos(1.f - ::ivcommon::Pow2(resolution) /
                                          (2.f * ::ivcommon::Pow2(max_scan_range)));
  const int angular_window_size =
      ::ivcommon::RoundToInt(options_.angular_search_window() / angular_step_size);
  for (int z = -linear_window_size; z <= linear_window_size; ++z) {
    for (int y = -linear_window_size; y <= linear_window_size; ++y) {
      for (int x = -linear_window_size; x <= linear_window_size; ++x) {
        for (int rz = -angular_window_size; rz <= angular_window_size; ++rz) {
          for (int ry = -angular_window_size; ry <= angular_window_size; ++ry) {
            for (int rx = -angular_window_size; rx <= angular_window_size; ++rx) {
              const Eigen::Vector3d angle_axis(rx * angular_step_size,
                                               ry * angular_step_size,
                                               rz * angular_step_size);
              result.emplace_back(
                  Eigen::Vector3d(x * resolution, y * resolution,
                                  z * resolution),
                  ivcommon::transform::AngleAxisVectorToRotationQuaternion(angle_axis));
            }
          }
        }
      }
    }
  }
  return result;
}

float RealTimeCorrelativeScanMatcher::ScoreCandidate(
    const HybridGrid& hybrid_grid,
    const sensor::PointCloud& transformed_point_cloud,
    const ivcommon::transform::Rigid3d& transform) const {
  float score = 0.f;
  for (const Eigen::Vector3d& point : transformed_point_cloud) {
    score += hybrid_grid.GetProbability(hybrid_grid.GetCellIndex(point));
  }
  score /= static_cast<float>(transformed_point_cloud.size());
  const float angle = ivcommon::transform::GetAngle(transform);
  score *=
      std::exp(-ivcommon::Pow2(transform.translation().norm() *
                                 options_.translation_delta_cost_weight() +
                             angle * options_.rotation_delta_cost_weight()));
  CHECK_GT(score, 0.f);
  return score;
}

}  // namespace scan_matching
}  // namespace mapping_3d
