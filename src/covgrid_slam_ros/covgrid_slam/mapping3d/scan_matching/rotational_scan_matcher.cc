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

#include "covgrid_slam/mapping3d/scan_matching/rotational_scan_matcher.h"

#include <map>
#include <vector>

#include "ivcommon/common/math.h"
#include "covgrid_slam/mapping3d/scan_matching/rotational_histogram.h"

namespace mapping3d {
namespace scan_matching {


RotationalScanMatcher::RotationalScanMatcher(
    const std::vector<mapping::TrajectoryNode>& nodes, const int histogram_size)
    : histogram_(Eigen::VectorXf::Zero(histogram_size)) {
  for (const mapping::TrajectoryNode& node : nodes) {
    AddValuesToHistogram(
        GetValuesForHistogram(sensor::TransformPointCloud(
            node.constant_data->range_data.returns.Decompress(),
            node.pose)),
        0.f, &histogram_);
  }
}

RotationalScanMatcher::RotationalScanMatcher(
    const Eigen::VectorXf histogram)
{
	histogram_ = histogram;
}

std::vector<float> RotationalScanMatcher::Match(
    const sensor::PointCloud& point_cloud,
    const std::vector<float>& angles) const {
  std::vector<float> result;
  result.reserve(angles.size());
  const std::vector<AngleValuePair> value_vector =
      GetValuesForHistogram(point_cloud);
  for (const float angle : angles) {
    Eigen::VectorXf scan_histogram = Eigen::VectorXf::Zero(histogram_.size());
    AddValuesToHistogram(value_vector, angle, &scan_histogram);
    result.push_back(MatchHistogram(scan_histogram));
  }
  return result;
}

float RotationalScanMatcher::MatchHistogram(
    const Eigen::VectorXf& scan_histogram) const {
  // We compute the dot product of normalized histograms as a measure of
  // similarity.
  const float scan_histogram_norm = scan_histogram.norm();
  const float histogram_norm = histogram_.norm();
  const float normalization = scan_histogram_norm * histogram_norm;
  if (normalization < 1e-3f) {
    return 1.f;
  }
  return histogram_.dot(scan_histogram) / normalization;
}

}  // namespace scan_matching
}  // namespace mapping_3d
