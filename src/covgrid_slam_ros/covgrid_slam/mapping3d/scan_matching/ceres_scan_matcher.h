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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "covgrid_slam/mapping3d/scan_matching/proto/scan_matcher_options.pb.h"
#include "ivcommon/transform/rigid_transform.h"

#include "covgrid_slam/mapping3d/ceres_pose.h"
#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"

namespace mapping3d {

namespace scan_matching {


proto::ScanMatcherOptions CreateScanMatcherOptions(
    ::ivcommon::LuaParameterDictionary* parameter_dictionary);

using PointCloudAndHybridGridPointers =
    std::pair<const sensor::PointCloud*, const HybridGrid*>;

using PointCloudAndFeatureHybridGridPointers =
    std::pair<const sensor::PointCloud*, const FeatureHybridGrid*>;
using PointCloudAndIntensityHybridGridPointers =
    std::pair<const sensor::PointCloud*, const IntensityHybridGrid*>;
// This scan matcher uses Ceres to align scans with an existing map.
class CeresScanMatcher {
 public:
  explicit CeresScanMatcher(const proto::ScanMatcherOptions& options);

  CeresScanMatcher(const CeresScanMatcher&) = delete;
  CeresScanMatcher& operator=(const CeresScanMatcher&) = delete;

  // Aligns 'point_clouds' within the 'hybrid_grids' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const ivcommon::transform::posestamped& previous_pose,
             const ivcommon::transform::posestamped& initial_pose_estimate,
	     const ivcommon::transform::posestamped& last_pose,
             const std::vector<PointCloudAndHybridGridPointers>&
                 point_clouds_and_hybrid_grids,
	     const std::vector<PointCloudAndFeatureHybridGridPointers>&
		 point_clouds_and_featurehybrid_grids,
		 const std::vector<PointCloudAndIntensityHybridGridPointers>&
		 				 point_clouds_and_intensityhybrid_grids,
             ivcommon::transform::Rigid3d* pose_estimate,
             double* cost);

 private:
  const proto::ScanMatcherOptions options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
