/*!
* \file g2o_scan_matcher.h
* \brief g2o点云匹配，估计车辆位姿
*
* 调用g2o接口，进行点云匹配，估计车辆位姿
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/
#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_G2O_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_G2O_SCAN_MATCHER_H_

#include <utility>
#include <vector>
#include "Eigen/Core"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "covgrid_slam/mapping3d/scan_matching/proto/scan_matcher_options.pb.h"
#include "ivcommon/transform/rigid_transform.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
//#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "covgrid_slam/mapping3d/ceres_pose.h"
#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/mapping3d/scan_matching/ceres_scan_matcher.h"
#include "covgrid_slam/mapping3d/scan_matching/g2o_feature_space_cost_functor.h"
#include "covgrid_slam/sensor/point_cloud.h"


namespace mapping3d {

namespace scan_matching {

using PointCloudAndHybridGridPointers =
    std::pair<const sensor::PointCloud*, const HybridGrid*>;

using PointCloudAndFeatureHybridGridPointers =
    std::pair<const sensor::PointCloud*, const FeatureHybridGrid*>;

// This scan matcher uses G2O to align scans with an existing map.
/// \brief 利用g2o扫描匹配，优化估计位姿
///
class G2oScanMatcher {
 public:
  explicit G2oScanMatcher(const proto::ScanMatcherOptions& options);

  G2oScanMatcher(const G2oScanMatcher&) = delete;
  G2oScanMatcher& operator=(const G2oScanMatcher&) = delete;

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
             ivcommon::transform::Rigid3d* pose_estimate,
             double* cost);

 private:
  const proto::ScanMatcherOptions options_;//!< 优化参数
  g2o::SparseOptimizer optimizer;//!< 优化器
  g2o::OptimizationAlgorithmLevenberg* solver;//!<LM 结算器
};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_G2O_SCAN_MATCHER_H_
