/*!
* \file g2o_scan_matcher.cc
* \brief g2o点云匹配，估计车辆位姿
*
* 调用g2o接口，进行点云匹配，估计车辆位姿
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/

#include "covgrid_slam/mapping3d/scan_matching/g2o_scan_matcher.h"

#include <string>
#include <utility>
#include <vector>

#include "ivcommon/common/make_unique.h"
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

#include "covgrid_slam/common/optimize_solver_options.h"
#include "covgrid_slam/mapping3d/scan_matching/g2o_feature_space_cost_functor.h"
#include "covgrid_slam/mapping3d/scan_matching/g2o_vertexse3_delta_cost_functor.h"
#include "covgrid_slam/mapping3d/scan_matching/occupied_space_cost_functor.h"

namespace mapping3d {
namespace scan_matching {


G2oScanMatcher::G2oScanMatcher(
    const proto::ScanMatcherOptions& options)
    : options_(options) {
	solver = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(false);

}

/// \brief 点云与地图匹配
///
/// \param previous_pose 预估计位姿
/// \param initial_pose_estimate 优化初始位姿
/// \param last_pose 上一次位姿
/// \param point_clouds_and_hybrid_grids 采样点云和栅格地图
/// \param point_clouds_and_featurehybrid_grids 采样点云和特征栅格地图
/// \param pose_estimate 位姿估计
/// \param cost 优化最终的损失
void G2oScanMatcher::Match(const ivcommon::transform::posestamped& previous_pose,
			     const ivcommon::transform::posestamped& initial_pose_estimate,
			     const ivcommon::transform::posestamped& last_pose,
                             const std::vector<PointCloudAndHybridGridPointers>&
                                 point_clouds_and_hybrid_grids,
			     const std::vector<PointCloudAndFeatureHybridGridPointers>&
				 point_clouds_and_featurehybrid_grids,
                             ivcommon::transform::Rigid3d* const pose_estimate,
                             double* const cost) {

  // set up two poses

//	LOG(INFO)<<"g2o optimize";
    optimizer.clear();
    int vertex_id = 0;

//    LOG(INFO)<<initial_pose_estimate.pose;
	Eigen::Isometry3d cam; // camera pose
	cam = initial_pose_estimate.pose.rotation();
	cam.translation() = initial_pose_estimate.pose.translation();

	// set up node
	g2o::VertexSE3 *pose = new g2o::VertexSE3();
	pose->setEstimate(cam);

	pose->setId(vertex_id);      // vertex id

//	    std::cerr << t.transpose() << " | " << q.coeffs().transpose() << std::endl;

	// set first cam pose fixed


	// add to optimizer
	optimizer.addVertex(pose);

	const sensor::PointCloud& point_cloud =
	*point_clouds_and_featurehybrid_grids[0].first;
	const FeatureHybridGrid& hybrid_grid = *point_clouds_and_featurehybrid_grids[0].second;
	LOG(INFO)<<"featurn num in this scan:"<<point_cloud.size();
	LOG(INFO)<<"confirmed number in submap: " << hybrid_grid.get_confirmed_voxel_number();

	int i=0;
	const float thHuber3D = 0.2;//sqrt(0.5);
	for(auto& point:point_cloud)
	{
//		double range = point.norm();
//		if(range<10)
//			range = 10;
//		double rangeweight = 1/(log(range));
		FeatureSpaceEdge* featureedge = new FeatureSpaceEdge(
								point, hybrid_grid,{last_pose,initial_pose_estimate});
		featureedge->setId(i);
		featureedge->setVertex(0,pose);
		Eigen::Matrix<double,1,1> information =  Eigen::Matrix<double,1,1>::Identity()*
							options_.feature_space_weight(0)*options_.feature_space_weight(0) /
								    (static_cast<double>(point_cloud.size()));
		featureedge->setInformation(information);
//		featureedge->setInformation(Eigen::Matrix<double,1,1>::Identity()*4);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(thHuber3D);
        featureedge->setRobustKernel(rk);

		i++;
		optimizer.addEdge(featureedge);

	}
	double timediff = ::ivcommon::ToSeconds(initial_pose_estimate.time - last_pose.time);
	double scale = 0.2/timediff;
//	scale = 1.;
	if(options_.rotation_weight()>0 && options_.translation_weight()>0)
	{
		Eigen::Matrix<double,6,6> information;
		information.setZero();
		information.block<3,3>(0,0) = Eigen::Matrix<double,3,3>::Identity()
				* options_.translation_weight()*options_.translation_weight()*scale*scale;
		information.block<3,3>(3,3) = Eigen::Matrix<double,3,3>::Identity()
				* options_.rotation_weight()*options_.rotation_weight();
	    Eigen::Isometry3d isopose;
	    isopose = previous_pose.pose.rotation();
	    isopose.translation() = initial_pose_estimate.pose.translation();
		VertexSE3DeltaEdge* deltaedge = new VertexSE3DeltaEdge(isopose);
		deltaedge->setId(i++);
		deltaedge->setVertex(0,pose);

		deltaedge->setInformation(information);
		optimizer.addEdge(deltaedge);
	}


	optimizer.initializeOptimization();
	optimizer.optimize(options_.g2o_solver_options().max_num_iterations());
//	LOG(INFO)<<optimizer.chi2();
	*cost =  0.5 * optimizer.chi2(); //0.5*error*\Omega*error
   *pose_estimate = ivcommon::transform::Rigid3d(pose->estimate().translation(),Eigen::Quaterniond(pose->estimate().rotation()));
//   LOG(INFO)<<*pose_estimate;
//   LOG(INFO)<<pose_estimate->DebugString();

}

}  // namespace scan_matching
}  // namespace mapping_3d
