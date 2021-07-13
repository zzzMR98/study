/*!
* \file g2o_feature_space_cost_functor.h
* \brief 符合g2o要求的优化边，进行点云的匹配，位姿估计
*
* 符合g2o要求的优化边，进行点云的匹配，位姿估计,雅克比矩阵的计算。
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/


#include "covgrid_slam/mapping3d/scan_matching/g2o_feature_space_cost_functor.h"

#include "Eigen/Core"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"

#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/mapping3d/scan_matching/feature_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"
namespace mapping3d {
/// \brief 扫描匹配
///
namespace scan_matching {


// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
FeatureSpaceEdge::FeatureSpaceEdge(const sensor::Point& point,
                         const FeatureHybridGrid& hybrid_grid,
			   std::pair<::ivcommon::transform::posestamped,ivcommon::transform::posestamped> posetimepair)
    : point_(point),
      feature_grid_(new FeatureGrid(hybrid_grid)),
		posetimepair_(posetimepair),
		update_(false){
//	  information_ = Eigen::Matrix<double,3,3>::Zero();
	  derivative_ = Eigen::Matrix<double,1,3>::Zero();
	  rate_ = 1. - (0.1-(point_.ringandtime - int(point_.ringandtime)))
		/ ::ivcommon::ToSeconds(posetimepair_.second.time - posetimepair_.first.time);
}

void FeatureSpaceEdge::linearizeOplus()
{
	g2o::VertexSE3* vp0 = static_cast<g2o::VertexSE3*>(_vertices[0]);
//	LOG(INFO)<<derivative_;
    // this could be more efficient

  {
	g2o::Matrix3D R1 = nowtransform_.rotation().matrix();
	g2o::Matrix3D P_M ;
	P_M<<0,point_[2],-point_[1],
	-point_[2],0,point_[0],
	point_[1],-point_[0],0;

//    	  P_M *= rate_;
//	  _jacobianOplusXi.block<3,3>(0,0) = R1;
////	  _jacobianOplusXi.block<3,3>(0,3) = R1*P_M;
//	  _jacobianOplusXi.block<3,1>(0,3) = R1*dRidx.transpose()*point_.cast<double>();
//	  _jacobianOplusXi.block<3,1>(0,4) = R1*dRidy.transpose()*point_.cast<double>();
//	  _jacobianOplusXi.block<3,1>(0,5) = R1*dRidz.transpose()*point_.cast<double>();

    _jacobianOplusXi.block<1,3>(0,0) = derivative_ * R1;
    _jacobianOplusXi.block<1,3>(0,3) = 2*derivative_ * R1 * P_M;
	update_ = false;
//    	  LOG(INFO)<<id()<<" "<<++num<<" "<<derivative_;
//    	  LOG(INFO)<<_jacobianOplusXi;

  }
}

}  // namespace scan_matching
}  // namespace mapping_3d

