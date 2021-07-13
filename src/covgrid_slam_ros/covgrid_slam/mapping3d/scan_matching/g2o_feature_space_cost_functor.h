/*!
* \file g2o_feature_space_cost_functor.h
* \brief 符合g2o要求的优化边，进行点云的匹配，位姿估计
*
* 符合g2o要求的优化边，进行点云的匹配，位姿估计，损失函数的计算
*
* \author jikaijin jikaijin94@gmail.com
* \version v1.2.1
* \date 2018/11/14
*/

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_G2O_MATCHING_FEATURE_SPACE_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_G2O_MATCHING_FEATURE_SPACE_COST_FUNCTOR_H_

#include "Eigen/Core"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/mapping3d/scan_matching/feature_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"
//#include "g2o/core/base_binary_edge.h"

namespace mapping3d {
namespace scan_matching {

namespace types_icp {
  void init();
}

/// \brief 符合g2o要求的优化边，进行点云的匹配，位姿估计，损失函数及雅克比矩阵的计算
///	符合g2o要求的优化边，进行点云的匹配，位姿估计，损失函数及雅克比矩阵的计算
class FeatureSpaceEdge: public g2o::BaseUnaryEdge<1,double,g2o::VertexSE3>{
 public:


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  // Creates an FeatureSpaceEdge using the specified grid, 'rotation' to
  // add to all poses, and point cloud.
  FeatureSpaceEdge(const sensor::Point& point,
                           const FeatureHybridGrid& hybrid_grid,
			   std::pair<::ivcommon::transform::posestamped,ivcommon::transform::posestamped> posetimepair);


     // I/O functions
     bool read(std::istream& is){}
     bool write(std::ostream& os) const{}

     void computeError() override
     {
    	 if(update_)
    	 {
//    		 LOG(INFO)<<id();
    		 return;
    	 }

       // from <ViewPoint> to <Point>
       const g2o::VertexSE3 *vp = static_cast<const g2o::VertexSE3*>(_vertices[0]);

       // get vp1 point into vp0 frame
       // could be more efficient if we computed this transform just once

       ivcommon::transform::Rigid3d transform(vp->estimate().translation(),Eigen::Quaterniond(vp->estimate().rotation()));

//	    Eigen::Quaterniond q = posetimepair_.first.pose.rotation().slerp(rate_,transform.rotation());
//	    Eigen::Vector3d t = posetimepair_.first.pose.translation()*(1.-rate_)
//		+transform.translation()*rate_;
	     nowtransform_ = transform;//transform::Rigid3d(t,q);

	  	worldpoint_ =
	  			nowtransform_ * point_.cast<double>();

//	  	Eigen::Vector3d val = feature_grid_->GetMinDistancesquare(worldpoint_[0],worldpoint_[1],worldpoint_[2],information_);
//
//	  	this->setInformation(information_);


//	  _error << scaling_factor_ * val[0], scaling_factor_ * val[1],scaling_factor_ * val[2];

	  _error << feature_grid_->GetMinDistancesquare(worldpoint_[0],worldpoint_[1],worldpoint_[2],derivative_);


	  update_ = true;

     }

     // try analytic jacobians

     void linearizeOplus() override;


 private:
  const sensor::Point point_; //!< 点
  const std::shared_ptr<FeatureGrid> feature_grid_;//!< 特征栅格地图
  std::pair<::ivcommon::transform::posestamped,ivcommon::transform::posestamped> posetimepair_;//!<前后帧位姿
  Eigen::Vector3d worldpoint_;//!<世界坐标系的点
//  Eigen::Matrix<double,3,3> information_;
  Eigen::Matrix<double,1,3> derivative_;//!<导数
//  int num = 0;
  double rate_;//!<根据时间计算的差值比率
  ivcommon::transform::Rigid3d nowtransform_;//!<当前估计位姿
  bool update_;//!< 更新标志
};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_G2O_MATCHING_FEATURE_SPACE_COST_FUNCTOR_H_
