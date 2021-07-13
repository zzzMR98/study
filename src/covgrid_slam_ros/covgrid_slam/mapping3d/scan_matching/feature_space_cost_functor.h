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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FEATURE_SPACE_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FEATURE_SPACE_COST_FUNCTOR_H_

#include "Eigen/Core"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/mapping3d/scan_matching/feature_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"
#include "covgrid_slam/mapping3d/submaps.h"

namespace mapping3d {
namespace scan_matching {

// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
class FeatureSpaceCostFunctor {
 public:
  // Creates an FeatureSpaceCostFunctor using the specified grid, 'rotation' to
  // add to all poses, and point cloud.
  FeatureSpaceCostFunctor(const double scaling_factor,
                           const sensor::PointCloud& point_cloud,
                           const FeatureHybridGrid& hybrid_grid,
			   std::pair<::ivcommon::transform::posestamped,ivcommon::transform::posestamped> posetimepair,
			   bool use_Eigen = false,
			   std::shared_ptr<const Submap> submap = nullptr)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        feature_grid_(hybrid_grid),
	posetimepair_(posetimepair),
	use_Eigen_(use_Eigen),
	submap_(submap){}

  FeatureSpaceCostFunctor(const FeatureSpaceCostFunctor&) = delete;
  FeatureSpaceCostFunctor& operator=(const FeatureSpaceCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const translation, const T* const rotation,
                  T* const residual) const {

      const ivcommon::transform::Rigid3<T> transform(
              Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
              !use_Eigen_ ? Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                                   rotation[3])
                                   :
                           Eigen::Quaternion<T>(rotation[3], rotation[0], rotation[1],
                                   rotation[2]));

    return Evaluate(transform, residual);
  }

  template <typename T>
  bool Evaluate(const ivcommon::transform::Rigid3<T>& transform,
                T* const residual) const {



	  Eigen::Quaternion<T> q = transform.rotation();
	  Eigen::Matrix<T, 3, 1> t = transform.translation();
	  ivcommon::transform::Rigid3<T> nowtransform(t,q);

	  if(use_Eigen_)
		  nowtransform = submap_->local_pose().inverse().cast<T>() * nowtransform;

    for (size_t i = 0; i < point_cloud_.size(); ++i) {



//	if(point_cloud_[i].type == sensor::Point::Type::kFlat)
//	  {
//	    double rated = 1. - (0.1-(point_cloud_[i].ringandtime - int(point_cloud_[i].ringandtime)))
//		/ ::ivcommon::ToSeconds(posetimepair_.second.time - posetimepair_.first.time);
//
//	    T rate = T(rated);


	  const Eigen::Matrix<T, 3, 1> world =
	      nowtransform * point_cloud_[i].cast<T>();

//	  Eigen::Matrix<T,3,1> val = feature_grid_.GetMinDistanceMatrix(world[0],world[1],world[2]);
////	  LOG(INFO)<<val(0,0);
////	  LOG(INFO)<<val(1,0);
////	  LOG(INFO)<<val(2,0);
//
//      //      LOG(INFO)<<"val:"<<val;
//	  residual[3*i] = scaling_factor_ * val(0,0) ;
//	  residual[3*i+1] = scaling_factor_ * val(1,0) ;
//	  residual[3*i+2] = scaling_factor_ * val(2,0) ;
//      const Eigen::Matrix<T, 3, 1> world = point_cloud_[i].cast<T>();
	  T val = feature_grid_.GetMinDistance(world[0],world[1],world[2]);

	  residual[i] = scaling_factor_ * val;

//	  }
//	else
//	  residual[i] = T(scaling_factor_);

    }
    return true;
  }

 private:
  const double scaling_factor_;//!< 缩放尺度
  const sensor::PointCloud& point_cloud_;//!< 点云
  const FeatureGrid feature_grid_; //!< 特征栅格
  const std::pair<::ivcommon::transform::posestamped,ivcommon::transform::posestamped> posetimepair_;//!< 前后帧位姿
  bool use_Eigen_;
  std::shared_ptr<const Submap> submap_;

};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FEATURE_SPACE_COST_FUNCTOR_H_
