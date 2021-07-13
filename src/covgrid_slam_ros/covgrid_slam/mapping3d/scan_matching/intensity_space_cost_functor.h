#ifndef COVGRID_MAPPING_3D_SCAN_MATCHING_INTENSITY_SPACE_COST_FUNCTOR_H_
#define COVGRID_MAPPING_3D_SCAN_MATCHING_INTENSITY_SPACE_COST_FUNCTOR_H_

#include "Eigen/Core"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"

#include "covgrid_slam/mapping3d/ceres_pose.h"
#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/mapping3d/scan_matching/interpolated_intensity_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"

namespace mapping3d {
namespace scan_matching {

// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
class IntensitySpaceCostFunctor {
 public:
  // Creates an OccupiedSpaceCostFunctor using the specified grid, 'rotation' to
  // add to all poses, and point cloud.
	IntensitySpaceCostFunctor(const double scaling_factor,
                           const sensor::PointCloud& point_cloud,
                           const IntensityHybridGrid& hybrid_grid,
			   std::pair<::ivcommon::transform::posestamped,ivcommon::transform::posestamped> posetimepair)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
		interpolated_intensity_grid_(hybrid_grid),
	posetimepair_(posetimepair){}

	IntensitySpaceCostFunctor(const IntensitySpaceCostFunctor&) = delete;
	IntensitySpaceCostFunctor& operator=(const IntensitySpaceCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const translation, const T* const rotation,
                  T* const residual) const {
    const ivcommon::transform::Rigid3<T> transform(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
        Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                             rotation[3]));
    return Evaluate(transform, residual);
  }

  template <typename T>
  bool Evaluate(const ivcommon::transform::Rigid3<T>& transform,
                T* const residual) const {
    for (size_t i = 0; i < point_cloud_.size(); ++i) {

	double rated = 1. - (0.1-(point_cloud_[i].ringandtime - int(point_cloud_[i].ringandtime)))
	    / ::ivcommon::ToSeconds(posetimepair_.second.time - posetimepair_.first.time);

	T rate = T(rated);
	Eigen::Quaternion<T> q = posetimepair_.first.pose.rotation().cast<T>().slerp(rate,transform.rotation());
	Eigen::Matrix<T, 3, 1> t = posetimepair_.first.pose.translation().cast<T>()*(1.-rate)
	    +transform.translation()*rate;
	::ivcommon::transform::Rigid3<T> nowtransform(t,q);

      const Eigen::Matrix<T, 3, 1> world =
	  nowtransform * point_cloud_[i].cast<T>();

      const T value =
    		  interpolated_intensity_grid_.GetValue(world[0], world[1], world[2]);
//      LOG(INFO)<<"probability:"<<probability;
      residual[i] = scaling_factor_ * (T(point_cloud_[i].intensity) - value);
//      	  LOG(INFO)<<"val:"<<residual[i];
    }
    return true;
  }

 private:
  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const InterpolatedIntensityGrid interpolated_intensity_grid_;
  const std::pair<::ivcommon::transform::posestamped,ivcommon::transform::posestamped> posetimepair_;
};

}  // namespace scan_matching
}  // namespace mapping_3d

#endif  // COVGRID_MAPPING_3D_SCAN_MATCHING_INTENSITY_SPACE_COST_FUNCTOR_H_
