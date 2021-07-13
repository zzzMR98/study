
#include "iv_slam_mapping/sensor/voxel_filter.h"

#include <cmath>
#include "ivcommon/common/math.h"
namespace iv_slam_mapping {
namespace sensor {
///
/// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
/// a voxel edge.
  ///
PointCloud VoxelFiltered(const PointCloud& point_cloud, const float size) {
  VoxelFilter voxel_filter(size);
  voxel_filter.InsertPointCloud(point_cloud);
  return voxel_filter.point_cloud();
}
 ///
  /// 'size' is the length of a voxel edge.
   ///
VoxelFilter::VoxelFilter(const float size) : voxels_(size) {}
///
  /// Inserts a point cloud into the voxel filter.
  ///
void VoxelFilter::InsertPointCloud(const PointCloud& point_cloud) {
  for (const Eigen::Vector3f& point : point_cloud) {
    auto* const value = voxels_.mutable_value(voxels_.GetCellIndex(point));
    if (*value == 0) {
      point_cloud_.push_back(point);
      *value = 1;
    }
  }
}
///
  /// Returns the filtered point cloud representing the occupied voxels.
  ///
const PointCloud& VoxelFilter::point_cloud() const { return point_cloud_; }

}  /// 命名空间 sensor
}  /// 命名空间 iv_slam_mapping
