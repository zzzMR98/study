
#ifndef CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_


#include "ivcommon/common/lua_parameter_dictionary.h"
#include "iv_slam_mapping/mapping_3d/hybrid_grid.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/sensor/proto/adaptive_voxel_filter_options.pb.h"

namespace iv_slam_mapping {
namespace sensor {
///
/// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
/// a voxel edge.
  ///
PointCloud VoxelFiltered(const PointCloud& point_cloud, float size);
///
/// Voxel filter for point clouds. For each voxel, the assembled point cloud
/// contains the first point that fell into it from any of the inserted point
/// clouds.
///
class VoxelFilter {
 public:
   ///
  /// 'size' is the length of a voxel edge.
   ///
  explicit VoxelFilter(float size);

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;
///
  /// Inserts a point cloud into the voxel filter.
  ///
  void InsertPointCloud(const PointCloud& point_cloud);
///
  /// Returns the filtered point cloud representing the occupied voxels.
  ///
  const PointCloud& point_cloud() const;

 private:
  mapping_3d::HybridGridBase<uint8> voxels_;
  PointCloud point_cloud_;
};


}  /// 命名空间 sensor
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
