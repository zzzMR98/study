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

#ifndef CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_

#include "ivcommon/common/lua_parameter_dictionary.h"
#include "covgrid_slam/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "covgrid_slam/mapping3d/hybrid_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"

namespace sensor {

// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
// a voxel edge.
PointCloud VoxelFiltered(const PointCloud& point_cloud, float size);

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  explicit VoxelFilter(float size);

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Inserts a point cloud into the voxel filter.
  void InsertPointCloud(const PointCloud& point_cloud);

  // Returns the filtered point cloud representing the occupied voxels.
  const PointCloud& point_cloud() const;

 private:
  mapping3d::HybridGridBase<uint8> voxels_;
  PointCloud point_cloud_;
};
class MaxIntensityVoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  explicit MaxIntensityVoxelFilter(float size);

  MaxIntensityVoxelFilter(const MaxIntensityVoxelFilter&) = delete;
  MaxIntensityVoxelFilter& operator=(const MaxIntensityVoxelFilter&) = delete;

  // Inserts a point cloud into the voxel filter.
  void InsertPointCloud(const PointCloud& point_cloud);

  // Returns the filtered point cloud representing the occupied voxels.
  const PointCloud& point_cloud() const;

 private:
  mapping3d::HybridGridBase<std::pair<int,float>> max_intensity_voxels_;
  PointCloud point_cloud_;
};

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    ::ivcommon::LuaParameterDictionary* const parameter_dictionary);

class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud Filter(const PointCloud& point_cloud) const;
  PointCloud MaxIntensityFilter(const PointCloud& point_cloud) const;
 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor

#endif  // CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
