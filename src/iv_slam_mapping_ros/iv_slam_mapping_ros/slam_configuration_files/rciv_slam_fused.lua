include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "global_earth_frame",
  tracking_frame = "vehicle_frame",
  published_frame = "vehicle_frame", 
  odom_frame = "vehicle_lidar_odometry_frame",
  provide_odom_frame = true,
  use_lidar_odometry = true,
  use_location_module = false,
  use_compressed_pointcloud = false,
  wiping_movingtaget = false, --是否接受动态障碍物数据并在三维地图中滤除动态障碍物
  lookup_transform_timeout_sec = 0.2,
  node_information_write = false,  
}

TRAJECTORY_BUILDER_3D.use_location_module = true
TRAJECTORY_BUILDER_3D.range_data_write = false --是否存储点云信息用于全局位姿优化模块
TRAJECTORY_BUILDER_3D.serve_global_optimization = false--是否向全局位姿优化模块发送地图创建结果
TRAJECTORY_BUILDER_3D.offline_map_invoke = false
TRAJECTORY_BUILDER_3D.offline_map_file_name_time = "20180613_184822"
TRAJECTORY_BUILDER_3D.submaps.twid_submap_write = false
TRAJECTORY_BUILDER_3D.submaps.enable_traversablearea_extraction = true--是否进行准可通行区域提取
TRAJECTORY_BUILDER_3D.submaps.trid_submap_write = false--是否存储三维子地图用于全局位姿优化模块
TRAJECTORY_BUILDER_3D.submaps.visualization_trid_submap_write = false
TRAJECTORY_BUILDER_3D.submaps.twid_submap_display = true
TRAJECTORY_BUILDER_3D.submaps.trid_submap_display = false
TRAJECTORY_BUILDER_3D.submaps.kxrayobstructedcellprobabilitylimit =0.650-- 0.899--//0-1
TRAJECTORY_BUILDER_3D.submaps.rough_intensity = 5--grid
TRAJECTORY_BUILDER_3D.submaps.kminzdifference = 0.5/0.2--voxel
TRAJECTORY_BUILDER_3D.submaps.kminzdifference_beyond =  0.8/0.2 --voxel
TRAJECTORY_BUILDER_3D.submaps.zdifference_change_thresh = 40/0.2--voxel
TRAJECTORY_BUILDER_3D.submaps.obstacle_emptythresh = 0.5--0-1
TRAJECTORY_BUILDER_3D.submaps.extension_index = 20/0.2 --grid
MAP_BUILDER.use_trajectory_builder_3d = true

return options
