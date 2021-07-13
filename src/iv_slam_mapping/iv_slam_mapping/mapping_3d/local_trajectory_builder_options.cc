
#include "iv_slam_mapping/mapping_3d/local_trajectory_builder_options.h"
#include "iv_slam_mapping/mapping_3d/submaps.h"
#include "iv_slam_mapping/sensor/voxel_filter.h"
#include "glog/logging.h"

namespace iv_slam_mapping {
namespace mapping_3d {
///
  ///从ｌｕａ配置文件中创建TrajectoryBuilderOptions
  ///
proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    ::ivcommon::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));///点云距离滤波最小范围
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));///点云距离滤波最大范围
  options.set_voxel_filter_size( parameter_dictionary->GetDouble("voxel_filter_size"));///点云稀疏化体素阈值
  options.set_imu_gravity_time_constant( parameter_dictionary->GetDouble("imu_gravity_time_constant"));///重力常数
  options.set_range_data_write( parameter_dictionary->GetBool("range_data_write"));///是否保存点云数据
  options.set_offline_map_invoke( parameter_dictionary->GetBool("offline_map_invoke"));///是否使用离线地图
  options.set_offline_map_file_name_time(parameter_dictionary->GetString("offline_map_file_name_time"));///离线地图所在文件名
  options.set_serve_global_optimization( parameter_dictionary->GetBool("serve_global_optimization"));///是否为全局位姿优化模块发送位姿结果
  *options.mutable_submaps_options() = mapping_3d::CreateSubmapsOptions( parameter_dictionary->GetDictionary("submaps").get());///子地图配置选项
  options.set_use_location_module(parameter_dictionary->GetBool("use_location_module"));///是否使用融合定位模块信息
  return options;
}

}  /// 命名空间 mapping_3d
}  /// 命名空间 iv_slam_mapping
