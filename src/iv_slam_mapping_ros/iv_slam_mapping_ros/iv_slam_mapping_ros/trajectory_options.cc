
#include "iv_slam_mapping_ros/trajectory_options.h"

 #include "iv_slam_mapping/mapping/trajectory_builder.h"
#include "glog/logging.h"

namespace iv_slam_mapping_ros {
///
///从配置文件中假造ＴｒａｊｅｃｔｏｒｙＯｐｔｉｏｎｓ
///
TrajectoryOptions CreateTrajectoryOptions(::ivcommon::LuaParameterDictionary* const lua_parameter_dictionary) {
  TrajectoryOptions options;
  options.trajectory_builder_options =::iv_slam_mapping::mapping::CreateTrajectoryBuilderOptions(lua_parameter_dictionary->GetDictionary("trajectory_builder").get());///轨迹创建配置选项
  options.tracking_frame = lua_parameter_dictionary->GetString("tracking_frame");///跟踪坐标系，一般为车体坐标系
  options.published_frame =lua_parameter_dictionary->GetString("published_frame");///消息发布坐标系，一般为车体坐标系
  options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");///里程计坐标系
  options.provide_odom_frame = lua_parameter_dictionary->GetBool("provide_odom_frame");///是否提供里程计 
  options.wiping_movingtaget = lua_parameter_dictionary->GetBool("wiping_movingtaget");///是否剔除动态障碍物
  options.use_compressed_pointcloud = lua_parameter_dictionary->GetBool("use_compressed_pointcloud");///是否接收压缩点云
  options.use_lidar_odometry = lua_parameter_dictionary->GetBool("use_lidar_odometry");///是否使用具有在线和先验模式信息的激光雷达里程计
  options.use_location_module = lua_parameter_dictionary->GetBool("use_location_module");///是否使用融合定位模块信息
 
  return options;
}
}  // namespace iv_slam_mapping_ros
