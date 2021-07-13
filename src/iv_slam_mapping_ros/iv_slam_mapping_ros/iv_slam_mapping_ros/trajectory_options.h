

#ifndef CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_
#define CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_
#include <string>
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/port.h"
#include "iv_slam_mapping/mapping/proto/trajectory_builder_options.pb.h"
#include "iv_slam_ros_msgs/TrajectoryOptions.h"

namespace iv_slam_mapping_ros {
///
  ///ＴｒａｊｅｃｔｏｒｙＯｐｔｉｏｎｓ结构体
  ///
struct TrajectoryOptions {
  ::iv_slam_mapping::mapping::proto::TrajectoryBuilderOptions trajectory_builder_options;///轨迹创建配置选项
  string tracking_frame;///跟踪坐标系，一般为车体坐标系
  string published_frame;///消息发布坐标系，一般为车体坐标系
  string odom_frame; ///里程计坐标系
  bool provide_odom_frame;///是否提供里程计
  bool use_lidar_odometry;///是否使用具有在线和先验模式信息的激光雷达里程计
  bool wiping_movingtaget;///是否剔除动态障碍物
  bool use_compressed_pointcloud;///是否接收压缩点云
  bool use_location_module;///是否使用融合定位模块信息
  
};

///
///从配置文件中假造ＴｒａｊｅｃｔｏｒｙＯｐｔｉｏｎｓ
///
TrajectoryOptions CreateTrajectoryOptions(
    ::ivcommon::LuaParameterDictionary* lua_parameter_dictionary);

}  /// 命名空间 iv_slam_mapping_ros

#endif  // CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_
