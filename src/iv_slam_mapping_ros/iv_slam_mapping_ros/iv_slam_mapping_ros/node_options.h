#ifndef CARTOGRAPHER_ROS_NODE_OPTIONS_H_
#define CARTOGRAPHER_ROS_NODE_OPTIONS_H_
#include <string>
#include <tuple>
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/port.h"
#include "iv_slam_mapping/mapping/proto/map_builder_options.pb.h"
#include "iv_slam_mapping_ros/trajectory_options.h"

namespace iv_slam_mapping_ros {
///
  ///关于Ｎｏｄｅ的一些配置选项
  ///
struct NodeOptions {
  ::iv_slam_mapping::mapping::proto::MapBuilderOptions map_builder_options;
  string map_frame;///全局坐标系
  double lookup_transform_timeout_sec;/// 寻找坐标转化关系的等待时间
  bool node_information_write; ///是否保存当前帧点云数据
};

///
///读取配置文件信息，创建Ｎｏｄｅ节点
///
NodeOptions CreateNodeOptions(::ivcommon::LuaParameterDictionary* lua_parameter_dictionary);
///
  ///加载配置文件参数到　node_options, trajectory_options；
  ///
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions( const string& configuration_directory,const string& configuration_basename);

}  // namespace iv_slam_ros

#endif  // CARTOGRAPHER_ROS_NODE_OPTIONS_H_
