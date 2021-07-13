#ifndef CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_H_
#define CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_H_

#include "ivcommon/common/lua_parameter_dictionary.h"
#include "iv_slam_mapping/mapping_3d/proto/local_trajectory_builder_options.pb.h"

namespace iv_slam_mapping {
namespace mapping_3d {
///
  ///从ｌｕａ配置文件中创建TrajectoryBuilderOptions
  ///
proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(::ivcommon::LuaParameterDictionary* parameter_dictionary);
}  /// 命名空间 mapping_3d
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_H_
