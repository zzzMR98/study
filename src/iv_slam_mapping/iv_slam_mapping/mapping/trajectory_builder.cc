
#include "iv_slam_mapping/mapping/trajectory_builder.h"
#include "iv_slam_mapping/mapping_3d/local_trajectory_builder_options.h"

namespace iv_slam_mapping {
namespace mapping {
///
  ///从ｌｕａ配置文件中创建TrajectoryBuilderOptions
  ///
proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(::ivcommon::LuaParameterDictionary* const parameter_dictionary) {
  proto::TrajectoryBuilderOptions options;
  *options.mutable_trajectory_builder_3d_options() = mapping_3d::CreateLocalTrajectoryBuilderOptions( parameter_dictionary->GetDictionary("trajectory_builder_3d").get());
  return options;
}

}  // namespace mapping
}  // namespace iv_slam_mapping
