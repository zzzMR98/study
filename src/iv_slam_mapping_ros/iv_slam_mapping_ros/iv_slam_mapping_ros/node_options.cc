
#include "iv_slam_mapping_ros/node_options.h"

#include <vector>

#include "ivcommon/common/configuration_file_resolver.h"
#include "iv_slam_mapping/mapping/map_builder.h"
#include "glog/logging.h"

namespace iv_slam_mapping_ros {
///
///读取配置文件信息，创建Ｎｏｄｅ节点
///
NodeOptions CreateNodeOptions( ::ivcommon::LuaParameterDictionary* const lua_parameter_dictionary) {
  NodeOptions options;
  options.map_builder_options = ::iv_slam_mapping::mapping::CreateMapBuilderOptions(lua_parameter_dictionary->GetDictionary("map_builder").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec = lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.node_information_write = lua_parameter_dictionary->GetBool("node_information_write");
  return options;
}
///
  ///加载配置文件参数到　node_options, trajectory_options；
  ///
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions( const string& configuration_directory, const string& configuration_basename) {
  auto file_resolver = ::ivcommon::make_unique<::ivcommon::ConfigurationFileResolver>( std::vector<string>{configuration_directory});///获取文件路径
  const string code = file_resolver->GetFileContentOrDie(configuration_basename);///在该路径下寻找目标配置文件，找不到就杀死节点
  ::ivcommon::LuaParameterDictionary lua_parameter_dictionary( code, std::move(file_resolver));///创建配置文件参数字典
  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary), CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace iv_slam_mapping_ros
