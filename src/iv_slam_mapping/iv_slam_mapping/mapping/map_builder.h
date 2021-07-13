#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "Eigen/Geometry"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/port.h"
#include "iv_slam_mapping/mapping/proto/map_builder_options.pb.h"
#include "iv_slam_mapping/mapping/proto/trajectory_builder_options.pb.h"
#include "iv_slam_mapping/mapping/submaps.h"
#include "iv_slam_mapping/mapping/trajectory_builder.h"
#include "iv_slam_mapping/mapping/collated_trajectory_builder.h"
#include "iv_slam_mapping/sensor/collator.h"
#include "ros/ros.h"

namespace iv_slam_mapping {
namespace mapping {
///
  ///从ｌｕａ中创建MapBuilderOptions
  ///
proto::MapBuilderOptions CreateMapBuilderOptions(::ivcommon::LuaParameterDictionary* const parameter_dictionary);

///
///地图创建者对象，主要用于管理路线，
///本文中只使用一个路线
///
class MapBuilder {
 public:
  MapBuilder(const proto::MapBuilderOptions& options, const ::ros::NodeHandle map_builder_nh,bool wiping_dynamic_object);
  ~MapBuilder();
  MapBuilder(const MapBuilder&) = delete;///禁止复制对象
  MapBuilder& operator=(const MapBuilder&) = delete;///禁止复制对象

  ///
  ///创建一个新的轨迹，并返回轨迹ｉｄ;trajectory_id
  ///
  int AddTrajectoryBuilder( const std::unordered_set<string>& expected_sensor_ids,
      const proto::TrajectoryBuilderOptions& trajectory_options);
///
  /// Returns the TrajectoryBuilder corresponding to the specified
  /// 'trajectory_id'.
  ///
  mapping::TrajectoryBuilder* GetTrajectoryBuilder(int trajectory_id) const;
///
  /// Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  /// i.e. no further sensor data is expected.
  ///
  void FinishTrajectory(int trajectory_id);



std::vector<std::unique_ptr<mapping::CollatedTrajectoryBuilder>> trajectory_builders_;///路线创建者，在本文中只有一个
 private:
  const proto::MapBuilderOptions options_;
  sensor::Collator sensor_collator_;///传感数据整理器对象，用于整理各传感器数据用于调度
  ::ros::NodeHandle map_builder_nh_;///传入的ｎｏｄｅ_handle
  bool wiping_dynamic_object_;///是否滤除动态障碍物
};

}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
