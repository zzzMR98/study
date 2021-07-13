#include "iv_slam_mapping/mapping/map_builder.h"
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>
#include "ivcommon/common/make_unique.h"
#include "ivcommon/transform/rigid_transform.h"
#include "ivcommon/transform/transform.h"
#include "iv_slam_mapping/mapping/collated_trajectory_builder.h"
#include "iv_slam_mapping/mapping_3d/global_trajectory_builder.h"
#include "iv_slam_mapping/sensor/range_data.h"
#include "iv_slam_mapping/sensor/voxel_filter.h"
#include "glog/logging.h"


namespace iv_slam_mapping {
namespace mapping {
///
  ///从ｌｕａ中创建MapBuilderOptions
  ///
proto::MapBuilderOptions CreateMapBuilderOptions(
    ::ivcommon::LuaParameterDictionary* const parameter_dictionary) {
  proto::MapBuilderOptions options;
  options.set_use_trajectory_builder_3d(
      parameter_dictionary->GetBool("use_trajectory_builder_3d"));
  return options;
}
///
///地图创建者对象，主要用于管理路线，
///本文中只使用一个路线
///
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options,const ::ros::NodeHandle map_builder_nh,bool wiping_dynamic_object)
    : options_(options),map_builder_nh_(map_builder_nh),wiping_dynamic_object_(wiping_dynamic_object),sensor_collator_(wiping_dynamic_object) {}
///
///析构函数
///
MapBuilder::~MapBuilder() {}
///
  ///创建一个新的轨迹，并返回轨迹ｉｄ;trajectory_id
  ///
int MapBuilder::AddTrajectoryBuilder( const std::unordered_set<string>& expected_sensor_ids,const proto::TrajectoryBuilderOptions& trajectory_options) {
  const int trajectory_id = trajectory_builders_.size(); 
    LOG(INFO)<<"options_.use_trajectory_builder_3d() is "<<options_.use_trajectory_builder_3d();
    trajectory_builders_.push_back( ::ivcommon::make_unique<CollatedTrajectoryBuilder>( &sensor_collator_, trajectory_id, expected_sensor_ids,
            ::ivcommon::make_unique<mapping_3d::GlobalTrajectoryBuilder>(trajectory_options.trajectory_builder_3d_options(),trajectory_id,map_builder_nh_)));
  return trajectory_id;
}
///
  /// Returns the TrajectoryBuilder corresponding to the specified
  /// 'trajectory_id'.
  ///
TrajectoryBuilder* MapBuilder::GetTrajectoryBuilder(
    const int trajectory_id) const {
  return trajectory_builders_.at(trajectory_id).get();
}
///
  /// Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  /// i.e. no further sensor data is expected.
  ///
void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_.FinishTrajectory(trajectory_id);
}


}  ///  命名空间　mapping
}  /// 命名空间 iv_slam_mapping
