///
///这个类作为CollatedTrajectoryBuilder的基类，
///主要用于将各个传感器数据传送到sensor_collator_进行数据调度
///
///
#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
#include <functional>
#include <memory>
#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/make_unique.h"
#include "ivcommon/common/port.h"
#include "ivcommon/common/time.h"
#include "iv_slam_mapping/mapping/proto/trajectory_builder_options.pb.h"
#include "iv_slam_mapping/mapping/submaps.h"
#include "iv_slam_mapping/sensor/data.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/sensor/range_data.h"

namespace iv_slam_mapping {
namespace mapping {
///
  ///从ｌｕａ配置文件中创建TrajectoryBuilderOptions
  ///
proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    ::ivcommon::LuaParameterDictionary* const parameter_dictionary);

///
///这个类作为CollatedTrajectoryBuilder的基类，
///
class TrajectoryBuilder {
 public:
//   struct PoseEstimate {
//     PoseEstimate() = default;
//     PoseEstimate(::ivcommon::Time time, const ivcommon::transform::Rigid3d& pose, const sensor::PointCloud& point_cloud)
//         : time(time), pose(pose), point_cloud(point_cloud) {}
// 
//     ::ivcommon::Time time = ::ivcommon::Time::min();
//     ivcommon::transform::Rigid3d pose = ivcommon::transform::Rigid3d::Identity();
//     sensor::PointCloud point_cloud;
//   };

  TrajectoryBuilder() {}
  virtual ~TrajectoryBuilder() {}
  TrajectoryBuilder(const TrajectoryBuilder&) = delete;
  TrajectoryBuilder& operator=(const TrajectoryBuilder&) = delete;

//   virtual const PoseEstimate& pose_estimate() const = 0;

  virtual void AddSensorData(const string& sensor_id, std::unique_ptr<sensor::Data> data) = 0;///在CollatedTrajectoryBuilder被重写

  void AddRangefinderData(const string& sensor_id, ::ivcommon::Time time, const Eigen::Vector3f& origin, const sensor::PointCloud& ranges) {
    AddSensorData(sensor_id, ::ivcommon::make_unique<sensor::Data>( time, sensor::Data::Rangefinder{origin, ranges}));
  }

  void AddImuData(const string& sensor_id, ::ivcommon::Time time, const Eigen::Vector3d& linear_acceleration,const Eigen::Vector3d& angular_velocity) {
    AddSensorData(sensor_id, ::ivcommon::make_unique<sensor::Data>( time, sensor::Data::Imu{linear_acceleration, angular_velocity}));
  }

  void AddOdometerData(const string& sensor_id, ::ivcommon::Time time,const ::ivcommon::transform::Rigid3d& odometer_pose) {
    AddSensorData(sensor_id, ::ivcommon::make_unique<sensor::Data>(time, odometer_pose));
  }
    void AddOdometerWithGPSData(const string& sensor_id, ::ivcommon::Time time, const ::ivcommon::transform::Rigid3d& odometer_pose, const Eigen::Vector3d& GPS) { 
    AddSensorData(sensor_id, ::ivcommon::make_unique<sensor::Data>(time, sensor::Data::Odometry_Pose_withGPS{odometer_pose,GPS}));
  }
     void AddLidarOdometerData(const string& sensor_id, ::ivcommon::Time time, const ::ivcommon::transform::Rigid3d& odometer_pose, const Eigen::Vector3d& GPS,std::vector<int>& indexs,short int & mode) { 
    AddSensorData(sensor_id, ::ivcommon::make_unique<sensor::Data>(time, sensor::Data::LidarOdometry{odometer_pose,GPS,indexs,mode}));
  }
  void AddLocationModuleData(const string& sensor_id, ::ivcommon::Time time,const ::ivcommon::transform::Rigid3d& location_module_pose){
     AddSensorData(sensor_id,::ivcommon::make_unique<sensor::Data>(time, sensor::Data::OdometrydData{location_module_pose}));
  }
     void AddDynamicObjectData(const string& sensor_id, ::ivcommon::Time time, const sensor::Data::DynamicObject& dynamic_object) { 
    AddSensorData(sensor_id, ::ivcommon::make_unique<sensor::Data>(time, sensor::Data::DynamicObject{dynamic_object}));
  }
};

}  /// 命名空间 mapping
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
