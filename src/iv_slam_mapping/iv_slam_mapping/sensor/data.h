#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "ivcommon/common/time.h"
#include "ivcommon/transform/rigid_transform.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/sensor/range_data.h"
namespace iv_slam_mapping {
namespace sensor {
///
/// This type is a logical union, i.e. only one type of sensor data is actually
/// filled in. It is only used for time ordering sensor data before passing it
/// on.
  ///
struct Data {
  enum class Type { kImu, kRangefinder, kOdometer, kOdometerWithGPS, KLidarOdometry, KDynamicObject , KLocationModule};///用ｅｎｕｍ管理数据类型

  struct Imu {
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
  };///ＩＭＵ数据类型

  struct Rangefinder {
    Eigen::Vector3f origin;
    PointCloud ranges;
  };///点云数据类型
  struct OdometrydData{
    ::ivcommon::transform::Rigid3d pose;
  };///普通传统里程计数据类型
  
   struct Odometry_Pose_withGPS {
    ::ivcommon::transform::Rigid3d odometer_pose;
    Eigen::Vector3d GPS;    
  };///带有ｇｐｓ的里程计数据类型
    struct LidarOdometry {
    ::ivcommon::transform::Rigid3d odometer_pose;
    Eigen::Vector3d GPS;
std::vector<int> indexs;
short int mode;
  };///雷达里程计数据类型
  
 
  
  struct DynamicObject{
     struct MovingTarget{    
    struct HistoryTraj{
      std::vector<Eigen::Vector3d> points;
    };
    int history_num;
    bool is_updated;
    std::vector<HistoryTraj> history_traj;    
  };
  
    int target_num;
    ::ivcommon::Time time;
    std::vector<MovingTarget> moving_target;    
  };///动态障碍物数据类型

  Data(const ::ivcommon::Time time, const Imu& imu)
      : type(Type::kImu), time(time), imu(imu) {}///重载构造函数，根据数据类型的不同，调用不同的重载函数

  Data(const ::ivcommon::Time time, const Rangefinder& rangefinder)
      : type(Type::kRangefinder), time(time), rangefinder(rangefinder) {}///重载构造函数，根据数据类型的不同，调用不同的重载函数

  Data(const ::ivcommon::Time time, const ::ivcommon::transform::Rigid3d& odometer_pose)
      : type(Type::kOdometer), time(time), odometer_pose(odometer_pose) {}///重载构造函数，根据数据类型的不同，调用不同的重载函数
      
  Data(const ::ivcommon::Time time, const Odometry_Pose_withGPS& odometry_pose_withGPS)
      : type(Type::kOdometerWithGPS), time(time), odometry_pose_withGPS(odometry_pose_withGPS) {
      }///重载构造函数，根据数据类型的不同，调用不同的重载函数
  Data(const ::ivcommon::Time time, const LidarOdometry& lidar_odometry)
      : type(Type::KLidarOdometry), time(time), lidar_odometry(lidar_odometry) {}///重载构造函数，根据数据类型的不同，调用不同的重载函数
  Data(const ::ivcommon::Time time, const DynamicObject& dynamic_object)
      : type(Type::KDynamicObject), time(time), dynamic_object(dynamic_object) {}///重载构造函数，根据数据类型的不同，调用不同的重载函数
  Data(const ::ivcommon::Time time, const OdometrydData& location_module)
      : type(Type::KLocationModule), time(time), location_module(location_module) {} ///重载构造函数，根据数据类型的不同，调用不同的重载函数

  Type type;
  ::ivcommon::Time time;
  Imu imu;///ＩＭＵ数据，现不用
  Rangefinder rangefinder;///点云数据
  ::ivcommon::transform::Rigid3d odometer_pose;///普通传统里程计数据，现不用
  Odometry_Pose_withGPS odometry_pose_withGPS;///带有全局位姿的激光雷达里程计数据，现不用
  LidarOdometry lidar_odometry;///激光雷达里程计数据
  DynamicObject dynamic_object;///动态障碍物数据
  OdometrydData location_module;///融合定位模块数据
};

}  /// 命名空间 sensor
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
