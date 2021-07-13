
#ifndef CARTOGRAPHER_SENSOR_ODOMETRY_DATA_H_
#define CARTOGRAPHER_SENSOR_ODOMETRY_DATA_H_
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "ivcommon/common/time.h"
#include "iv_slam_mapping/sensor/proto/sensor.pb.h"
#include "ivcommon/transform/rigid_transform.h"

namespace iv_slam_mapping {
namespace sensor {
///
///具有先验地图功能的激光雷达里程计数据
///
struct LidarOdometryData{
  ::ivcommon::Time time;///时间戳
  ::ivcommon::transform::Rigid3d pose;///激光雷达里程计局部位姿
  Eigen::Vector3d GPS;///x is the longitude .y is latitude .z is altitude.
  std::vector<int> indexs;///地图索引
 short int mode;///０为正在创建地图，１为一个地图创建完成，２为先验地图模式
};
struct OdometryData {
  ::ivcommon::Time time;
  ::ivcommon::transform::Rigid3d pose;
};
}  /// 命名空间 sensor
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_SENSOR_ODOMETRY_DATA_H_
