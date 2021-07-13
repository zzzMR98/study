
#ifndef CARTOGRAPHER_SENSOR_IMU_DATA_H_
#define CARTOGRAPHER_SENSOR_IMU_DATA_H_

#include "Eigen/Core"
#include "ivcommon/common/time.h"
#include "iv_slam_mapping/sensor/proto/sensor.pb.h"

namespace iv_slam_mapping {
namespace sensor {
///
  ///IMU数据，现不用
  ///
struct ImuData {
  ::ivcommon::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};
}  // 命名空间 sensor
}  // 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_SENSOR_IMU_DATA_H_
