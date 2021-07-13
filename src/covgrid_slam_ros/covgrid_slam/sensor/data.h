/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include "ivcommon/common/time.h"
#include "ivcommon/transform/rigid_transform.h"
#include "covgrid_slam/sensor/point_cloud.h"
#include "covgrid_slam/sensor/range_data.h"

namespace sensor {

// This type is a logical union, i.e. only one type of sensor data is actually
// filled in. It is only used for time ordering sensor data before passing it
// on.
struct Data {
  enum class Type { kImu, kRangefinder, kOdometer };

  struct Imu {
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
  };

  struct Rangefinder {
    Eigen::Vector3d origin;
    PointCloud ranges;
  };

  Data(const ::ivcommon::Time time, const Imu& imu)
      : type(Type::kImu), time(time), imu(imu) {}

  Data(const ::ivcommon::Time time, const Rangefinder& rangefinder)
      : type(Type::kRangefinder), time(time), rangefinder(rangefinder) {}

  Data(const ::ivcommon::Time time, const ivcommon::transform::Rigid3d& odometer_pose)
      : type(Type::kOdometer), time(time), odometer_pose(odometer_pose) {}

  Type type;
  ::ivcommon::Time time;
  Imu imu;
  Rangefinder rangefinder;
  ivcommon::transform::Rigid3d odometer_pose;
};

}  // namespace sensor

#endif  // CARTOGRAPHER_MAPPING_DATA_H_
