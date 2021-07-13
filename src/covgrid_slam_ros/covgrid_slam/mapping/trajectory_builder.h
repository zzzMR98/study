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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_

#include <functional>
#include <memory>
#include <string>

#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/make_unique.h"
#include "ivcommon/common/port.h"
#include "ivcommon/common/time.h"

#include "covgrid_slam/mapping/submaps.h"
#include "covgrid_slam/sensor/data.h"
#include "covgrid_slam/sensor/point_cloud.h"
#include "covgrid_slam/sensor/range_data.h"

namespace mapping {


// This interface is used for both 2D and 3D SLAM.
class TrajectoryBuilder {
 public:
  // Represents a newly computed pose. 'pose' is the end-user visualization of
  // orientation and 'point_cloud' is the point cloud, in the local map frame.
  struct PoseEstimate :public ivcommon::transform::posestamped{
    PoseEstimate() = default;
    PoseEstimate(::ivcommon::Time time, const ivcommon::transform::Rigid3d& pose,
                 const sensor::PointCloud& point_cloud)
        : point_cloud(point_cloud) {
      ivcommon::transform::posestamped::time = (time);
      ivcommon::transform::posestamped::pose = (pose);
    }

    sensor::PointCloud point_cloud;
  };

  TrajectoryBuilder() {}
  virtual ~TrajectoryBuilder() {}

  TrajectoryBuilder(const TrajectoryBuilder&) = delete;
  TrajectoryBuilder& operator=(const TrajectoryBuilder&) = delete;

  virtual const PoseEstimate& pose_estimate() const = 0;

  virtual void AddSensorData(const string& sensor_id,
                             std::unique_ptr<sensor::Data> data) = 0;

  void AddRangefinderData(const string& sensor_id, ::ivcommon::Time time,
                          const Eigen::Vector3d& origin,
                          const sensor::PointCloud& ranges) {
    AddSensorData(sensor_id,
                  ::ivcommon::make_unique<sensor::Data>(
                      time, sensor::Data::Rangefinder{origin, ranges}));
  }

  void AddImuData(const string& sensor_id, ::ivcommon::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) {
    AddSensorData(sensor_id, ::ivcommon::make_unique<sensor::Data>(
                                 time, sensor::Data::Imu{linear_acceleration,
                                                         angular_velocity}));
  }

  void AddOdometerData(const string& sensor_id, ::ivcommon::Time time,
                       const ivcommon::transform::Rigid3d& odometer_pose) {
    AddSensorData(sensor_id,
                  ::ivcommon::make_unique<sensor::Data>(time, odometer_pose));
  }
};

}  // namespace mapping

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
