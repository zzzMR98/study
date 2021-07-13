/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_IMU_DATA_H_
#define CARTOGRAPHER_SENSOR_IMU_DATA_H_

#include "Eigen/Core"
#include "ivcommon/common/time.h"

namespace sensor {

struct ImuData {
  ::ivcommon::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
  double heading;
};

enum class LidarGpsStatus
{
  invalid = 0, warning = 1, valid = 2
};

struct GpsData {
  ::ivcommon::Time time  =  ::ivcommon::Time::min();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double latitude = 0;
  double longitude = 0;
  double heading = 0;  //角度
};

struct GpsInsData {
  ::ivcommon::Time time  =  ::ivcommon::Time::min();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double latitude = 0;
  double longitude = 0;
  double altitude = 0;
  double heading = 0;  //角度
  double roll = 0;  //角度
  double pitch = 0;  //角度
};

struct InsVelocityData {
  ::ivcommon::Time time  =  ::ivcommon::Time::min();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct EcuData{
	::ivcommon::Time time  =  ::ivcommon::Time::min();
	unsigned char f_shift;
	unsigned char f_shift1;
	double fDeForwardVel;
	double fFLRWheelAverAngle;
	double fForwardVel;

	double petral_pressure;
	double pressure_back;
	double FrontLeftWheelSpeed;
	double FrontRightWheelSpeed;
	double BackLeftWheelSpeed;//1
	double BackRightWheelSpeed;
};

}  // namespace sensor

#endif  // CARTOGRAPHER_SENSOR_IMU_DATA_H_
