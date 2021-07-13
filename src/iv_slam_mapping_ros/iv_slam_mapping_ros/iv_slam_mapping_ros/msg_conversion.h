
#ifndef CARTOGRAPHER_ROS_MSG_CONVERSION_H_
#define CARTOGRAPHER_ROS_MSG_CONVERSION_H_
#include "ivcommon/common/port.h"
#include "ivcommon/transform/rigid_transform.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
//#include <pcl/ros/conversions.h>
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace iv_slam_mapping_ros {
::ivcommon::transform::Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform);///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．

::ivcommon::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．

}  // namespace iv_slam_mapping_ros

#endif  // CARTOGRAPHER_ROS_MSG_CONVERSION_H_
