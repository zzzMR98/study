
#include "iv_slam_mapping_ros/msg_conversion.h"
#include "ivcommon/common/port.h"
#include "ivcommon/common/time.h"
#include "ivcommon/common/time_conversion.h"
#include "ivcommon/transform/proto/transform.pb.h"
#include "ivcommon/transform/transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace iv_slam_mapping_ros {
  
using ::ivcommon::transform::Rigid3d;
///
///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．
///
Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),ToEigen(transform.transform.rotation));
}
///
///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．
///
Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z}, ToEigen(pose.orientation));
}
///
///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．
///
Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}
///
///数据类型转换，从ｒｏｓ＿ｍｓｇ格式转换为程序中的格式．
///
Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

}  // namespace iv_slam_mapping_ros
