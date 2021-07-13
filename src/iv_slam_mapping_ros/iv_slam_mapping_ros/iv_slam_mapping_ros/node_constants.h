
#ifndef CARTOGRAPHER_ROS_NODE_CONSTANTS_H_
#define CARTOGRAPHER_ROS_NODE_CONSTANTS_H_

#include <string>
#include <vector>
///
///此头文件包括Ｎｏｄｅ节点需要的各话题名
///
namespace iv_slam_mapping_ros {
constexpr char kPointCloud2Topic[] = "points2";///三维点云
constexpr char kDynamicObjectTopic[] = "MovingTarget";///动态障碍物
constexpr char kLocationModuleTopic[] = "sensor_fusion_output";///融合定位
constexpr char kOdometryTopic[] = "lidar_odometry_to_earth";///雷达里程计
constexpr char kLidarOdometryTopic[] = "lidar_odometry_for_mapping";///有先验地图的雷达里程计
}  /// 命名空间 iv_slam_mapping_ros

#endif  // CARTOGRAPHER_ROS_NODE_CONSTANTS_H_
