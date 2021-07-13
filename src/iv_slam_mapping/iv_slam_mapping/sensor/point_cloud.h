
#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "iv_slam_mapping/sensor/proto/sensor.pb.h"
#include "ivcommon/transform/rigid_transform.h"
#include "glog/logging.h"

namespace iv_slam_mapping {
namespace sensor {

typedef std::vector<Eigen::Vector3f> PointCloud;

struct PointCloudWithIntensities {
  PointCloud points;
  std::vector<float> intensities;
};///带有强度信息的点云

///
///对点云进行坐标变换
///
PointCloud TransformPointCloud(const PointCloud& point_cloud, const ::ivcommon::transform::Rigid3f& transform);

}  // 命名空间 sensor
}  // 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
