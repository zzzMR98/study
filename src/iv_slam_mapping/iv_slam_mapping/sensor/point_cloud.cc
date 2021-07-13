
#include "iv_slam_mapping/sensor/point_cloud.h"

#include "iv_slam_mapping/sensor/proto/sensor.pb.h"
#include "ivcommon/transform/transform.h"

namespace iv_slam_mapping {
namespace sensor {
///
///对点云进行坐标变换
///
PointCloud TransformPointCloud(const PointCloud& point_cloud, const ::ivcommon::transform::Rigid3f& transform) {
  PointCloud result;
  result.reserve(point_cloud.size());
  for (const Eigen::Vector3f& point : point_cloud) {
    result.emplace_back(transform * point);
  }
  return result;
}

}  // 命名空间  sensor
}  // 命名空间 iv_slam_mapping
