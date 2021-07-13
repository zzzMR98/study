/*
 * rotation_histogram.h
 *
 *  Created on: Mar 6, 2018
 *      Author: jkj
 */

#ifndef COVGRID_SLAM_MAPPING3D_SCAN_MATCHING_ROTATIONAL_HISTOGRAM_H_
#define COVGRID_SLAM_MAPPING3D_SCAN_MATCHING_ROTATIONAL_HISTOGRAM_H_
#include <map>
#include <vector>

#include "ivcommon/common/math.h"

#include "covgrid_slam/mapping/trajectory_node.h"
#include "covgrid_slam/sensor/point_cloud.h"


namespace mapping3d {
namespace scan_matching {

namespace {

constexpr float kMinDistance = 0.2f;
constexpr float kMaxDistance = 0.9f;
constexpr float kSliceHeight = 0.2f;

void AddValueToHistogram(float angle, const float value,
                         Eigen::VectorXf* histogram) {
  // Map the angle to [0, pi), i.e. a vector and its inverse are considered to
  // represent the same angle.
  while (angle > static_cast<float>(M_PI)) {
    angle -= static_cast<float>(M_PI);
  }
  while (angle < 0.f) {
    angle += static_cast<float>(M_PI);
  }
  const float zero_to_one = angle / static_cast<float>(M_PI);
  const int bucket = ::ivcommon::Clamp<int>(
      ::ivcommon::RoundToInt(histogram->size() * zero_to_one - 0.5f), 0,
      histogram->size() - 1);
  (*histogram)(bucket) += value;
}

Eigen::Vector3d ComputeCentroid(const sensor::PointCloud& slice) {
  CHECK(!slice.empty());
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& point : slice) {
    sum += point;
  }
  return sum / static_cast<float>(slice.size());
}

struct AngleValuePair {
  float angle;
  float value;
};

void AddPointCloudSliceToValueVector(
    const sensor::PointCloud& slice,
    std::vector<AngleValuePair>* value_vector) {
  if (slice.empty()) {
    return;
  }
  // We compute the angle of the ray from a point to the centroid of the whole
  // point cloud. If it is orthogonal to the angle we compute between points, we
  // will add the angle between points to the histogram with the maximum weight.
  // This is to reject, e.g., the angles observed on the ceiling and floor.
  const Eigen::Vector3d centroid = ComputeCentroid(slice);
  Eigen::Vector3d last_point = slice.front();
  for (const Eigen::Vector3d& point : slice) {
    const Eigen::Vector2d delta = (point - last_point).head<2>();
    const Eigen::Vector2d direction = (point - centroid).head<2>();
    const float distance = delta.norm();
    if (distance < kMinDistance || direction.norm() < kMinDistance) {
      continue;
    }
    if (distance > kMaxDistance) {
      last_point = point;
      continue;
    }
    const float angle = ::ivcommon::atan2(delta);
    const float value = std::max(
        0., 1. - std::abs(delta.normalized().dot(direction.normalized())));//1-cos(angle)
    value_vector->push_back(AngleValuePair{angle, value});
  }
}

// A function to sort the points in each slice by angle around the centroid.
// This is because the returns from different rangefinders are interleaved in
// the data.
sensor::PointCloud SortSlice(const sensor::PointCloud& slice) {
  struct SortableAnglePointPair {
    bool operator<(const SortableAnglePointPair& rhs) const {
      return angle < rhs.angle;
    }

    float angle;
    Eigen::Vector3d point;
  };
  const Eigen::Vector3d centroid = ComputeCentroid(slice);
  std::vector<SortableAnglePointPair> by_angle;
  by_angle.reserve(slice.size());
  for (const Eigen::Vector3d& point : slice) {
    const Eigen::Vector2d delta = (point - centroid).head<2>();
    if (delta.norm() < kMinDistance) {
      continue;
    }
    by_angle.push_back(SortableAnglePointPair{::ivcommon::atan2(delta), point});
  }
  std::sort(by_angle.begin(), by_angle.end());
  sensor::PointCloud result;
  for (const auto& pair : by_angle) {
    result.push_back(pair.point);
  }
  return result;
}

std::vector<AngleValuePair> GetValuesForHistogram(
    const sensor::PointCloud& point_cloud) {
  std::map<int, sensor::PointCloud> slices;
  for (const Eigen::Vector3d& point : point_cloud) {
    slices[::ivcommon::RoundToInt(point.z() / kSliceHeight)].push_back(point);
  }
  std::vector<AngleValuePair> result;
  for (const auto& slice : slices) {
    AddPointCloudSliceToValueVector(SortSlice(slice.second), &result);
  }
  return result;
}

void AddValuesToHistogram(const std::vector<AngleValuePair>& value_vector,
                          const float rotation, Eigen::VectorXf* histogram) {
  for (const AngleValuePair& pair : value_vector) {
    AddValueToHistogram(pair.angle + rotation, pair.value, histogram);
  }
}

}  // namespace

}  // namespace scan_matching
}  // namespace mapping_3d


#endif /* COVGRID_SLAM_MAPPING3D_SCAN_MATCHING_ROTATIONAL_HISTOGRAM_H_ */
