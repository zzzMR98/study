
#ifndef CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_

#include <iterator>
#include <vector>

#include "Eigen/Core"
#include "ivcommon/common/port.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/sensor/proto/sensor.pb.h"

namespace iv_slam_mapping {
namespace sensor {
///
/// A compressed representation of a point cloud consisting of a collection of
/// points (Vector3f).
/// Internally, points are grouped by blocks. Each block encodes a bit of meta
/// data (number of points in block, coordinates of the block) and encodes each
/// point with a fixed bit rate in relation to the block.
  ///
class CompressedPointCloud {
 public:
   
   class ConstIterator;

  CompressedPointCloud() : num_points_(0) {}
  explicit CompressedPointCloud(const PointCloud& point_cloud);///从普通点云构造压缩点云
  explicit CompressedPointCloud(const proto::CompressedPointCloud& proto);///从ｐｒｏｔｏ格式的压缩点云构造压缩点云
///
  /// Returns decompressed point cloud.
  ///
  PointCloud Decompress() const;

  bool empty() const;
  size_t size() const;
  ConstIterator begin() const;
  ConstIterator end() const;

  bool operator==(const CompressedPointCloud& right_hand_container) const;
  proto::CompressedPointCloud ToProto() const;

 private:
  std::vector<int32> point_data_;
  size_t num_points_;
};
///
/// Forward iterator for compressed point clouds.
///
class CompressedPointCloud::ConstIterator
    : public std::iterator<std::forward_iterator_tag, Eigen::Vector3f> {
 public:
   ///
  /// Creates begin iterator.
   ///
  explicit ConstIterator(const CompressedPointCloud* compressed_point_cloud);
///
  /// Creates end iterator.
  ///
  static ConstIterator EndIterator(
      const CompressedPointCloud* compressed_point_cloud);
///
///重载运算符　取址操作
///
  Eigen::Vector3f operator*() const;
  ///
///重载运算符＋＋
///
  ConstIterator& operator++();
  ///
///重载运算符　不等于
///
  bool operator!=(const ConstIterator& it) const;

 private:
   ///
  /// Reads next point from buffer. Also handles reading the meta data of the
  /// next block, if the current block is depleted.
   ///
  void ReadNextPoint();

  const CompressedPointCloud* compressed_point_cloud_;
  size_t remaining_points_;
  int32 remaining_points_in_current_block_;
  Eigen::Vector3f current_point_;
  Eigen::Vector3i current_block_coordinates_;
  std::vector<int32>::const_iterator input_;
};

}  /// 命名空间 sensor
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
