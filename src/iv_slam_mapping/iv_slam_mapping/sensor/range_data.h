#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#include "ivcommon/common/port.h"
#include "iv_slam_mapping/sensor/compressed_point_cloud.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/sensor/proto/sensor.pb.h"
namespace iv_slam_mapping {
namespace sensor {
///
/// Rays begin at 'origin'. 'returns' are the points where obstructions were
/// detected. 'misses' are points in the direction of rays for which no return
/// was detected, and were inserted at a configured distance. It is assumed that
/// between the 'origin' and 'misses' is free space.
  ///
struct RangeData {
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;
};
///
///对点云进行坐标变换
///
RangeData TransformRangeData(const RangeData& range_data,
                             const ::ivcommon::transform::Rigid3f& transform);
///
/// Like RangeData but with compressed point clouds. The point order changes
/// when converting from RangeData.
///
struct CompressedRangeData {
  Eigen::Vector3f origin;
  CompressedPointCloud returns;
  CompressedPointCloud misses;
};
///
///将压缩后的点云转为ｐｒｏｔｏｃｏｌ格式
///
proto::CompressedRangeData ToProto(
    const CompressedRangeData& compressed_range_data);
///
///将ｐｒｏｔｏ格式的压缩点云转为普通格式的压缩点云
///
CompressedRangeData FromProto(const proto::CompressedRangeData& proto);
///
///压缩点云
///
CompressedRangeData Compress(const RangeData& range_data);
///
///解压缩点云
///
RangeData Decompress(const CompressedRangeData& compressed_range_data);

}  /// 命名空间 sensor
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_
