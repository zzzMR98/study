#include "iv_slam_mapping/sensor/range_data.h"

#include "iv_slam_mapping/sensor/proto/sensor.pb.h"
#include "ivcommon/transform/transform.h"

namespace iv_slam_mapping {
namespace sensor {
///
///对点云进行坐标变换
///
RangeData TransformRangeData(const RangeData& range_data,
                             const ::ivcommon::transform::Rigid3f& transform) {
  return RangeData{transform * range_data.origin, TransformPointCloud(range_data.returns, transform),TransformPointCloud(range_data.misses, transform),};
}
///
///将压缩后的点云转为ｐｒｏｔｏｃｏｌ格式
///
proto::CompressedRangeData ToProto(
    const CompressedRangeData& compressed_range_data) {
  proto::CompressedRangeData proto;
  *proto.mutable_origin() = ::ivcommon::transform::ToProto(compressed_range_data.origin);
  *proto.mutable_returns() = compressed_range_data.returns.ToProto();
  *proto.mutable_misses() = compressed_range_data.misses.ToProto();
  return proto;
}
///
///将ｐｒｏｔｏ格式的压缩点云转为普通格式的压缩点云
///
CompressedRangeData FromProto(const proto::CompressedRangeData& proto) {
  return CompressedRangeData{ ::ivcommon::transform::ToEigen(proto.origin()),  CompressedPointCloud(proto.returns()), CompressedPointCloud(proto.misses()),
  };
}
///
///压缩点云
///
CompressedRangeData Compress(const RangeData& range_data) {
  return CompressedRangeData{ range_data.origin, CompressedPointCloud(range_data.returns), CompressedPointCloud(range_data.misses),};
}
///
///解压缩点云
///
RangeData Decompress(const CompressedRangeData& compressed_range_data) {
  return RangeData{compressed_range_data.origin, compressed_range_data.returns.Decompress(), compressed_range_data.misses.Decompress()};
}

}  // namespace sensor
}  // namespace iv_slam_mapping
