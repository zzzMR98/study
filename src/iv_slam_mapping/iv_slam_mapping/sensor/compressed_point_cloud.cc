
#include "iv_slam_mapping/sensor/compressed_point_cloud.h"

#include <limits>

#include "ivcommon/common/math.h"
#include "iv_slam_mapping/mapping_3d/hybrid_grid.h"

namespace iv_slam_mapping {
namespace sensor {

namespace {
///
/// Points are encoded on a fixed grid with a grid spacing of 'kPrecision' with
/// integers. Points are organized in blocks, where each point is encoded
/// relative to the block's origin in an int32 with 'kBitsPerCoordinate' bits per
/// coordinate.
///  
constexpr float kPrecision = 0.001f;  /// in meters.
constexpr int kBitsPerCoordinate = 10;
constexpr int kCoordinateMask = (1 << kBitsPerCoordinate) - 1;
constexpr int kMaxBitsPerDirection = 23;

}  // namespace
///
///创建正向迭代器
///
CompressedPointCloud::ConstIterator::ConstIterator( const CompressedPointCloud* compressed_point_cloud)
    : compressed_point_cloud_(compressed_point_cloud),remaining_points_(compressed_point_cloud->num_points_),
      remaining_points_in_current_block_(0), input_(compressed_point_cloud->point_data_.begin()) {
  if (remaining_points_ > 0) {
    ReadNextPoint();
  }
}
///
///创建逆向迭代器
///
CompressedPointCloud::ConstIterator CompressedPointCloud::ConstIterator::EndIterator( const CompressedPointCloud* compressed_point_cloud) {
  ConstIterator end_iterator(compressed_point_cloud);
  end_iterator.remaining_points_ = 0;
  return end_iterator;
}
///
///取址操作
///
Eigen::Vector3f CompressedPointCloud::ConstIterator::operator*() const {
  CHECK_GT(remaining_points_, 0);
  return current_point_;
}
///
///重载运算符＋＋
///
CompressedPointCloud::ConstIterator& CompressedPointCloud::ConstIterator::operator++() {
  --remaining_points_;
  if (remaining_points_ > 0) {
    ReadNextPoint();
  }
  return *this;
}
///
///重载运算符，不等于
///
bool CompressedPointCloud::ConstIterator::operator!=(
    const ConstIterator& it) const {
  CHECK(compressed_point_cloud_ == it.compressed_point_cloud_);
  return remaining_points_ != it.remaining_points_;
}
///
///读取并解析下一个压缩点
///
void CompressedPointCloud::ConstIterator::ReadNextPoint() {
  if (remaining_points_in_current_block_ == 0) {
    remaining_points_in_current_block_ = *input_++;
    for (int i = 0; i < 3; ++i) {
      current_block_coordinates_[i] = *input_++ << kBitsPerCoordinate;
    }
  }
  --remaining_points_in_current_block_;
  const int point = *input_++;
  constexpr int kMask = (1 << kBitsPerCoordinate) - 1;
  current_point_[0] =
      (current_block_coordinates_[0] + (point & kMask)) * kPrecision;
  current_point_[1] = (current_block_coordinates_[1] +
                       ((point >> kBitsPerCoordinate) & kMask)) *
                      kPrecision;
  current_point_[2] =
      (current_block_coordinates_[2] + (point >> (2 * kBitsPerCoordinate))) *
      kPrecision;
}
///
///从普通点云构造压缩点云
///
CompressedPointCloud::CompressedPointCloud(const PointCloud& point_cloud)
    : num_points_(point_cloud.size()) {
      ///
      ///将点云组织在ｂｌｏｃｋｓ中，以便于遍历和查询
      ///
  struct RasterPoint {
    Eigen::Array3i point;
    int index;
  };
  using Blocks = mapping_3d::HybridGridBase<std::vector<RasterPoint>>;
  Blocks blocks(kPrecision);
  int num_blocks = 0;
  CHECK_LE(point_cloud.size(), std::numeric_limits<int>::max());
  for (int point_index = 0; point_index < static_cast<int>(point_cloud.size());
       ++point_index) {
    const Eigen::Vector3f& point = point_cloud[point_index];
    CHECK_LT(point.cwiseAbs().maxCoeff() / kPrecision,
             1 << kMaxBitsPerDirection)
        << "Point out of bounds: " << point;
    Eigen::Array3i raster_point;
    Eigen::Array3i block_coordinate;
    for (int i = 0; i < 3; ++i) {
      raster_point[i] = ::ivcommon::RoundToInt(point[i] / kPrecision);
      block_coordinate[i] = raster_point[i] >> kBitsPerCoordinate;
      raster_point[i] &= kCoordinateMask;
    }
    auto* const block = blocks.mutable_value(block_coordinate);
    num_blocks += block->empty();
    block->push_back({raster_point, point_index});
  }

  ///
  ///对每一个ｂｌｏｃｋ编码，并将ｂｌｏｃｋ中的数据以及属性压缩存到ｐｏｉｎｔ_data_中
  ///
  point_data_.reserve(4 * num_blocks + point_cloud.size());
  for (Blocks::Iterator it(blocks); !it.Done(); it.Next(), --num_blocks) {
    const auto& raster_points = it.GetValue();
    CHECK_LE(raster_points.size(), std::numeric_limits<int32>::max());
    point_data_.push_back(raster_points.size());
    const Eigen::Array3i block_coordinate = it.GetCellIndex();
    point_data_.push_back(block_coordinate.x());
    point_data_.push_back(block_coordinate.y());
    point_data_.push_back(block_coordinate.z());
    for (const RasterPoint& raster_point : raster_points) {
      point_data_.push_back((((raster_point.point.z() << kBitsPerCoordinate) +
                              raster_point.point.y())
                             << kBitsPerCoordinate) +
                            raster_point.point.x());
    }
  }
  CHECK_EQ(num_blocks, 0);
}
///
///从ｐｒｏｔｏ格式的压缩点云构造压缩点云
///
CompressedPointCloud::CompressedPointCloud(
    const proto::CompressedPointCloud& proto) {
  num_points_ = proto.num_points();
  const int data_size = proto.point_data_size();
  point_data_.reserve(data_size);
  // TODO(wohe): Verify that 'point_data_' does not contain malformed data.
  for (int i = 0; i != data_size; ++i) {
    point_data_.emplace_back(proto.point_data(i));///proto.point_data(i)为已经压缩过的点
  }
}
///
///判断压缩点云是否为空
///
bool CompressedPointCloud::empty() const { return num_points_ == 0; }
///
///返回压缩点云中点的数量
///
size_t CompressedPointCloud::size() const { return num_points_; }
///
///创建正向迭代器
///
CompressedPointCloud::ConstIterator CompressedPointCloud::begin() const {
  return ConstIterator(this);
}
///
///创建逆向迭代器
///
CompressedPointCloud::ConstIterator CompressedPointCloud::end() const {
  return ConstIterator::EndIterator(this);
}
///
///解压缩点云，利用迭代器以及重载运算符“*”解压缩点云，迭代器负责获得current_point_，而运算符负责返回current_point_
///
PointCloud CompressedPointCloud::Decompress() const {
  PointCloud decompressed;
  for (const Eigen::Vector3f& point : *this) {
    decompressed.push_back(point);
  }
  return decompressed;
}
///
///重载运算符相等
///
bool sensor::CompressedPointCloud::operator==(
    const sensor::CompressedPointCloud& right_hand_container) const {
  return point_data_ == right_hand_container.point_data_ &&
         num_points_ == right_hand_container.num_points_;
}
///
///将压缩点云存储到ｐｒｏｔｏ中
///
proto::CompressedPointCloud CompressedPointCloud::ToProto() const {
  proto::CompressedPointCloud result;
  result.set_num_points(num_points_);
  for (const int32 data : point_data_) {
    result.add_point_data(data);
  }
  return result;
}

}  /// 命名空间 sensor
}  /// 命名空间 iv_slam_mapping
