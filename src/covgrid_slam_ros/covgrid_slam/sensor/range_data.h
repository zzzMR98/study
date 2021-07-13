
#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "ivcommon/common/port.h"

#include "covgrid_slam/sensor/compressed_point_cloud.h"
#include "covgrid_slam/sensor/point_cloud.h"

namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData {
  Eigen::Vector3d origin;
  ::ivcommon::Time stamp;
  PointCloud returns;
  PointCloud misses;
};

RangeData TransformRangeData(const RangeData& range_data,
                             const ivcommon::transform::Rigid3d& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Like RangeData but with compressed point clouds. The point order changes
// when converting from RangeData.
struct CompressedRangeData {
  Eigen::Vector3d origin;
  ::ivcommon::Time stamp;
  CompressedPointCloud returns;
  CompressedPointCloud misses;
};

CompressedRangeData Compress(const RangeData& range_data);

RangeData Decompress(const CompressedRangeData& compressed_range_data);

}  // namespace sensor

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_
