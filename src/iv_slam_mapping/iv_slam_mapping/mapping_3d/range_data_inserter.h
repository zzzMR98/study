#ifndef CARTOGRAPHER_MAPPING_3D_RANGE_DATA_INSERTER_H_
#define CARTOGRAPHER_MAPPING_3D_RANGE_DATA_INSERTER_H_
#include "iv_slam_mapping/mapping_3d/hybrid_grid.h"
#include "iv_slam_mapping/mapping_3d/proto/range_data_inserter_options.pb.h"
#include "iv_slam_mapping/sensor/point_cloud.h"
#include "iv_slam_mapping/sensor/range_data.h"
#include "ivcommon/common/lua_parameter_dictionary.h"

namespace iv_slam_mapping {
namespace mapping_3d {
///
  ///从ｌｕａ配置文件中创建RangeDataInserterOptions
  ///
proto::RangeDataInserterOptions CreateRangeDataInserterOptions(::ivcommon::LuaParameterDictionary* parameter_dictionary);

class RangeDataInserter {
 public:
  explicit RangeDataInserter(const proto::RangeDataInserterOptions& options);///构造函数

  RangeDataInserter(const RangeDataInserter&) = delete;///禁止复制对象
  RangeDataInserter& operator=(const RangeDataInserter&) = delete;///禁止复制对象
///
  /// Inserts 'range_data' into 'hybrid_grid'.
  ///
  void Insert(const sensor::RangeData& range_data,HybridGrid* hybrid_grid) const;

 private:
  const proto::RangeDataInserterOptions options_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  /// 命名空间 mapping_3d
}  /// 命名空间 iv_slam_mapping

#endif  // CARTOGRAPHER_MAPPING_3D_RANGE_DATA_INSERTER_H_
