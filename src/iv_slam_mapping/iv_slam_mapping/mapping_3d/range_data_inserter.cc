#include "iv_slam_mapping/mapping_3d/range_data_inserter.h"
#include "Eigen/Core"
#include "iv_slam_mapping/mapping/probability_values.h"
#include "glog/logging.h"

namespace iv_slam_mapping {
namespace mapping_3d {
///
  ///从ｌｕａ配置文件中创建RangeDataInserterOptions
  ///
proto::RangeDataInserterOptions CreateRangeDataInserterOptions( ::ivcommon::LuaParameterDictionary* parameter_dictionary) {
  proto::RangeDataInserterOptions options;
  options.set_hit_probability( parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability( parameter_dictionary->GetDouble("miss_probability"));
  options.set_num_free_space_voxels( parameter_dictionary->GetInt("num_free_space_voxels"));
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}
///
///构造函数
///
RangeDataInserter::RangeDataInserter( const proto::RangeDataInserterOptions& options) : options_(options),
      hit_table_(mapping::ComputeLookupTableToApplyOdds(  mapping::Odds(options_.hit_probability()))),
      miss_table_(mapping::ComputeLookupTableToApplyOdds(  mapping::Odds(options_.miss_probability()))) {}///the probability relative to the hit_numbers ,the index is the hit_number lzz
///
  /// Inserts 'range_data' into 'hybrid_grid'.
  ///
void RangeDataInserter::Insert(const sensor::RangeData& range_data,   HybridGrid* hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid);
  for (const Eigen::Vector3f& hit : range_data.returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
    hybrid_grid->ApplyLookupTable(hit_cell, hit_table_);//?
  }
///
  /// By not starting a new update after hits are inserted, we give hits priority
  /// (i.e. no hits will be ignored because of a miss in the same cell).
  ///
  hybrid_grid->FinishUpdate();/// recover the cell kUpdateMarker lzz
}

}  // namespace mapping_3d
}  // namespace iv_slam_mapping
