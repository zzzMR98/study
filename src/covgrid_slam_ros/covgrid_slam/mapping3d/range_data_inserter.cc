/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "covgrid_slam/mapping3d/range_data_inserter.h"

#include "Eigen/Core"
#include "covgrid_slam/mapping/probability_values.h"
#include "glog/logging.h"

namespace mapping3d {

namespace {

void InsertMissesIntoGrid(const std::vector<uint16>& miss_table,
                          const Eigen::Vector3d& origin,
                          const sensor::PointCloud& returns,
                          HybridGrid* hybrid_grid,
                          const int num_free_space_voxels) {
  const Eigen::Array3i origin_cell = hybrid_grid->GetCellIndex(origin);
  for (const sensor::Point& hit : returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);

    const Eigen::Array3i delta = hit_cell - origin_cell;
    const int num_samples = delta.cwiseAbs().maxCoeff();
    CHECK_LT(num_samples, 1 << 15);
    // 'num_samples' is the number of samples we equi-distantly place on the
    // line between 'origin' and 'hit'. (including a fractional part for sub-
    // voxels) It is chosen so that between two samples we change from one voxel
    // to the next on the fastest changing dimension.
    //
    // Only the last 'num_free_space_voxels' are updated for performance.
    for (int position = std::max(0, num_samples - num_free_space_voxels);
         position < num_samples; ++position) {
      const Eigen::Array3i miss_cell =
          origin_cell + delta * position / num_samples;
      hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
    }
  }
}

void InsertMissesIntoGrid(const std::vector<uint16>& miss_table,
                          const Eigen::Vector3d& origin,
                          const sensor::PointCloud& returns,
                          FeatureHybridGrid* hybrid_grid,
                          const int num_free_space_voxels) {
  const Eigen::Array3i origin_cell = hybrid_grid->GetCellIndex(origin);
  for (const sensor::Point& hit : returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);

    const Eigen::Array3i delta = hit_cell - origin_cell;
    const int num_samples = delta.cwiseAbs().maxCoeff();
    CHECK_LT(num_samples, 1 << 15);
    // 'num_samples' is the number of samples we equi-distantly place on the
    // line between 'origin' and 'hit'. (including a fractional part for sub-
    // voxels) It is chosen so that between two samples we change from one voxel
    // to the next on the fastest changing dimension.
    //
    // Only the last 'num_free_space_voxels' are updated for performance.
    for (int position = std::max(0, num_samples - num_free_space_voxels);
         position < num_samples; ++position) {
      const Eigen::Array3i miss_cell =
          origin_cell + delta * position / num_samples;
      hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
    }
  }
}

}  // namespace

proto::RangeDataInserterOptions CreateRangeDataInserterOptions(
    ::ivcommon::LuaParameterDictionary* parameter_dictionary) {
  proto::RangeDataInserterOptions options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_num_free_space_voxels(
      parameter_dictionary->GetInt("num_free_space_voxels"));
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

RangeDataInserter::RangeDataInserter(
    const proto::RangeDataInserterOptions& options)
    : options_(options),
      hit_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options_.hit_probability()))),
      miss_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options_.miss_probability()))) {}//the probability relative to the hit_numbers ,the index is the hit_number lzz

void RangeDataInserter::Insert(const sensor::RangeData& range_data,
                               HybridGrid* hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid);

  for (const sensor::Point& hit : range_data.returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
    hybrid_grid->ApplyLookupTable(hit_cell, hit_table_);//?
  }

  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.returns,
                       hybrid_grid, options_.num_free_space_voxels());
  hybrid_grid->FinishUpdate();// recover the cell kUpdateMarker lzz
}

void RangeDataInserter::Insert(const sensor::RangeData& range_data,
                               FeatureHybridGrid* hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid);

  hybrid_grid->clearindices();
  for (const sensor::Point& hit : range_data.returns) {
//    if(hit.type == sensor::Point::Type::kFlat)
      {
	const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
	hybrid_grid->addPoint(hit_cell, hit,hit_table_);//?
      }
  }
  InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.returns,
                       hybrid_grid, options_.num_free_space_voxels());
//  LOG(INFO)<<"range_data.misses:"<<range_data.misses.size();
//  InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.misses,
//                       hybrid_grid, options_.num_free_space_voxels());
  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).

  hybrid_grid->FinishUpdate();
}

void RangeDataInserter::Insert(const sensor::RangeData& range_data,
                               FeatureHybridGrid* hybrid_grid,float intensity) const {
  CHECK_NOTNULL(hybrid_grid);

  hybrid_grid->clearindices();
  for (sensor::Point hit : range_data.returns) {
//    if(hit.type == sensor::Point::Type::kFlat)
      {
    	if(hit.intensity>intensity)
		{
    		hit.intensity = intensity;
			const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
			hybrid_grid->addPoint(hit_cell, hit,hit_table_);//?
		}
      }
  }

  hybrid_grid->FinishUpdate();
}

void RangeDataInserter::Insert(const sensor::RangeData& range_data,
                               IntensityHybridGrid* hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid);

  for (sensor::Point hit : range_data.returns) {
//    if(hit.type == sensor::Point::Type::kFlat)
      {
    	if(hit.intensity<0.8)
    		continue;
    	hit.intensity = 0.9;
		const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
		hybrid_grid->addPoint(hit_cell, hit);//?
      }
  }

  hybrid_grid->FinishUpdate();
}

}  // namespace mapping_3d
