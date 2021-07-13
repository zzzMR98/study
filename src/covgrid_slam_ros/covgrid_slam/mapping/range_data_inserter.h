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

#ifndef CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_H_
#define CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_H_

#include <utility>
#include <vector>

#include "ivcommon/common/lua_parameter_dictionary.h"
#include "ivcommon/common/port.h"
#include "covgrid_slam/mapping/proto/range_data_inserter_options.pb.h"

#include "covgrid_slam/mapping/probability_grid.h"
#include "covgrid_slam/mapping/xy_index.h"
#include "covgrid_slam/sensor/point_cloud.h"
#include "covgrid_slam/sensor/range_data.h"

namespace mapping {

proto::RangeDataInserterOptions CreateRangeDataInserterOptions(
    ::ivcommon::LuaParameterDictionary* parameter_dictionary);
/// \brief 子地图插入的接口类
///
class RangeDataInserter {
 public:
  explicit RangeDataInserter(const proto::RangeDataInserterOptions& options);

  RangeDataInserter(const RangeDataInserter&) = delete;
  RangeDataInserter& operator=(const RangeDataInserter&) = delete;

  // Inserts 'range_data' into 'probability_grid'.
  void Insert(const sensor::RangeData& range_data,
              ProbabilityGrid* probability_grid) const;

 private:
  const proto::RangeDataInserterOptions options_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping_2d

#endif  // CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_H_
