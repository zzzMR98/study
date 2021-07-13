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

#ifndef CARTOGRAPHER_MAPPING_2D_RAY_CASTING_H_
#define CARTOGRAPHER_MAPPING_2D_RAY_CASTING_H_

#include <vector>

#include "ivcommon/common/port.h"
#include "ivcommon/transform/transform.h"
#include "covgrid_slam/mapping/probability_grid.h"
#include "covgrid_slam/sensor/point_cloud.h"
#include "covgrid_slam/sensor/range_data.h"

namespace mapping {

// For each ray in 'range_data', inserts hits and misses into
// 'probability_grid'. Hits are handled before misses.
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table, bool insert_free_space,
              ProbabilityGrid* probability_grid);

}  // namespace mapping_2d

#endif  // CARTOGRAPHER_MAPPING_2D_RAY_CASTING_H_
