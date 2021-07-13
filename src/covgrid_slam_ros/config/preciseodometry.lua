-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

MAX_3D_RANGE = 60.
histogram_size = 120
options={
	min_range = 1.,
	max_range = MAX_3D_RANGE,
	scans_per_accumulation = 1,
	voxel_filter_size = 0.15,
	without_imu = true,


	high_resolution_adaptive_voxel_filter = {
	max_length = 2.,
	min_num_points = 150,
	max_range = 20.,
	},

	low_resolution_adaptive_voxel_filter = {
	max_length = 4.5,
	min_num_points = 200,
	max_range = MAX_3D_RANGE,
	},


	feature_adaptive_voxel_filter = {
	max_length = 2.5,
	min_num_points = 450,
	max_range = 40,
	},


	intensity_adaptive_voxel_filter = {
	max_length = 1,
	min_num_points = 450,
	max_range = 30,
	},


	use_online_correlative_scan_matching = false,
	real_time_correlative_scan_matcher = {
	linear_search_window = 0.15,
	angular_search_window = math.rad(0.1),
	translation_delta_cost_weight = 1e-1,
	rotation_delta_cost_weight = 1e-1,
	},

	ceres_scan_matcher = {
	occupied_space_weight_0 = -1,
	occupied_space_weight_1 = -6.,
	feature_space_weight_0 = 20.,
	translation_weight = 6.,
	rotation_weight = 8e2,
	g2o_optimize = true;
	fuse_ins = false;
	--rotation_weight = 6e1,
	only_optimize_yaw = false,
	ceres_solver_options = {
	  use_nonmonotonic_steps = false,
	  max_num_iterations = 10,
	  num_threads = 1,
	},
	g2o_solver_options = {
	  max_num_iterations = 10,
	},

	},

	motion_filter = {
	max_time_seconds = 0.2,
	max_distance_meters = 0.15,
	max_angle_radians = 0.004,
	},

	imu_gravity_time_constant = 10.,


	submaps = {
	update_common_map = true;
	high_resolution = 0.15,
	high_resolution_max_range = 60.,
	low_resolution = 0.5,
	feature_resolution = 0.5,
	num_range_data = 40,
	rotational_histogram_size = histogram_size,
	range_data_inserter = {
	  hit_probability = 0.55,
	  miss_probability = 0.49,
	  num_free_space_voxels = 2,
	},

	automode_flag = true,
	update_flag = true,
	savemap_flag = false,
	readmap_flag = false,
	savemap_dir = "/home/jkj/catkin_workspace/map",
	readmap_dir = "/home/jkj/catkin_workspace/map/2018-04-12_20-21-29",

	},

  fast_correlative_scan_matcher_3d = {
  --branch_and_bound_depth = 8,
  branch_and_bound_depth = 8,
  full_resolution_depth = 3,
  rotational_histogram_size = histogram_size,
  min_rotational_score = 0.77,
  --min_rotational_score = 0.5,
      --linear_xy_search_window = 5.,
  linear_xy_search_window = 10.,
--  +-100m
  --linear_z_search_window = 1.,
  linear_z_search_window = 6.,
--  +-60m
  angular_search_window = math.rad(30.),
  },
  
  initmatch = {
     min_score = 0.6;
     min_low_resolution_score = 0.3;
  },
}
return options
