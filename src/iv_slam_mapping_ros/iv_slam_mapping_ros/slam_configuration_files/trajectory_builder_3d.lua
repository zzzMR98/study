MAX_3D_RANGE = 60.
TRAJECTORY_BUILDER_3D = {
    min_range = 2.,
    max_range = MAX_3D_RANGE,
    voxel_filter_size = 0.05,
    imu_gravity_time_constant = 9.8,
    range_data_write = false,
    serve_global_optimization = false,
    offline_map_invoke = false,
    offline_map_file_name_time = "20180417_161541",
    use_location_module = false,
    submaps = {
        high_resolution = 0.20,
        high_resolution_max_range = 60.,
        low_resolution = 0.6,
        num_range_data = 30,
        range_data_inserter = {
            hit_probability = 0.53,
            miss_probability = 0.49,
            num_free_space_voxels = 1
        },
        twid_submap_write = true,
        trid_submap_write = false,
        visualization_trid_submap_write = false,
        twid_submap_display = false,
        trid_submap_display = false,
        enable_traversablearea_extraction = true,
        kxrayobstructedcellprobabilitylimit = 0.750, -- //0-1
        rough_intensity = 5, -- grid
        kminzdifference = 0.4 / 0.2, -- voxel
        kminzdifference_beyond = 0.8 / 0.2, -- voxel
        zdifference_change_thresh = 40 / 0.2, -- voxel
        obstacle_emptythresh = 0.92, -- 0-1
        extension_index = 20 / 0.2 -- grid
    }
}
