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

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  --use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ----------------------------------------------------------
-- ------------------- TRAJECTORY BUILDER -------------------
-- ----------------------------------------------------------

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1 --3
--TRAJECTORY_BUILDER_3D.min_range = 1.
TRAJECTORY_BUILDER_3D.max_range = 130. --60.
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15 --0.15

--[[
-- ------------------ HIGH RESOLUTION ADAPTIVE VOXEL FILTER-
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15
-- ------------------- LOW RESOLUTION ADAPTIVE VOXEL FILTER -
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 130.
]]--

-- ------------------- CERES SCAN MATCHER -------------------
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 50 --5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 50 --4e2
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.

--TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004

-- ------------------- SUBMAPS ------------------------------
--[[
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10 --0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
]]--

-- ----------------------------------------------------------
-- ---------------------- MAP BUILDER -----------------------
-- ----------------------------------------------------------

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7 --4

-- ----------------------------------------------------------
-- ----------------------- POSE GRAPH -----------------------
-- ----------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes = 90--0 --90

--[[
-- ------------------- CONSTRAINT BUILDER -------------------
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 --0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.55--0.8 --0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6 --0.6

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10. --5.
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5. --1.

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 1e2
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 30 --10
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 4 --1

POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3
]]--

--[[
-- ------------------- OPTIMIZATION PROBLEM -----------------
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5

POSE_GRAPH.optimization_problem.log_solver_summary = true

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100 --50
]]--

POSE_GRAPH.optimization_problem.log_solver_summary = true


return options
