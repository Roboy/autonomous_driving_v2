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

include "rickshaw_outdoor.lua"

TRAJECTORY_BUILDER.pure_localization = true


-- TUNE
POSE_GRAPH.optimize_every_n_nodes = 1 --5 --10 -- decrease
POSE_GRAPH.global_sampling_ratio = 0.001 --0.001 --0.003 -- decrease
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 --0.1 --0.05 --0.3 -- decrease

POSE_GRAPH.global_constraint_search_after_n_seconds = 3 --10. --5.
POSE_GRAPH.constraint_builder.min_score = 0.75 --0.75

--[[
-- For lower Latency also consider
-- (global SLAM latency)
POSE_GRAPH.constraint_builder.min_score = 0.55 -- increase
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.max_constraint_distance = 15 --
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05 -- increase
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5. --5. -- decrease
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1. --1. -- decrease
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.) -- decrease
POSE_GRAPH.global_constraint_search_after_n_seconds = 5 --10. -- increase


-- (local SLAM latency)
]]--

-- maybe helps tuning
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 50 --5
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 50 --80

return options
