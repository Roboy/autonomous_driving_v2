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

VOXEL_SIZE = 5e-2

include "transform.lua"

options = {
  tracking_frame = "imu",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 130.,
    },
    --{
    --  action = "voxel_filter_and_remove_moving_objects",
    --  voxel_size = VOXEL_SIZE,
    --},
    --{
    --  action = "fixed_ratio_sampler",
    --  sampling_ratio = 0.9
    --},
    {
      action = "dump_num_points",
    },
    --[[
    -- Gray X-Rays. These only use geometry to color pixels.
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all",
      transform = YZ_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xy",
      separate_floors = true,
      draw_trajectories = false,
      transform = XY_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all",
      transform = XZ_TRANSFORM,
    },
    ]]--
    -- We now use the intensities to color our points. We apply a linear
    -- transform to clamp our intensity values into [0, 255] and then use this
    -- value for RGB of our points. Every stage in the pipeline after this now
    -- receives colored points.
    --
    -- We write xrays again. These now use geometry and the intensities to
    -- color pixels - they look quite similar, just a little lighter.
    {
      action = "intensity_to_color",
      min_intensity = 0.,
      max_intensity = 4095.,
    },
    --[[
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all_intensity",
      transform = YZ_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xy_intensity",
      draw_trajectories = false,
      transform = XY_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all_intensity",
      transform = XZ_TRANSFORM,
    },
    ]]--

    -- We also write a PLY file at this stage, because gray points look good.
    -- The points in the PLY can be visualized using
    -- https://github.com/googlecartographer/point_cloud_viewer.
    
    {
      action = "write_ply",
      filename = "points.ply",
    },
    
    {
      action = "write_probability_grid",
      resolution = 0.05,
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.56,
        miss_probability = 0.49,
      },
      draw_trajectories = false,
      filename = "probability_map"
    },
  }
}

return options
