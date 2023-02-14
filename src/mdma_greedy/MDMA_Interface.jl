# This file contains code for interfacing with Airsim

import JSON
using Test
using MDMA

function configs_from_file(filename::String, move_dist::Number)::MultiDroneMultiActorConfigs

    json_root = JSON.parsefile(filename)

    scene = json_root["scene"]
    scale = scene["scale"]
    blend_file = json_root["filename"]
    horizon = json_root["num_frames"]
    num_targets = json_root["num_targets"]
    robot_fovs = json_root["robot_fovs"]
    num_robots = json_root["num_robots"]

    generate_target_trajectories = Array{Target, 2}(nothing, num_targets, horizon)
    for loc_pos in json_root["actor_positions"]
        loc = loc_pos["location"]
        rot = loc_pos["rotation"]



end


@testset "test_file_parse" begin
    configs_from_file("../../blender/four_split_data.json", 3)
    println(pwd())
end
