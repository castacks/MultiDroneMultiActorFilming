# This file contains code for interfacing with Airsim

import JSON
using Test
using MDMA

export configs_from_file

function configs_from_file(filename::String, move_dist::Number)::MultiDroneMultiActorConfigs

    json_root = JSON.parsefile(filename)

    scene = json_root["scene"]
    scale = scene["scale"]
    blend_file = json_root["filename"]
    horizon = json_root["num_frames"]
    num_targets = json_root["num_targets"]
    robot_fovs = json_root["robot_fovs"]
    num_robots = json_root["num_robots"]
    sense_dist = json_root["sense_dist"]

    target_trajectories = Array{Target,2}(undef, horizon, num_targets)
    for (time, target_set) in enumerate(json_root["actor_positions"])
        target_row = Vector{Target}(undef, num_targets)
        for (id, loc_pos) in enumerate(target_set)
            loc = loc_pos["location"]
            rot = loc_pos["rotation"]
            x = loc[1]
            y = loc[2]
            h = rot[3] # Take rotatin around z as the "heading"
            target_row[id] = Target(x, y, h, id)
        end
        target_trajectories[time, :] = target_row
    end


    # Making the object
    grid = MDMA_Grid(Int64(scale["x"]), Int64(scale["y"]), horizon)
    fov = robot_fovs[1]
    sensor = ViewConeSensor(fov, sense_dist)

    return MultiDroneMultiActorConfigs(num_robots=num_robots, target_trajectories=target_trajectories, grid=grid, sensor=sensor, horizon=horizon, move_dist=move_dist)

end

map(x->MDMA.random_state(horizon, grid), 1:num_robots)

@testset "test_file_parse" begin
    configs_from_file("../../blender/four_split_data.json", 3)
end
