# This file contains code for interfacing with Airsim

import JSON3
using Test
using SubmodularMaximization
using MDMA

export configs_from_file, save_solution, load_solution

function configs_from_file(filename::String, experiment_name::String, move_dist::Number)::MultiDroneMultiActorConfigs

    json_string = read(filename, String)
    json_root = JSON3.read(json_string)

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
            weight = loc_pos["weight"]
            target_row[id] = multiply_face_weights(Target(x, y, h, id), weight)
        end
        target_trajectories[time, :] = target_row
    end


    # Making the object
    grid = MDMA_Grid(Int64(scale["x"]), Int64(scale["y"]), horizon)
    fov = robot_fovs[1]
    sensor = ViewConeSensor(fov, sense_dist)

    return MultiDroneMultiActorConfigs(experiment_name=experiment_name, num_robots=num_robots, target_trajectories=target_trajectories, grid=grid, sensor=sensor, horizon=horizon, move_dist=move_dist)

end

# Serialize the solution into a json object
function save_solution(experiment_name::String, path_to_experiments::String,subdir::String, solution::Solution, multi_configs::MultiDroneMultiActorConfigs)
    root_dict = Dict("value" => solution.value, "elements" => solution.elements)

    directory = "$(path_to_experiments)/$(experiment_name)/$(subdir)"
    mkpath(directory)
    mkpath("$(directory)/renders")
    open("$(directory)/solution.json", "w") do io
        JSON3.pretty(io, root_dict)
    end

    println("Saving solution to $(directory)/solution.json")
    render_paths(solution, multi_configs, "$(directory)/renders")

end


function load_solution(filename)
    json_string = read(filename, String)
    root_dict = JSON3.read(json_string)

    solution_value = root_dict["value"]
    solution_elements_dict = root_dict["elements"]
    # Solution Elements is an array of array of (Tuple, Vector{MDPState})

    elements = Tuple{Int64,Vector{MDPState}}[]
    for robot_dict in solution_elements_dict
        robot_id = robot_dict[1]
        robot_states = MDPState[]
        for states in robot_dict[2]
            state_dict = states["state"]
            depth = states["depth"]
            horizon = states["horizon"]
            prev = nothing
            state = UAVState(state_dict["x"], state_dict["y"], Symbol(state_dict["heading"]))
            push!(robot_states, MDPState(state, depth, horizon, prev))
        end
        push!(elements, (robot_id, robot_states))
    end

    Solution(solution_value, elements)

end
