using SubmodularMaximization
using MDMA
using DataFrames
using Images
using FileIO

export evaluate_solution, PPAEvaluation, ImageEvaluation

struct ImageEvaluation end
struct PPAEvaluation end

function evaluate_solution(
    experiment_name::String,
    path_to_experiments::String,
    _::ImageEvaluation,
)::DataFrame

    root_path = "$(path_to_experiments)/$(experiment_name)"
    blender_path = "$(root_path)/blender_output"

    multi_configs =
        configs_from_file("$(root_path)/$(experiment_name)_data.json", experiment_name, 3.0)

    horizon = multi_configs.horizon

    planner_types = ["greedy", "assignment", "formation"]
    if length(readdir(blender_path)) != 3
        println("Error, no planner directories")
        exit()
    end

    df = DataFrame(t = 1:horizon)
    for planner in planner_types
        cameras = readdir("$(blender_path)/$(planner)", join = true)
        evals = []
        for t = 1:horizon
            images_at_t = []
            for camera in cameras
                image_paths = readdir(camera, join = true)
                push!(images_at_t, image_paths[t])
            end

            val_now = 0.0
            for img_path in images_at_t
                img = load(img_path)
                f_img = float.(Gray.(img))
                val_now += sum(map(x -> x.val, f_img))
            end
            push!(evals, val_now)
        end
        df[!, planner] = evals
    end

    df
end

function evaluate_solution(
    experiment_name::String,
    path_to_experiments::String,
    _::PPAEvaluation,
)::DataFrame

    root_path = "$(path_to_experiments)/$(experiment_name)"

    # Just using this to get the trajectories and horizon
    multi_configs =
        configs_from_file("$(root_path)/$(experiment_name)_data.json", experiment_name, 3.0)

    target_trajectories = multi_configs.target_trajectories
    horizon = multi_configs.horizon

    # Make dataframe    
    df = DataFrame(t = 1:horizon)

    planner_types = ["greedy", "assignment", "formation"]
    for planner in planner_types
        solution = load_solution("$(root_path)/$(planner)/solution.json")

        # Get evaluation for all
        ppa_values = []

        # Might need to change the evaluation method.
        # First robot coverage array is zero
        # We have its trajectory so we can compute coverage data for the second robot
        # Lets compute the coverage on a per timestep basis if possible
        # In aggregate 
        coverage_data = generate_empty_coverage_data(multi_configs)
        for t = 1:horizon
            ppa_now = 0.0
            states = []

            # Would be nice to do this in a neater way
            # Get 3 states, one for each robot at this point in time
            for robot in solution.elements
                push!(states, robot[2][t])
            end

            targets = target_trajectories[t, :]
            for state in states
                for (target_id, target) in enumerate(targets)
                    for (f_id, face) in enumerate(target.faces)
                        distance = (
                            face.pos[1] - state.state.x,
                            face.pos[2] - state.state.y,
                            target_height / 2 - drone_height,
                        )
                        theta = dirAngle(state.state.heading)
                        look_direction = (cos(theta), sin(theta), 0.0)
                        # Set previous_coverage to zero
                        prior_pixel_density = coverage_data[t, target_id, f_id]
                        current_pixel_density = compute_coverage_value(
                            face,
                            look_direction,
                            distance,
                            prior_pixel_density,
                        )
                        coverage_data[t, target_id, f_id] = current_pixel_density
                        ppa_now += sqrt(current_pixel_density) - sqrt(prior_pixel_density)
                    end
                end
            end
            push!(ppa_values, ppa_now)
        end

        df[!, planner] = ppa_values
    end
    return df
end
