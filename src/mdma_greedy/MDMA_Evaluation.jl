using SubmodularMaximization
using MDMA
using DataFrames

export evaluate_solution, PPAEvaluation, ImageEvaluation

struct ImageEvaluation end
struct PPAEvaluation end

function evaluate_solution_visual(
    experiment_name::String,
    path_to_experiments::String, _::ImageEvaluation)

end

function evaluate_solution(experiment_name::String, path_to_experiments::String, _::PPAEvaluation)::DataFrame

    root_path = "$(path_to_experiments)/$(experiment_name)"
    target_path = "$(path_to_experiments)/$(experiment_name)/$(experiment_name)_data.json"

    # Just using this to get the trajectories and horizon
    multi_configs = configs_from_file(
        "$(root_path)/$(experiment_name)_data.json",
        experiment_name,
        3.0,
    )

    target_trajectories = multi_configs.target_trajectories
    horizon = multi_configs.horizon
    
    df = DataFrame(t=1:horizon)

    planner_types = ["greedy", "assignment", "formation"]
    for planner in planner_types
        solution = load_solution("$(root_path)/$(planner)/solution.json")
        # Make a dataframe
        # Get evaluation for all
        ppa_values = []

        for t in 1:horizon
            ppa_now = 0.0
            states = []
            # Would be nice to do this in a neater way
            for robot in solution.elements
                push!(states, robot[2][t])
            end
            targets = target_trajectories[t, :]    
            for state in states
                for target in targets
                    for face in target.faces
                        distance = (
                            face.pos[1] - state.state.x,
                            face.pos[2] - state.state.y,
                            target_height / 2 - drone_height,
                        )
                        theta = dirAngle(state.state.heading)
                        look_direction = (cos(theta), sin(theta), 0.0)
                        # Set previous_coverage to zero
                        ppa_now += compute_coverage_value(face, look_direction, distance, 0.0)
                    end
                end
            end
            push!(ppa_values, ppa_now)
        end

        df[!, planner] = ppa_values
    end
    return df
end
