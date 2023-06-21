import JSON3
using Test
using SubmodularMaximization
using MDMA
using Base.Threads

using DataFrames
using CSV
using Plots

export AssignmentPlanner, GreedyPlanner, FormationPlanner, run_all_experiments, ExperimentsConfig
export blender_render_all_experiments, evaluate_all_experiments

struct GreedyPlanner end
struct FormationPlanner end
struct AssignmentPlanner end

function run_experiment(
    experiment_name::String,
    path_to_experiments::String,
    _::GreedyPlanner,
)
    # Set model parameters
    focal_length = [20.0, 20.0]#mm
    resolution = [3840.0, 2160.0]
    lens_dim = [35.00, 24.00] #mm
    pitch = -0.3490655 # if you change this also change animate_cameras.py
    cutoff = 100.0
    # sensor = MDMA.PinholeCameraModel(focal_length, resolution, lens_dim, 0.0, pitch, cutoff)
    sensor = MDMA.ViewConeSensor(pi / 2, cutoff)
    move_dist = 3

    multi_configs = configs_from_file(
        "$(path_to_experiments)/$(experiment_name)/$(experiment_name)_data.json",
        experiment_name,
        move_dist,
    )

    multi_configs.sensor = sensor
    # model = MDMA.SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, target_trajectories, move_dist);
    robot_states = map(
        x -> MDMA.random_state(multi_configs.horizon, multi_configs.grid),
        1:multi_configs.num_robots,
    )
    multi_problem = MDMA.MultiRobotTargetCoverageProblem(robot_states, multi_configs)

    # Solving output
    println("Solving Solution for $(experiment_name) using GreedyPlanner")
    solution = solve_sequential(multi_problem)

    # Save the solution
    MDMA.save_solution(
        experiment_name,
        path_to_experiments,
        "greedy",
        solution,
        multi_configs,
    )
end

function run_experiment(
    experiment_name::String,
    path_to_experiments::String,
    _::AssignmentPlanner,
)
    cutoff = 100.0
    sensor = MDMA.ViewConeSensor(pi / 2, cutoff)
    move_dist = 3

    multi_configs = configs_from_file(
        "$(path_to_experiments)/$(experiment_name)/$(experiment_name)_data.json",
        experiment_name,
        move_dist,
    )
    multi_configs.sensor = sensor
    robot_states = map(
        x -> MDMA.random_state(multi_configs.horizon, multi_configs.grid),
        1:multi_configs.num_robots,
    )

    multi_problem = MDMA.MultiRobotTargetAssignmentProblem(robot_states, multi_configs)

    # Solving output
    println("Solving Solution for $(experiment_name) using AssignmentPlanner")
    solution = solve_sequential(multi_problem)

    # Save the solution
    MDMA.save_solution(
        experiment_name,
        path_to_experiments,
        "assignment",
        solution,
        multi_configs,
    )
end

function run_experiment(
    experiment_name::String,
    path_to_experiments::String,
    _::FormationPlanner,
)

    # Not actually used by this planner
    # Just using the exiting function configs_from_file
    # To extract some information we need
    move_dist = 3
    multi_configs = configs_from_file(
        "$(path_to_experiments)/$(experiment_name)/$(experiment_name)_data.json",
        experiment_name,
        move_dist,
    )

    # Initial States    
    # TODO: Random phase angle
    # Instead of starting states
    robot_states = map(
        x -> MDMA.random_state(multi_configs.horizon, multi_configs.grid),
        1:multi_configs.num_robots,
    )
    formation_configs = formation_from_multi(10.0, robot_states, multi_configs)

    println("Solving Solution for $(experiment_name) using FormationPlanner")
    solution = solve_formation(formation_configs)
    MDMA.save_solution(
        experiment_name,
        path_to_experiments,
        "formation",
        solution,
        multi_configs,
    )
end

struct ExperimentsConfig
    path_to_experiments::String
    experiments::Vector{String}
end


# Experiment Steps
#  - Generate $(experiment_name)_data.json using a blender call
#  - Run the experiment
#  - Render images
#  - Evaluate the solution and save to two csv's
# Experiment List
# ["cross_mix", "cluster", "four_split", "spreadout_group"]
function run_all_experiments(config::ExperimentsConfig)
    # mtime is what we will store
    Threads.@threads for experiment in config.experiments
        path = "$(config.path_to_experiments)/$(experiment)"
        planners = [FormationPlanner(), GreedyPlanner(), AssignmentPlanner()]
        # Generate experiment data
        println("Building Experiment Data")
        data_cmd = `blender --background $(path)/$(experiment).blend  --python $(config.path_to_experiments)/export_experiment_data.py`
        run(data_cmd)

        # Run the experiment
        println("Running Experiment $(experiment)")
        planner_type_to_string = x -> lowercase(chop(string(x), head=0, tail=9))
        for planner in planners
            pl_string = planner_type_to_string(planner)
            println("Running planner $(pl_string) for $(experiment)")
            run_experiment(experiment, config.path_to_experiments, planner)
            println("Planner $(pl_string) complete for $(experiment)")
        end

        # Render Images
        println("Blender Rendering for $(experiment)")
        render_cmd = `blender --background $(path)/$(experiment).blend --python $(config.path_to_experiments)/animate_cameras.py`
        run(render_cmd)
        println("Render Finish for $(experiment)")

        # Evaluate Experiments
        evaluate_all_experiments(config)
        println("Experiment $(experiment) complete!")
    end
end

function blender_render_all_experiments(config::ExperimentsConfig)
    Threads.@threads for experiment in config.experiments
        path = "$(config.path_to_experiments)/$(experiment)"
        println("Blender Rendering for $(experiment)")
        render_cmd = `blender --background $(path)/$(experiment).blend  --python $(config.path_to_experiments)/animate_cameras.py`
        run(render_cmd)
        println("Render Finish for $(experiment)")
    end
end
function evaluate_all_experiments(config::ExperimentsConfig)
    # mtime is what we will store
    for experiment in config.experiments
        path = "$(config.path_to_experiments)/$(experiment)"

        # Evaluate Experiments
        println("Evaluating PPA for $(experiment)")
        df_ppa = evaluate_solution(experiment, config.path_to_experiments, PPAEvaluation())
        CSV.write("$(path)/ppa_evaluation.csv", df_ppa)
        save_plot(df_ppa, path, "PPA", experiment)

        println("Evaluating Image for $(experiment)")
        df_image = evaluate_solution(experiment, config.path_to_experiments, ImageEvaluation())
        CSV.write("$(path)/image_evaluation.csv", df_image)
        save_plot(df_image, path, "Image", experiment)

        println("Experiment $(experiment) complete!")
    end
end
function save_plot(df::DataFrame, path::String, eval_kind::String, experiment_name::String)
    planners = ["formation","greedy","assignment"]
    pl = plot()
    for planner in planners
        plot!(pl, df[!, :t], df[!, Symbol(planner)], label=planner)
    end
    xlabel!(pl,"TimeStep")
    ylabel!(pl, "Evaluation ($(eval_kind))")
    title!(pl, "$(eval_kind) Evaluation for $(experiment_name)")
    savefig(pl, "$(path)/$(lowercase(eval_kind))_evaluation.png")
end
