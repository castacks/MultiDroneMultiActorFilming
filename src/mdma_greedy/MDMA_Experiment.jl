import JSON3
using Test
using SubmodularMaximization
using MDMA


export AssignmentPlanner, GreedyPlanner, FormationPlanner, run_experiment

struct GreedyPlanner end
struct FormationPlanner end
struct AssignmentPlanner end

function run_experiment(experiment_name::String, path_to_experiments::String, GreedyPlanner)
    # Set model parameters
    focal_length = [20., 20.]#mm
    resolution = [3840.0, 2160.0]
    lens_dim = [35.00, 24.00] #mm
    pitch = 0.
    cutoff = 10.
    # sensor = MDMA.PinholeCameraModel(focal_length, resolution, lens_dim, 0.0, pitch, cutoff)
    sensor = MDMA.ViewConeSensor(pi / 2, cutoff)
    move_dist = 3

    multi_configs = configs_from_file("$(path_to_experiments)/$(experiment_name)/$(experiment_name)_data.json", experiment_name, move_dist)
    multi_configs.sensor = sensor
    # model = MDMA.SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, target_trajectories, move_dist);
    robot_states = map(x -> MDMA.random_state(multi_configs.horizon, multi_configs.grid), 1:multi_configs.num_robots)
    robot_states = map(x -> MDMA.random_state(multi_configs.horizon, multi_configs.grid), 1:multi_configs.num_robots)
    multi_problem = MDMA.MultiRobotTargetCoverageProblem(robot_states, multi_configs)

    # Solving output
    println("Solving Solution for $(experiment_name)")
    solution = solve_sequential(multi_problem)

    # Save the solution
    MDMA.save_solution(experiment_name, path_to_experiments, solution, multi_configs)
end

function run_experiment(experiment_name::String, path_to_experiments::String, planner::AssignmentPlanner)
    # TODO Save solution
end

function run_experiment(experiment_name::String, path_to_experiments::String, planner::FormationPlanner)
    # TODO Save solution
end

function run_experiments(experiment_names::Vector{String}, path_to_experiments::String)
    # TODO Loop over all the names and run the experiments. If one fails continue
end
