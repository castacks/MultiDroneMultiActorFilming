import JSON3
using Test
using SubmodularMaximization
using MDMA

export AssignmentPlanner, GreedyPlanner, FormationPlanner, run_experiment

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
    pitch = -0.3490655
    cutoff = 100.0
    sensor = MDMA.PinholeCameraModel(focal_length, resolution, lens_dim, 0.0, pitch, cutoff)
    # sensor = MDMA.ViewConeSensor(pi / 2, cutoff)
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
    cutoff = 10.0
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
    cutoff = 10.0

    # TODO: Add random angle to phase
    # sensor = MDMA.PinholeCameraModel(focal_length, resolution, lens_dim, 0.0, pitch, cutoff)
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
