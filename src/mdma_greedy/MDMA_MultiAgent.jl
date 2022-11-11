using POMDPs
using POMDPModelTools
using POMDPPolicies
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays

export MultiDroneMultiActorConfigs, MultiDroneMultiActorProblem


State = MDPState

# Stores configuration variables for multi-robot target tracking
struct MultiDroneMultiActorConfigs
    grid::Grid
    sensor::ViewConeSensor
    horizon::Int64
    target_trajectories::Array{Target,2}
    move_dist::Float64

    # Configuration for the ValueIterationSolver solver
    solver_iterations::Int64
    belres::Float64

    function MultiDroneMultiActorConfigs(;
        grid,
        horizon=default_horizon,
        sensor=ViewConeSensor,
        solver_iterations,
        belres,
        default_num_iterations[horizon],
        robot_target_range_limit=Inf
    )

        new(grid, sensor, horizon, solver_iterations)
    end
end

# Constructor that can infer parameters based on the number of robots
#
# Note that inferred parameters can still be overriden, but we probably don't
# want to do so
# function MultiDroneMultiActorConfigs(num_robots;
#     kwargs...)
#     grid = Grid(num_robots=num_robots)
#     MultiDroneMultiActorConfigs(; grid=grid, kwargs...)
# end

# Copy constructor with overrides
# function MultiDroneMultiActorConfigs(configs::MultiDroneMultiActorConfigs;
#     kwargs...)

#     fields = fieldnames(MultiDroneMultiActorConfigs)
#     old_settings = Dict(field => getfield(configs, field) for field in fields)
#     settings = merge(old_settings, kwargs)

#     MultiDroneMultiActorConfigs(; settings...)
# end

# Solution elements consist of the robot index and the associated trajectory
# We construct solutions as such because the output may not have the same
# ordering as the robots
#
# Warning: this "problem" object may become invalid if any of the underlying
# objects change and may copy some but not of the inputs.
struct MultiRobotTargetCoverageProblem
    # Target tracking problems are defined by vectors of robot states
    partition_matroid::Vector{State}

    target_states::Vector{State}

    configs::MultiDroneMultiActorConfigs
end

# Construct a target coverage problem with configs.
function MultiRobotTargetCoverageProblem(robot_states::Vector{State},
    target_states::Vector{State};
    kwargs...)
    configs = MultiDroneMultiActorConfigs(; kwargs...)

    MultiRobotTargetCoverageProblem(robot_states, target_states, configs)
end

struct MultiDroneMultiActorProblem
    # Target tracking problems are defined by vectors of robot states
    partition_matroid::Vector{State}

    configs::MultiDroneMultiActorConfigs

    function MultiRobotTargetTrackingProblem(robot_states::Vector{State},
        configs::MultiDroneMultiActorConfigs)

    end
end

#

# Block is robot id
# Selections are other agent trajectories
function solve_block(p::MultiRobotTargetTrackingProblem, block::Integer,
    prior_selections::Vector)
    configs = p.configs

    trajectories = map(last, prior_selections)

    # Could pass this to single robot planner
    # Robot state or target state marked as covered?
    # Reward for each target that is covered
    # Check if target is covered in reward
    # Will later have new number and old number of pixels
    covered_states = compute_prior_coverage(p, prior_selections)

    # This is why we passed prior stuff to single robot
    # Is this covered? Do a quick lookup.
    problem = SingleRobotMultiTargetViewCoverageProblem(configs.grid, configs.sensor,
        configs.horizon,
        configs.target_trajectories, configs.move_dist
    )

    # Get the state of the robot?
    state = get_state(p, block)

    # Submodular solver expects certain output. We send that here.
    solution = solve_single_robot(problem, state,
        n_iterations=configs.solver_iterations)

    (block, solution.trajectory)
    # call solve_sequential on this
    # subtype PartitionProblem
    # also solve_myopic and rebeccas solver
    # also implement evaluate_solution
    # where we end up calling objective
end

# Can just call reward from single robot planner
# X the set of trajectories Vector(Int, Trajectory)
function objective(p::MultiRobotTargetTrackingProblem, X)
    configs = p.configs

    trajectories = map(last, X)

    # Sum reward across trajectory

end
