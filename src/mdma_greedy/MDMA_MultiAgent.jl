using POMDPs
using POMDPModelTools
using POMDPPolicies
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays

export MultiDroneMultiActorConfigs, MultiRobotTargetCoverageProblem
export generate_empty_coverage_data, compute_prior_coverage

# Stores configuration variables for multi-robot target tracking
struct MultiDroneMultiActorConfigs
    # Will need a grid for each robot
    grid::MDMA_Grid
    sensor::ViewConeSensor
    horizon::Int64
    target_trajectories::Array{Target,2}
    move_dist::Float64

    # Configuration for the ValueIterationSolver solver
    solver_iterations::Int64
    belres::Float64

    function MultiDroneMultiActorConfigs(;
        grid,
        sensor=ViewConeSensor,
        horizon,
        target_trajectories,
        move_dist,
        solver_iterations,
        belres
    )

        new(grid, sensor, horizon, solver_iterations)
    end
end

# Note that inferred parameters can still be overriden, but we probably don't
# want to do so
# function MultiDroneMultiActorConfigs(num_robots;
#     kwargs...)
#     grid = Grid(num_robots=num_robots)
#     MultiDroneMultiActorConfigs(; grid=grid, kwargs...)
# end

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
    partition_matroid::Vector{MDPState}

    num_robots::UInt32

    target_trajectories::Array{Target,2}

    configs::MultiDroneMultiActorConfigs
end

# Construct a target coverage problem with configs.
function MultiRobotTargetCoverageProblem(robot_states::Vector{MDPState},
    target_trajectories::Vector{MDPState};
    kwargs...)
    configs = MultiDroneMultiActorConfigs(; kwargs...)

    MultiRobotTargetCoverageProblem(robot_states, target_trajectories, configs)
end

# This should later be replaced with PPA
function compute_coverage_value(is_covered::Bool)::Float64
    if is_covered
        1
    else
        -1
    end
end



# Construct empty coverage data, which may be useful for testing
function generate_empty_coverage_data(p::MultiRobotTargetCoverageProblem, trajectories::Vector{Trajectory})::CoverageData

    target_traj = p.configs.target_trajectories
    num_targets = length(target_traj[1, :])
    coverage_data = Array{Tuple{Target,Float64},2}(undef, p.configs.horizon, num_targets)

    # If a target is detected by at least one robot it is covered
    # loop over each trajectory and compute detections. Is that really the best?
    for r in 1:p.num_robots
        current_robot_trajectory = trajectories[r]
        for time in 1:p.configs.horizon
            robot_state = current_robot_trajectory[time]
            for (target_idx, target) in enumerate(target_traj[time, :])
                # Set coverage data to the coverage value and the target
                # Set to 0 for testing
                coverage_data[time, target_idx] = (target, 0)
            end
        end
    end
    coverage_data
end

function compute_prior_coverage(p::MultiRobotTargetCoverageProblem, trajectories::Vector{Trajectory})::CoverageData

    # final_states = map(last, trajectories)

    # loop over targets and append to covered states
    # set coverage value for each target for now just 1 or 0
    target_traj = p.configs.target_trajectories
    num_targets = length(target_traj[1, :])

    # Mapping of which targets are covered through history
    # 2D array of tuples of targets and values where each row is a timestep
    # Use floats to store a "coverage value" which can be PPA.etc
    # For boolean detection we simply use 1 for covered and -1 for non-convered
    coverage_data = Array{Tuple{Target,Float64},2}(undef, p.configs.horizon, num_targets)

    # If a target is detected by at least one robot it is covered
    # loop over each trajectory and compute detections. Is that really the best?
    for r in 1:p.num_robots
        current_robot_trajectory = trajectories[r]
        for time in 1:p.configs.horizon
            robot_state = current_robot_trajectory[time]
            for (target_idx, target) in enumerate(target_traj[time, :])
                is_covered = detectTarget(robot_state.state, target, p.configs.sensor)
                cval = compute_coverage_value(is_covered)
                # Set coverage data to the coverage value and the target
                coverage_data[time, target_idx] = (target, cval)
            end
        end
    end
    coverage_data
end

function get_state(p, robot_id)
    p.partition_matroid[robot_id]
end


# Block is robot id
# Selections are other agent trajectories
function solve_block(p::MultiRobotTargetCoverageProblem, robot_id::Integer,
    prior_selections::Vector)
    configs = p.configs


    # Represent coverage as just a list of targets who are covered at a timestep
    # prior selections is a list of list of trajectories list of list of lists
    # of states this gets the most recent trajectory for each robot shape should
    # be (N, num_planning rounds, horizon)
    trajectories = map(last, prior_selections)

    # Could pass this to single robot planner
    # Robot state or target state marked as covered?
    # Reward for each target that is covered
    # Check if target is covered in reward
    # Will later have new number and old number of pixels
    covered_states = compute_prior_coverage(p, trajectories)

    # This is why we passed prior stuff to single robot
    # Is this covered? Do a quick lookup
    problem = SingleRobotMultiTargetViewCoverageProblem(
        configs.grid,
        configs.sensor,
        configs.horizon,
        configs.target_trajectories,
        configs.move_dist
    )

    # Get the state of the robot?
    state = get_state(p, robot_id)

    # Submodular solver expects certain output. We send that here.
    policy = solve_single_robot(problem, state,
        n_iterations=configs.solver_iterations)

    solution_trajectory = compute_path(problem, policy, state)


    (robot_id, solution_trajectory)
    # call solve_sequential on this
    # subtype PartitionProblem
    # also solve_myopic and rebeccas solver
    # also implement evaluate_solution
    # where we end up calling objective
end

# Can just call reward from single robot planner
# X the set of trajectories Vector(Int, Trajectory)
function objective(p::MultiRobotTargetCoverageProblem, X)
    configs = p.configs

    trajectories = map(last, X)

    # Sum reward across trajectory
    # Pass in the object POMDPS.reward will change
    # Check if target is already in list of covered targest
    # Sum over rewards for targets for trajectories

    # For PPA we use differences after and before

end
