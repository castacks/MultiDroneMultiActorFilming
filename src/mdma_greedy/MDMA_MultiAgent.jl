using POMDPs
using POMDPModelTools
using POMDPPolicies
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays


import SubmodularMaximization.solve_block
import SubmodularMaximization.objective

export MultiDroneMultiActorConfigs, MultiRobotTargetCoverageProblem
export generate_empty_coverage_data, compute_prior_coverage, solve_block, empty, objective

# Stores configuration variables for multi-robot target tracking
struct MultiDroneMultiActorConfigs
    num_robots::Int32
    target_trajectories::Array{Target,2}
    # Will need a grid for each robot
    grid::MDMA_Grid
    sensor::ViewConeSensor
    horizon::Int64
    move_dist::Float64

    # Configuration for the ValueIterationSolver solver
    solver_iterations::Int64
    belres::Float64

    # Initialization

    function MultiDroneMultiActorConfigs(;
        num_robots,
        target_trajectories,
        grid,
        sensor=ViewConeSensor,
        horizon,
        move_dist,
        solver_iterations,
        belres
    )
        new(num_robots, target_trajectories, grid, sensor, horizon, move_dist, solver_iterations, belres)
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
struct MultiRobotTargetCoverageProblem <: PartitionProblem{Tuple{Int64,Vector{MDMA.MDPState}}}
    # Target tracking problems are defined by vectors of robot states
    partition_matroid::Vector{MDPState} # Most recent version of states of robots
    coverage_data::CoverageData
    configs::MultiDroneMultiActorConfigs

end

function extract_single_robot_problems(model::MultiRobotTargetCoverageProblem, coverage::CoverageData)::Vector{SingleRobotMultiTargetViewCoverageProblem}
    problems = Vector{SingleRobotMultiTargetViewCoverageProblem}(undef)
    config = model.configs
    for initial_state in model.partition_matroid
        push!(MDMA.SingleRobotMultiTargetViewCoverageProblem(config.grid, config.sensor, config.horizon, config.target_trajectories, model.configs.move_dist, coverage, initial_state))
    end
    problems
end
# Construct a target coverage problem with configs.
function MultiRobotTargetCoverageProblem(robot_states::Vector{MDPState}, coverage_data::CoverageData,
    kwargs...)
    configs = MultiDroneMultiActorConfigs(; kwargs...)
    MultiRobotTargetCoverageProblem(robot_states, coverage_data, configs)
end

# This should later be replaced with PPA
function compute_coverage_value(is_covered::Bool, previous_coverage::Float64)::Float64
    if is_covered || (previous_coverage > 0)
        1
    else
        -1
    end
end

function empty(p::MultiRobotTargetCoverageProblem)
    Vector{MDPState}(undef, 0)
end


# Construct empty coverage data, which may be useful for testing
function generate_empty_coverage_data(configs::MultiDroneMultiActorConfigs)::CoverageData

    target_traj = configs.target_trajectories
    num_targets = length(target_traj[1, :])
    coverage_data = Array{Tuple{Target,Float64},2}(undef, configs.horizon, num_targets)

    # If a target is detected by at least one robot it is covered
    # loop over each trajectory and compute detections. Is that really the best?
    for time in 1:configs.horizon
        for (target_idx, target) in enumerate(target_traj[time, :])
            # Set coverage data to the coverage value and the target
            # Set to 0 for testing
            coverage_data[time, target_idx] = (target, 0)
        end
    end
    coverage_data
end

function compute_prior_coverage(configs::MultiDroneMultiActorConfigs, trajectories::Vector{Trajectory})::CoverageData

    # final_states = map(last, trajectories)

    # loop over targets and append to covered states
    # set coverage value for each target for now just 1 or 0
    target_traj = configs.target_trajectories
    num_targets = length(target_traj[1, :])

    # Mapping of which targets are covered through history
    # 2D array of tuples of targets and values where each row is a timestep
    # Use floats to store a "coverage value" which can be PPA.etc
    # For boolean detection we simply use 1 for covered and -1 for non-convere
    coverage_data = generate_empty_coverage_data(configs)

    # If a target is detected by at least one robot it is covered
    # loop over each trajectory and compute detections. Is that really the best?
    for current_robot_trajectory in trajectories
        for time in 1:configs.horizon
            robot_state = current_robot_trajectory[time]
            for (target_idx, target) in enumerate(target_traj[time, :])
                # Update coverage based on previous coverage values Need to
                # check if other robots have covered this target before
                # assigning new coverage
                (_, previous_coverage) = coverage_data[time, target_idx]
                is_covered = detectTarget(robot_state.state, target, configs.sensor)
                cval = compute_coverage_value(is_covered, previous_coverage)
                # Set coverage data to the coverage value and the target
                coverage_data[time, target_idx] = (target, cval)
            end
        end
    end
    coverage_data

    # use solve sequential
end

function get_state(p, robot_id)
    p.partition_matroid[robot_id]
end


# Block is robot id
# Selections are other agent trajectories
#

# function solve_block(p::MDMA.MultiRobotTargetCoverageProblem, block::Int64, selections::Vector{Tuple{Int64, Vector{MDMA.MDPState}}})
function solve_block(p::MDMA.MultiRobotTargetCoverageProblem, block::Integer,
    selections::Vector{Tuple{Int64,Vector{MDMA.MDPState}}})

    # Get configs
    configs = p.configs

    # Represent coverage as just a list of targets who are covered at a timestep
    # prior selections is a list of list of trajectories list of list of lists
    # of states this gets the most recent trajectory for each robot shape should
    # be (N, num_planning rounds, horizon)
    trajectories = map(last, selections)
    # trajectories::Vector{Vector{MDPState}} = map(x -> x[1], trajectories)

    # Could pass this to single robot planner
    # Robot state or target state marked as covered?
    # Reward for each target that is covered
    # Check if target is covered in reward
    # Will later have new number and old number of pixels
    covered_states = compute_prior_coverage(p.configs, trajectories)

    # Get the state of the robot?
    state = get_state(p, block)
    # state = last(trajectories[block])

    # This is why we passed prior stuff to single robot
    # Is this covered? Do a quick lookup
    single_problem = SingleRobotMultiTargetViewCoverageProblem(
        configs.grid,
        configs.sensor,
        configs.horizon,
        configs.target_trajectories,
        Int64(configs.move_dist),
        covered_states,
        state
    )


    # Submodular solver expects certain output. We send that here.
    policy = solve_single_robot(single_problem)

    solution_trajectory = compute_path(single_problem, policy, state)

    (block, solution_trajectory)
    # call solve_sequential on this
    # subtype PartitionProblem
    # also solve_myopic and rebeccas solver
    # also implement evaluate_solution
    # where we end up calling objective
end

# Can just call reward from single robot planner
# X the set of trajectories Vector(Int, Trajectory)
function objective(p::MultiRobotTargetCoverageProblem, X)::Float64
    configs = p.configs

    print("objective", typeof(X))
    # Sum reward across trajectory
    # Pass in the object POMDPS.reward will change
    # Check if target is already in list of covered targets
    # Sum over rewards for targets for trajectories

    sum_reward = 0
    for (robot_id, robot_trajectory) in X
        for state in robot_trajectory
            single_problem = SingleRobotMultiTargetViewCoverageProblem(
                configs.grid,
                configs.sensor,
                configs.horizon,
                configs.target_trajectories,
                configs.move_dist,
                p.coverage_data,
                state
            )
            sum_reward += reward(single_problem, state)

        end
        # This is why we passed prior stuff to single robot
        # Is this covered? Do a quick lookup
        # For the reward only the
    end

    # For PPA we use differences after and before
    sum_reward

end
