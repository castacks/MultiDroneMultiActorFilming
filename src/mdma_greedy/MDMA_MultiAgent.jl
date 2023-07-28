using POMDPs
using POMDPModelTools
using Base: parsed_toml
using POMDPPolicies
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays
using LinearAlgebra


import SubmodularMaximization.solve_block
import SubmodularMaximization.objective

export MultiDroneMultiActorConfigs, MultiRobotTargetCoverageProblem
export generate_empty_coverage_data,
    compute_prior_coverage, solve_block, empty, objective, compute_coverage_value


# Stores configuration variables for multi-robot target tracking
mutable struct MultiDroneMultiActorConfigs
    experiment_name::String
    num_robots::Int64
    target_trajectories::Array{Target,2}
    # Will need a grid for each robot
    grid::MDMA_Grid
    sensor::Camera
    horizon::Int64
    move_dist::Float64


    # Initialization

    function MultiDroneMultiActorConfigs(;
        experiment_name,
        num_robots,
        target_trajectories,
        grid,
        sensor,
        horizon,
        move_dist,
    )
        new(
            experiment_name,
            num_robots,
            target_trajectories,
            grid,
            sensor,
            horizon,
            move_dist,
        )
    end
end


# Note that inferred parameters can still be overriden, but we probably don't
# want to do so

# Solution elements consist of the robot index and the associated trajectory
# We construct solutions as such because the output may not have the same
# ordering as the robots
#
# Warning: this "problem" object may become invalid if any of the underlying
# objects change and may copy some but not of the inputs.
mutable struct MultiRobotTargetCoverageProblem <:
               PartitionProblem{Tuple{Int64,Vector{MDMA.MDPState}}}
    # Target tracking problems are defined by vectors of robot states
    partition_matroid::Vector{MDPState} # Most recent version of states of robots
    configs::MultiDroneMultiActorConfigs

end

function extract_single_robot_problems(
    model::MultiRobotTargetCoverageProblem,
    coverage::CoverageData,
)::Vector{SingleRobotMultiTargetViewCoverageProblem}
    problems = Vector{SingleRobotMultiTargetViewCoverageProblem}(undef)
    config = model.configs
    for initial_state in model.partition_matroid
        push!(
            MDMA.SingleRobotMultiTargetViewCoverageProblem(
                config.grid,
                config.sensor,
                config.horizon,
                config.target_trajectories,
                model.configs.move_dist,
                coverage,
                initial_state,
            ),
        )
    end
    problems
end
# Construct a target coverage problem with configs.
function MultiRobotTargetCoverageProblem(robot_states::Vector{MDPState}, kwargs...)
    configs = MultiDroneMultiActorConfigs(; kwargs...)
    MultiRobotTargetCoverageProblem(robot_states, configs)
end

# This should later be replaced with PPA
function compute_coverage_value(
    face::Face,
    heading::Tuple{Float64,Float64,Float64},
    distance::Tuple{Float64,Float64,Float64},
    previous_coverage::Float64,
)::Float64
    # update

    alpha = 1
    face_normal = face.normal

    # get pixel density
    prior_pixel_density = previous_coverage

    current_pixel_density =
        alpha *
        face.weight *
        abs(dot(distance, heading)) *
        -dot(distance, face_normal) *
        isvisible(distance, face_normal) / norm(distance)^3

    # Sum pixel density
    current_pixel_density + prior_pixel_density
end

function empty(p::MultiRobotTargetCoverageProblem)
    Vector{MDPState}(undef, 0)
end


# Construct empty coverage data, which may be useful for testing
function generate_empty_coverage_data(configs::MultiDroneMultiActorConfigs)::CoverageData

    target_traj = configs.target_trajectories
    num_targets = length(target_traj[1, :])
    num_faces = 7 # TODO Do not make this hardcoded! It should be 6 (for hexagon) and 1 normal on top
    coverage_data = Array{Float64,3}(undef, configs.horizon, num_targets, num_faces)

    # If a target is detected by at least one robot it is covered
    # loop over each trajectory and compute detections. Is that really the best?
    for time = 1:configs.horizon
        for (target_idx, target) in enumerate(target_traj[time, :])
            # Set coverage data to the coverage value for each face
            for (f_id, _) in enumerate(target.faces)
                coverage_data[time, target_idx, f_id] = 0
            end
        end
    end
    coverage_data
end

function compute_prior_coverage(
    configs::MultiDroneMultiActorConfigs,
    trajectories::Vector{Trajectory},
)::CoverageData

    # final_states = map(last, trajectories)

    # loop over targets and append to covered states
    # set coverage value for each target for now just 1 or 0
    target_traj = configs.target_trajectories

    # Mapping of which targets are covered through history
    # 2D array of tuples of targets and values where each row is a timestep
    # Use floats to store a "coverage value" which can be PPA.etc
    # For boolean detection we simply use 1 for covered and -1 for non-convere
    coverage_data = generate_empty_coverage_data(configs)

    # If a target is detected by at least one robot it is covered
    # loop over each trajectory and compute detections. Is that really the best?
    for current_robot_trajectory in trajectories
        for time = 1:configs.horizon
            robot_state = current_robot_trajectory[time]
            for (target_idx, target) in enumerate(target_traj[time, :])
                if detectTarget(robot_state.state, target, configs.sensor)
                    for (f_idx, face) in enumerate(target.faces)
                        previous_coverage = coverage_data[time, target_idx, f_idx]
                        distance = (
                            face.pos[1] - robot_state.state.x,
                            face.pos[2] - robot_state.state.y,
                            target_height / 2 - drone_height,
                        )
                        theta = dirAngle(robot_state.state.heading)
                        heading = (cos(theta), sin(theta), 0.0)
                        num_pixels = compute_coverage_value(
                            face,
                            heading,
                            distance,
                            previous_coverage,
                        )
                        coverage_data[time, target_idx, f_idx] = num_pixels
                    end
                end
                # Update coverage based on previous coverage values Need to
                # check if other robots have covered this target before
                # assigning new coverage
                # compute current number of pixels on this face
                # Set coverage data to the coverage value and the target
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
function solve_block(
    p::MDMA.MultiRobotTargetCoverageProblem,
    block::Integer,
    selections::Vector{Tuple{Int64,Vector{MDMA.MDPState}}},
)


    println("Processing Robot ", block)
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
    # TODO coverage should be coming of of multirobot problem
    single_problem = SingleRobotMultiTargetViewCoverageProblem(
        configs.grid,
        configs.sensor,
        configs.horizon,
        configs.target_trajectories,
        Int64(configs.move_dist),
        covered_states,
        state,
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

    # Sum reward across trajectory
    # Pass in the object POMDPS.reward will change
    # Check if target is already in list of covered targets
    # Sum over rewards for targets for trajectories


    # TODO CoverageData object for each robot
    # At the moment the coverage data is set to empty,
    # So the reward will not matsc
    sum_reward = 0
    for (robot_id, robot_trajectory) in X
        robot_reward = 0
        for state in robot_trajectory
            single_problem = SingleRobotMultiTargetViewCoverageProblem(
                configs.grid,
                configs.sensor,
                configs.horizon,
                configs.target_trajectories,
                Int64(configs.move_dist),
                generate_empty_coverage_data(p.configs),
                state,
            )
            sum_reward += reward(single_problem, state)

            robot_reward = reward(single_problem, state)
        end
        # println("Robot id: $(robot_id), Reward: $(robot_reward)")
        # This is why we passed prior stuff to single robot
        # Is this covered? Do a quick lookup
        # For the reward only the
    end

    # For PPA we use differences after and before
    sum_reward

end
