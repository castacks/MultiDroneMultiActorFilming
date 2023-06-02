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

export MultiRobotTargetAssignmentProblem

mutable struct MultiRobotTargetAssignmentProblem <:
               PartitionProblem{Tuple{Int64,Vector{MDMA.MDPState}}}
    # Target tracking problems are defined by vectors of robot states
    partition_matroid::Vector{MDPState} # Most recent version of states of robots
    configs::MultiDroneMultiActorConfigs
end

function MultiRobotTargetAssignmentProblem(robot_states::Vector{MDPState}, kwargs...)
    configs = MultiDroneMultiActorConfigs(; kwargs...)
    MultiRobotTargetAssignmentProblem(robot_states, configs)
end

function compute_assignments(robot_id, num_targets, num_robots)
    width = (num_targets / num_robots) < 1 ? 1 : Integer(floor(num_targets / num_robots))
    offset = (width * (robot_id - 1)) + 1
    assignments = fill(false, (1, num_targets))
    assignments[offset:offset+width-1] = fill(true, (1, width))
    return assignments
end

# Want assignment to evenly split robots amongst targets
# In two cases this is not possible (when not divisible)
# Case1: 
#  - num_targets/num_robots < 1.0 
#  - In this case we simply do not assign some targets coverage
# Case2:
#  - num_targets/num_robots > 1.0 
#  - In this case I'll just wrap the assignments
function get_assignments(robot_id::Integer, num_targets::Integer, num_robots::Integer)
    if (num_targets % num_robots != 0)
        return compute_assignments(robot_id % num_targets + 1, num_targets, num_robots)
    else
        return compute_assignments(robot_id, num_targets, num_robots)
    end
end

function solve_block(
    p::MDMA.MultiRobotTargetAssignmentProblem,
    block::Integer,
    selections::Vector{Tuple{Int64,Vector{MDMA.MDPState}}},
)

    println("Processing Robot ", block)
    # Get configs
    configs = p.configs

    trajectories = map(last, selections)

    targets = p.configs.target_trajectories[1, :]
    # Assignments should just be bools for each target_id
    assignments = get_assignments(block, length(targets), p.configs.num_robots)
    # Get the state of the robot?
    state = get_state(p, block)

    single_problem = SingleRobotTargetAssignmentProblem(
        configs.grid,
        configs.sensor,
        configs.horizon,
        configs.target_trajectories,
        Int64(configs.move_dist),
        vec(assignments),
        state,
    )


    # Submodular solver expects certain output. We send that here.
    policy = solve_single_robot(single_problem)

    solution_trajectory = compute_path(single_problem, policy, state)

    (block, solution_trajectory)
end

# Can just call reward from single robot planner
# X the set of trajectories Vector(Int, Trajectory)
function objective(p::MultiRobotTargetAssignmentProblem, X)::Float64
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
            targets = p.configs.target_trajectories[1, :]
            # Assignments should just be bools for each target_id
            assignments = get_assignments(robot_id, length(targets), p.configs.num_robots)
            single_problem = SingleRobotTargetAssignmentProblem(
                configs.grid,
                configs.sensor,
                configs.horizon,
                configs.target_trajectories,
                Int64(configs.move_dist),
                vec(assignments),
                state,
            )
            sum_reward += reward(single_problem, state)

            robot_reward = reward(single_problem, state)
        end
    end

    # For PPA we use differences after and before
    sum_reward

end

mutable struct SingleRobotTargetAssignmentProblem <: AbstractSingleRobotProblem
    grid::MDMA_Grid
    sensor::Camera
    horizon::Int64
    target_trajectories::Array{Target,2}
    move_dist::Int64
    assignments::Vector{Bool}
    initial_state::MDPState

    function SingleRobotTargetAssignmentProblem(
        grid::MDMA_Grid,
        sensor::Camera,
        horizon::Integer,
        target_trajectories::Array{Target,2},
        move_dist::Integer,
        assignments::Vector{Bool},
        initial_state::MDPState,
    )

        new(
            grid,
            sensor,
            horizon,
            target_trajectories,
            move_dist,
            assignments,
            initial_state,
        )
    end
end


# This should depend on the prior observations as well as other plans from robots
function POMDPs.reward(
    model::SingleRobotTargetAssignmentProblem,
    state::MDPState,
    action::MDPState,
)
    # Want to just give a reward value if you detect an object
    reward = 0
    time = action.depth
    targets = model.target_trajectories[time, :]
    for (target_id, t) in enumerate(targets)
        # Pixel density by face should be stored
        if (model.assignments[target_id])
            if detectTarget(action.state, t, model.sensor)
                for (f_id, face) in enumerate(t.faces)
                    face_normal = face.normal

                    distance = (
                        face.pos[1] - action.state.x,
                        face.pos[2] - action.state.y,
                        target_height / 2 - drone_height,
                    )
                    theta = dirAngle(action.state.heading)
                    heading = (cos(theta), sin(theta), 0.0)

                    alpha = 10
                    face_normal = face.normal

                    current_pixel_density =
                        alpha *
                        face.weight *
                        abs(dot(distance, heading)) *
                        -dot(distance, face_normal) *
                        isvisible(distance, face_normal) / norm(distance)^3

                    reward += current_pixel_density
                end
            end
        end
    end

    if (action.state.heading == state.state.heading)
        reward += 0.02
    end

    if (action.state.x == state.state.x) && (action.state.y == state.state.y)
        reward += 0.01
    end

    reward
end

POMDPs.reward(model::SingleRobotTargetAssignmentProblem, action::MDPState) =
    POMDPs.reward(model, action, action)
