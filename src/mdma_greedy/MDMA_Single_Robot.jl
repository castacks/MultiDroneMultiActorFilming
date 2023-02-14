using POMDPs
using POMDPModelTools
using POMDPPolicies
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays

export get_states, dims, num_states, SingleRobotMultiTargetViewCoverageProblem, solve_single_robot, compute_path, reward

##################################################
## Override function solve_single_robot         ##
## Input: problem, state, n_iterations.etc      ##
## Output: Tuple:                               ##
## (action, trajectory)                         ##
##################################################

## target_coverage.jl has analogs to MDMA_Detection.jl


# Constructor for initial states
# TODO Update this

get_states(g::MDMA_Grid) = g.states
function get_states(width::Int64, height::Int64, horizon::Int64)
    states = Array{MDPState,4}(undef, width, height, 8, horizon)
    for i in CartesianIndices(states)
        r = i[1] # row
        c = i[2] # column
        d = i[3] # direction
        t = i[4] # time
        states[i] = MDPState(UAVState(c, r, cardinaldir[d]), t, horizon, nothing)
    end
    states
end

dims(g::MDMA_Grid) = (g.width, g.height, g.angle_divisions, g.horizon)
num_states(g::MDMA_Grid) = length(g.states)

# States in grid should not have x or y lower than 1 or more than width/height
# state_to_index(g::Grid, s::MDPState) = (s.state.y, s.state.x, dir_to_index(s.state.heading), s.depth)


abstract type AbstractSingleRobotProblem <: MDP{MDPState,MDPState} end

mutable struct SingleRobotMultiTargetViewCoverageProblem <: AbstractSingleRobotProblem
    grid::MDMA_Grid
    sensor::ViewConeSensor
    horizon::Int64
    target_trajectories::Array{Target,2}
    move_dist::Int64
    coverage_data::CoverageData
    initial_state::MDPState
    # Add default height of 7 meters

    function SingleRobotMultiTargetViewCoverageProblem(
        grid::MDMA_Grid,
        sensor::ViewConeSensor,
        horizon::Integer,
        target_trajectories::Array{Target,2},
        move_dist::Integer,
        coverage_data::CoverageData,
        initial_state::MDPState
        )

        new(grid, sensor, horizon, target_trajectories, move_dist, coverage_data, initial_state)
    end
end
get_states(model::SingleRobotMultiTargetViewCoverageProblem) = get_states(model.grid)
horizon(x::AbstractSingleRobotProblem) = x.horizon


# POMDPs.transition(model::AbstractSingleRobotProblem, state::UAVState, action::UAVState) = transition(model, MDPState(state), action)
function POMDPs.transition(model::AbstractSingleRobotProblem, state::MDPState, action::MDPState)

    nbors = neighbors(model, state, model.move_dist)
    if action in nbors
        return SparseCat([MDPState(state, action)], [1.0])
    else
        # TODO Change here
        return SparseCat([MDPState(state)], [1.0])
    end

end

function POMDPs.states(model::AbstractSingleRobotProblem)
    get_states(model.grid)
end

function POMDPs.actions(model::AbstractSingleRobotProblem)
    get_states(model.grid)
end

function POMDPs.actions(model::AbstractSingleRobotProblem, state::MDPState)
    neighbors(model, state, model.move_dist)
end

function POMDPs.stateindex(model::AbstractSingleRobotProblem, s::MDPState)
    cart = CartesianIndex(s.state.y, s.state.x, dir_to_index(s.state.heading), (model.horizon + 1) - s.depth)
    # cart = CartesianIndex(s.state.y, s.state.x, dir_to_index(s.state.heading), s.depth)
    grid = model.grid
    d = dims(grid)
    lin = LinearIndices((1:d[1], 1:d[2], 1:d[3], 1:d[4]))
    return lin[cart]
end

function POMDPs.actionindex(model::AbstractSingleRobotProblem, action::MDPState)
    stateindex(model, action)
end

function dist_check(x1::Number, y1::Number, x2::Number, y2::Number, d::Number)
    (x2 - x1)^2 + (y2 - y1)^2 <= d^2
end

# neighbors(grid::Grid, mstate::MDPState, d::Integer) = neighbors(grid, mstate.state, d)
# Compute neighbors within a certain distance of the state in the grid
function neighbors(model::AbstractSingleRobotProblem, state::MDPState, d::Integer)::Vector{MDPState}

    actions = Vector{MDPState}(undef, 0)
    diff = d ÷ 2 + 1# Search within a square region around the object
    diff = Int64(diff)
    grid = model.grid
    uavstate = state.state
    for x = uavstate.x-diff:uavstate.x+diff
        for y = uavstate.y-diff:uavstate.y+diff
            if dist_check(uavstate.x, uavstate.y, x, y, d) && in_bounds(grid, x, y)
                # TODO Change here
                if state.depth + 1 <= model.horizon
                    push!(actions, MDPState(state, UAVState(x, y, ccw(uavstate.heading))))
                    push!(actions, MDPState(state, UAVState(x, y, cw(uavstate.heading))))
                    push!(actions, MDPState(state, UAVState(x, y, uavstate.heading)))
                end
            end
        end
    end
    actions
end

in_bounds(grid::MDMA_Grid, x::Integer, y::Integer) = in_bounds(grid, UAVState(x, y, :N))
in_bounds(grid::MDMA_Grid, state::MDPState) = in_bounds(grid, state.state)
function in_bounds(grid::MDMA_Grid, state::State)
    if state.x > 0 && state.x <= grid.width
        if state.y > 0 && state.y <= grid.height
            return true
        end
    end
    false
end

# Produces the policy for a single robot
function solve_single_robot(problem::AbstractSingleRobotProblem)

    solver = ValueIterationSolver(max_iterations=90, belres=1e-6, verbose=true, include_Q=false)
    policy = solve(solver, problem)

    return policy
end


# This should depend on the prior observations as well as other plans from robots
function POMDPs.reward(model::SingleRobotMultiTargetViewCoverageProblem, state::MDPState, action::MDPState)
    coverage_data = model.coverage_data
    # Want to just give a reward value if you detect an object
    reward = 0
    time = action.depth
    targets = model.target_trajectories[time, :]
    for (target_id, target) in enumerate(targets)
        target_coverage = coverage_data[time, target_id]
        if detectTarget(action.state, target, model.sensor)
          if (target_coverage > 0)
              reward = 0
          else
              reward += 1
          end
        end
    end

    if (action.state.heading == state.state.heading)
        reward += 0.1
    end

    if (action.state.x == state.state.x) && (action.state.y == state.state.y)
        reward += 0.05
    end
    reward
end

# For reward only the action matters in this case
POMDPs.reward(model::SingleRobotMultiTargetViewCoverageProblem, action::MDPState) = POMDPs.reward(model, action, action)

reward = POMDPs.reward

# Get path from policy
function compute_path(model, policy, state)::Trajectory
    path = Vector{MDPState}(undef, 0)
    push!(path, state)
    for x in 2:model.horizon
        state = action(policy, state)
        push!(path, state)
    end
    path
end

# Don't need any discount
POMDPs.discount(model::AbstractSingleRobotProblem) = 1

# Mark states as terminal when they have reached the horizon
function POMDPs.isterminal(model::AbstractSingleRobotProblem, state::MDPState)
    state.depth == model.horizon
end

# Upper left
POMDPs.initialstate(model::AbstractSingleRobotProblem) = model.initial_state

# Generate basic trajectories
function generate_target_trajectories(grid::MDMA_Grid, horizon::Integer, initial::Vector{Target})::Array{Target,2}
    trajectory = Array{Target,2}(undef, horizon, length(initial))
    yend = 4
    x = grid.width - 5
    trajectory[1, :] = initial
    current = initial
    tid = 1
    for i = 2:horizon
        new = Vector{Target}(undef, 0)
        push!(new, Target(initial[1].x, initial[1].y, 0, tid))
        for t in current[2:end]
            tid += 1
            push!(new, Target(t.x - 0, t.y - 0.5, 0, tid))
        end
        current = new
        trajectory[i, :] = current
    end
    return trajectory
end

@testset "single_robot_planner" begin
    sensor = ViewConeSensor(pi / 2, 3)
    targets = Vector{Target}(undef, 0)
    push!(targets, Target(1, 2, 0, 1))
    push!(targets, Target(1, 3, 0, 2))
    push!(targets, Target(2, 3, 0, 3))

    horizon = 4
    grid = MDMA_Grid(20, 20, horizon)

    # initial_state = MDPState(UAVState(0,0,:S))
    # traj = generate_target_trajectories(grid, horizon, targets)
    # coverage = generate_empty_coverage_data()
    # model = SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, traj, 3, )

    # #TODO Fix tests
    # # @test reward(model, MDPState(UAVState(1, 1, :N), horizon), MDPState(UAVState(1, 1, :N), horizon)) == 15
    # targets[3] = Target(10, 10, 0, 3)
    # traj = generate_target_trajectories(grid, horizon, targets)
    # model = SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, traj, 3)
    # @test reward(model, MDPState(UAVState(1, 1, :N), horizon), MDPState(UAVState(1, 1, :N), horizon)) == 10
end
