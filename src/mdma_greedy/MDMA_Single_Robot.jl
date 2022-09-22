using POMDPs
using POMDPModelTools
using POMDPPolicies
using POMDPSimulators
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays

export State, Trajectory, Grid, get_states, dims, num_states, MDPState
# include("MDMA.jl")
# using MDMA

##################################################
## Override function solve_single_robot         ##
## Input: problem, state, n_iterations.etc      ##
## Output: Tuple:                               ##
## (action, trajectory)                         ##
##################################################

## target_coverage.jl has analogs to MDMA_Detection.jl

const State = UAVState
const Sensor = ViewConeSensor
const Trajectory = Vector{State}
struct MDPState
    state::State
    depth::Int64
    prev::Union{MDPState, Nothing}
end
# Constructor for initial states
MDPState(state) = MDPState(state, 1, nothing)
MDPState(m::MDPState, s::State) = MDPState(s, m.depth + 1, nothing)
MDPState(m::MDPState) = MDPState(m.state, m.depth+1, nothing)
MDPState(m::MDPState, a::MDPState) = MDPState(a.state, m.depth + 1, nothing)

# State and trajectory objects for convenience
struct Grid
    width::Int64
    height::Int64
    angle_divisions::Int64
    horizon::Int64
    states::Array{MDPState,4}

    # We will precompute some of the large objects that we use frequently
    function Grid(width, height, horizon)
        x = new(width, height, 8, horizon, get_states(width, height, horizon))
        x
    end
end
get_states(g::Grid) = g.states
function get_states(width::Int64, height::Int64, horizon::Int64)
    states = Array{MDPState,4}(undef, width, height, 8, horizon)
    for i in CartesianIndices(states)
        r = i[1]
        c = i[2]
        d = i[3]
        t = i[4]
        states[i] = MDPState(UAVState(c,r,cardinaldir[d]), t, nothing)
    end
    states
end

dims(g::Grid) = (g.width, g.height, g.angle_divisions, g.horizon)
num_states(g::Grid) = length(g.states)

# States in grid should not have x or y lower than 1 or more than width/height
state_to_index(g::Grid, s::MDPState) = (s.state.y, s.state.x, dir_to_index(s.state.heading), s.depth)


abstract type AbstractSingleRobotProblem <: MDP{MDPState, MDPState} end

struct SingleRobotMultiTargetViewCoverageProblem <: AbstractSingleRobotProblem
    grid::Grid
    sensor::ViewConeSensor
    horizon::Int64
    intial_targets::Vector{Target}
    target_trajectories::Array{Target, 2}
    move_dist::Int64
    prior_trajectories::Vector{Vector{UAVState}}
    function SingleRobotMultiTargetViewCoverageProblem(grid::Grid,
                                                       sensor::ViewConeSensor,
                                                       horizon::Integer,
                                                       targets::Vector{Target},
                                                       move_dist::Integer,
                                                       prior_trajectories = Trajectory[])
        target_trajectories = generate_target_trajectories(grid, horizon, targets)
        new(grid, sensor, horizon, targets, target_trajectories, move_dist, prior_trajectories)
    end
end
get_states(model::SingleRobotMultiTargetViewCoverageProblem) = get_states(model.grid)
horizon(x::AbstractSingleRobotProblem) = x.horizon


# POMDPs.transition(model::AbstractSingleRobotProblem, state::UAVState, action::UAVState) = transition(model, MDPState(state), action)
function POMDPs.transition(model::AbstractSingleRobotProblem, state::MDPState, action::MDPState)

    time = state.depth
    nbors = neighbors(model, state, model.move_dist)
    if action in nbors
        return SparseCat([MDPState(state, action)], [1.0])
    else
        return SparseCat([MDPState(state.state, state.depth + 1, nothing)], [1.0])
    end

end


function POMDPs.states(model::AbstractSingleRobotProblem)
    get_states(model)
end

function POMDPs.actions(model::AbstractSingleRobotProblem)
    get_states(model.grid)
end

function POMDPs.actions(model::AbstractSingleRobotProblem, state::MDPState)
    neighbors(model, state, model.move_dist)
end

function POMDPs.stateindex(model::AbstractSingleRobotProblem, s::MDPState)
    cart = CartesianIndex(s.state.y, s.state.x, dir_to_index(s.state.heading), s.depth)
    grid = model.grid
    d = dims(grid)
    lin = LinearIndices((1:d[1], 1:d[2], 1:d[3], 1:d[4]))
    return lin[cart]
end
# POMDPs.stateindex(model::AbstractSingleRobotProblem, state::MDPState) = stateindex(model, state.state)
# function POMDPs.stateindex(model::AbstractSingleRobotProblem, state::UAVState)
#     state.x * model.grid.width + state.y*model.grid.height + findall(x->x==state.heading, cardinaldir)[1]
# end

function POMDPs.actionindex(model::AbstractSingleRobotProblem, action::MDPState)
    stateindex(model, action)
end

function dist_check(x1::Number,y1::Number, x2::Number,y2::Number, d::Number)
    (x2-x1)^2 + (y2-y1)^2 <= d^2
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
            if dist_check(uavstate.x, uavstate.y, x,y, d) && in_bounds(grid,x,y)
                if state.depth + 1 <= model.horizon
                  push!(actions, MDPState(state, UAVState(x,y, ccw(uavstate.heading))))
                  push!(actions, MDPState(state, UAVState(x,y, cw(uavstate.heading))))

                end
            end
        end
    end
    if state.depth + 1 <= model.horizon
        push!(actions, MDPState(state))
    end
    actions
end

in_bounds(grid::Grid, x::Integer, y::Integer) = in_bounds(grid, UAVState(x,y, :N))
in_bounds(grid::Grid, state::MDPState) = in_bounds(grid, state.state)
function in_bounds(grid::Grid, state::State)
    if state.x > 0 && state.x <= grid.width
        if state.y > 0 && state.y <= grid.height
            return true
        end
    end
    false
end
# Define neighbors and take cartesian product with directions
# Make neighbors within a distance of grid
# Have used a sparsematrix to represent transitions in prior work

function solve_single_robot(problem::AbstractSingleRobotProblem)

    solver = ValueIterationSolver(max_iterations=90, belres=1e-6, verbose=true)
    policy = solve(solver, problem)

    # action, info = action_info(policy, MDPState(state))
    # print(action)
    # print(info)
    return policy
end

function POMDPs.reward(model::AbstractSingleRobotProblem, state::MDPState, action::MDPState)
    # Want to just give a reward value if you detect an object
    reward = 0
    time = action.depth
    # println("r ", time)
    targets = model.target_trajectories[time,:]
    # for targets in eachrow(model.target_trajectories)
    for t in targets
        if detectTarget(action.state, t, model.sensor)
            reward += 5
        end
    end
    # end
    reward
end


POMDPs.discount(model::AbstractSingleRobotProblem) = 0.9

function isterminal(model::AbstractSingleRobotProblem, state::MDPState)
    state.depth == model.horizon
end

POMDPs.initialstate(model::AbstractSingleRobotProblem) = MDPState(UAVState(1,1,:S))

function generate_target_trajectories(grid::Grid, horizon::Integer, initial::Vector{Target})::Array{Target,2}
    trajectory = Array{Target, 2}(undef, horizon, length(initial))
    yend = 4
    x = grid.width - 5
    trajectory[1,:] = initial
    current = initial
    for i = 2:horizon
        new = Vector{Target}(undef,0)
        for t in current
            push!(new, Target(t.x,t.y+(-0.5), 0))
        end
        current = new
        trajectory[i,:] = current
    end
    return trajectory
end

@testset "single_robot_planner" begin
    sensor = ViewConeSensor(pi/2, 3)
    targets = Vector{Target}(undef, 0)
    push!(targets, Target(1,2,0))
    push!(targets, Target(1,3,0))
    push!(targets, Target(2,3,0))

    horizon = 4
    grid = Grid(20,20, horizon)
    model = SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, targets, 3)

    # print(model.target_trajectories)
    # print(transition(model, MDPState(UAVState(1,1,:N)), UAVState(1,2,:N)))
    # print(transition(model, UAVState(1,1,:N), UAVState(1,2,:N))
    # println("test ", reward(model, MDPState(UAVState(1,1,:N)), UAVState(1,1,:N)))
    @test reward(model, MDPState(UAVState(1,1,:N)), MDPState(UAVState(1,1,:N))) == 15
    targets[3] = Target(10,10,0)
    model = SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, targets, 3)
    @test reward(model, MDPState(UAVState(1,1,:N)), MDPState(UAVState(1,1,:N))) == 10
end
