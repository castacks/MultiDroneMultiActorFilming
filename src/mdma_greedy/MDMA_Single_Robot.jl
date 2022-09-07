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
MDPState(state) = MDPState(state, 0, nothing)
MDPState(m::MDPState, s::State) = MDPState(s, m.depth + 1, m)

# State and trajectory objects for convenience
struct Grid
    width::Int64
    height::Int64
    depth::Int64
    states::Array{State,3}
    transition_matrix::SparseMatrixCSC{Float64,Int64}

    # We will precompute some of the large objects that we use frequently
    function Grid(width, height)
        x = new(width, height, 8, get_states(width, height),
                sparse([], [], Float64[], width*height, width*height)
                )
        # x.transition_matrix .= generate_transition_matrix(x)
        x
    end
end
get_states(g::Grid) = g.states
function get_states(width::Int64, height::Int64)
    states = Array{State,3}(undef, width, height, 8)
    for i in CartesianIndices(states)
        r = i[1]
        c = i[2]
        d = i[3]
        states[i] = UAVState(c,r,cardinaldir[d])
    end
    states
end

dims(g::Grid) = (g.width, g.height, g.depth)
num_states(g::Grid) = length(g.states)

# States in grid should not have x or y lower than 1 or more than width/height
state_to_index(g::Grid, s::MDPState) = (s.state.y, s.state.x, dir_to_index(s.state.heading))


abstract type AbstractSingleRobotProblem <: MDP{MDPState, State} end

struct SingleRobotMultiTargetViewCoverageProblem <: AbstractSingleRobotProblem
    grid::Grid
    sensor::ViewConeSensor
    horizon::Int64
    targets::Vector{Target}
    move_dist::Int64
    prior_trajectories::Vector{Vector{UAVState}}
    function SingleRobotMultiTargetViewCoverageProblem(grid::Grid,
                                                       sensor::ViewConeSensor,
                                                       horizon::Integer,
                                                       targets::Vector{Target},
                                                       move_dist::Integer,
                                                       prior_trajectories = Trajectory[])
        new(grid, sensor, horizon, targets, move_dist, prior_trajectories)
    end
end
get_states(model::SingleRobotMultiTargetViewCoverageProblem) = get_states(model.grid)
horizon(x::AbstractSingleRobotProblem) = x.horizon

function state_trajectory(state::MDPState)
    ret = Array{State}(undef, state.depth)

    ret[1] = state.state

    curr_state = state
    for ii = 2:state.depth
        curr_state = curr_state.prev
        ret[ii] = curr_state.state
    end

    ret
end

POMDPs.transition(model::AbstractSingleRobotProblem, state::UAVState, action::UAVState) = transition(model, MDPState(state), action)
function POMDPs.transition(model::AbstractSingleRobotProblem, state::MDPState, action::UAVState)

    nbors = neighbors(model.grid, state, model.move_dist)
    if action in nbors
        states = nbors
        num_neighbors = length(nbors)

        # Need to convert all neighbors to MDPStates
        MDPStates = Vector{MDPState}(undef, 0)
        for s in nbors
            push!(MDPStates, MDPState(state,s))
        end

        # map(x->MDPState(state,action), nbors)
        probs = Vector{Float64}(undef, num_neighbors)
        push!(MDPStates, state)
        push!(probs, 1.0)
        return SparseCat([MDPState(state, action)], [1.0])
    else
        states = nbors
        num_neighbors = length(nbors)

        # Need to convert all neighbors to MDPStates
        MDPStates = Vector{MDPState}(undef, 0)
        for s in nbors
            push!(MDPStates, MDPState(state,s))
        end

        # map(x->MDPState(state,action), nbors)
        probs = Vector{Float64}(undef, num_neighbors)
        push!(MDPStates, state)
        push!(probs, 1.0)
        return SparseCat(MDPStates, probs)
    end

end
function POMDPs.states(model::AbstractSingleRobotProblem)
    uavstates = get_states(model)
    mdpstates = Vector{MDPState}(undef, 0)
    for s in uavstates
        push!(mdpstates, MDPState(s))
    end
    mdpstates
end
function POMDPs.actions(model::AbstractSingleRobotProblem)
    get_states(model.grid)
end

function POMDPs.actions(model::AbstractSingleRobotProblem, state::MDPState)
    neighbors(model.grid, state.state, model.move_dist)
end

function POMDPs.stateindex(model::AbstractSingleRobotProblem, s::MDPState)
    cart = CartesianIndex(s.state.y, s.state.x, dir_to_index(s.state.heading))
    grid = model.grid
    d = dims(grid)
    lin = LinearIndices((1:d[1], 1:d[2], 1:d[3]))
    return lin[cart]
end
# POMDPs.stateindex(model::AbstractSingleRobotProblem, state::MDPState) = stateindex(model, state.state)
# function POMDPs.stateindex(model::AbstractSingleRobotProblem, state::UAVState)
#     state.x * model.grid.width + state.y*model.grid.height + findall(x->x==state.heading, cardinaldir)[1]
# end

function POMDPs.actionindex(model::AbstractSingleRobotProblem, state::UAVState)
    stateindex(model, MDPState(state))
end

function dist_check(x1::Number,y1::Number, x2::Number,y2::Number, d::Number)
    (x2-x1)^2 + (y2-y1)^2 <= d^2
end

neighbors(grid::Grid, mstate::MDPState, d::Integer) = neighbors(grid, mstate.state, d)
# Compute neighbors within a certain distance of the state in the grid
function neighbors(grid::Grid, state::State, d::Integer)::Vector{State}
    actions = Vector{State}(undef, 0)
    diff = d ÷ 2 + 1# Search within a square region around the object
    diff = Int64(diff)
    for x = state.x-diff:state.x+diff
        for y = state.y-diff:state.y+diff
            if dist_check(state.x, state.y, x,y, d) && in_bounds(grid,x,y)
                # for d in cardinaldir
                #     new = UAVState(x,y,d)
                #     if new != state
                #         push!(actions, new)
                #     end
                # end
                push!(actions, UAVState(x,y, ccw(state.heading)))
                push!(actions, UAVState(x,y, cw(state.heading)))
                # push!(actions, UAVState(x,y, state.heading))
                # if x != state.x && y != state.y
                push!(actions, state)
                # push!(actions, UAVState(x,y, state.heading))
                # end
            end
        end
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

function solve_single_robot(problem::AbstractSingleRobotProblem,
                            state::State)

    solver = ValueIterationSolver(max_iterations=5, belres=1e-6, verbose=true)
    policy = solve(solver, problem)

    action, info = action_info(policy, MDPState(state))
    print(action)
    print(info)
    return policy
end

function POMDPs.reward(model::AbstractSingleRobotProblem, state::MDPState, action::State)
    # Want to just give a reward value if you detect an object
    reward = 0
    for t in model.targets
        if detectTarget(action, t, model.sensor)
            reward += 5
        end
    end
    reward
end


POMDPs.discount(model::AbstractSingleRobotProblem) = 1.0

function isterminal(model::AbstractSingleRobotProblem, state::MDPState)
    state.depth == model.horizon
end

POMDPs.initialstate(model::AbstractSingleRobotProblem) = MDPState(UAVState(1,1,:S))


@testset "single_robot_planner" begin
    sensor = ViewConeSensor(pi/2, 3)
    targets = Vector{Target}(undef, 0)
    push!(targets, Target(1,2,0))
    push!(targets, Target(1,3,0))
    push!(targets, Target(2,3,0))

    grid = Grid(20,20)
    horizon = 30
    model = SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, targets, 3)
    # print(transition(model, MDPState(UAVState(1,1,:N)), UAVState(1,2,:N)))
    # print(transition(model, UAVState(1,1,:N), UAVState(1,2,:N))
    @test reward(model, MDPState(UAVState(1,1,:N)), UAVState(1,1,:N)) == 15
    targets[3] = Target(10,10,0)
    @test reward(model, MDPState(UAVState(1,1,:N)), UAVState(1,1,:N)) == 10
end
