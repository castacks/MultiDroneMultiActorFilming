using POMDPs
using POMDPModelTools
using POMDPPolicies
using POMDPSimulators
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays

export State, Trajectory, Grid, get_states, dims, num_states, MDPStates
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
    prev::Union{State, Nothing}
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

# function generate_transition_matrix(g::Grid)
#     rows = Int64[]
#     columns = Int64[]
#     weights = Float64[]

#     # Push uniform transition probabilities for each state
#     for state in get_states(g)
#         ns = neighbors(g, state)

#         source = state_to_index(g, state)
#         weight = 1 / length(ns)

#         for neighbor in ns
#             dest = state_to_index(g, neighbor)

#             push!(columns, source)
#             push!(rows, dest)
#             push!(weights, weight)
#         end
#     end

#     size = length(get_states(g))
#     sparse(rows, columns, weights, size, size)
# end

# function transition(model::SingleRobotMultiTargetViewCoverageProblem, )


abstract type AbstractSingleRobotProblem <: MDP{MDPState, State} end

struct SingleRobotMultiTargetViewCoverageProblem <: AbstractSingleRobotProblem
    grid::Grid
    sensor::ViewConeSensor
    horizon::Int64
    targets::Vector{Target}
    prior_trajectories::Vector{Vector{UAVState}}
    function SingleRobotMultiTargetViewCoverageProblem(grid::Grid,
                                                       sensor::ViewConeSensor,
                                                       horizon::Integer,
                                                       targets::Vector{Target},
                                                       prior_trajectories = Trajectory[])
        new(grid, sensor, horizon, targets, prior_trajectories)
    end
end
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
# const grid_size = 100
# grid = Grid(grid_size, grid_size)
# sensor = ViewConeSensor()

function POMDPs.actions(model::AbstractSingleRobotProblem)
    get_states(model.grid)
end

function POMDPs.actions(model::AbstractSingleRobotProblem, state::MDPState)
    neighbors(model.grid, state.state)
end

function actionindex(model::AbstractSingleRobotProblem, state::MDPState) end

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
                push!(actions, UAVState(x,y, ccw(state.heading)))
                push!(actions, UAVState(x,y, cw(state.heading)))
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
                            state::State;
                            n_iterations =
                                default_num_iterations[problem.horizon],
                            exploration_constant =
                                exploration_constant(problem.horizon))

    solver = ValueIterationSolver(max_iterations=100, belres=1e-6, verbose=true)
    policy = solve(solver, problem)

    action, info = action_info(policy, MDPState(state))
    tree = info[:tree]
end

function reward(model::AbstractSingleRobotProblem, state::MDPState, action::State)
    # Want to just give a reward value if you detect an object
    reward = 0
    for t in model.targets
        if detectTarget(action, t, ViewConeSensor(pi/2, 3))
            reward += 5
        end
    end
    reward
end


POMDPs.discount(model::AbstractSingleRobotProblem) = 1.0

function isterminal(model::AbstractSingleRobotProblem, state)
    state.depth == model.horizon
end


@testset "single_robot_planner" begin
    sensor = ViewConeSensor(pi/2, 3)
    targets = Vector{Target}(undef, 0)
    push!(targets, Target(1,2,0))
    push!(targets, Target(1,3,0))
    push!(targets, Target(2,3,0))

    grid = Grid(20,20)
    horizon = 30
    model = SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, targets)
    @test reward(model, MDPState(UAVState(1,1,:N)), UAVState(1,1,:N)) == 15
    targets[3] = Target(10,10,0)
    @test reward(model, MDPState(UAVState(1,1,:N)), UAVState(1,1,:N)) == 10
end
