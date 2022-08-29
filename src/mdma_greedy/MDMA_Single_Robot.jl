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
const Trajectory = Vector{State}
struct MDPState
    state::State
    depth::Int64
end
# Constructor for initial states
MDPState(state) = MDPState(state, 0)

# State and trajectory objects for convenience
struct Grid
    width::Int64
    height::Int64
    states::Array{MDPState,2}
    transition_matrix::SparseMatrixCSC{Float64,Int64}

    # We will precompute some of the large objects that we use frequently
    function Grid(width, height)
        x = new(width, height, get_states(width, height),
                sparse([], [], Float64[], width*height, width*height)
                )
        # x.transition_matrix .= generate_transition_matrix(x)
        x
    end
end
get_states(g::Grid) = g.states
function get_states(width::Int64, height::Int64)
    states = Array{MDPState,3}(undef, width, height, 8)
    # for state in states
    #     println(state)
    # end
    for r=1:height
        for c=1:width
            for d=1:8
                states[r][c][d] = MDPState(UAVState(c,r,cardinaldir[d]), 0)
                println(c," ", r," ",d)
            end
        end
    end
    states
end
dims(g::Grid) = (g.width, g.height)
num_states(g::Grid) = length(g.states)

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


function POMDPs.actions(model::AbstractSingleRobotProblem, state::UAVState)
    neighbors(model.grid, state.state)
end



function dist_check(x1::Number,y1::Number, x2::Number,y2::Number, d::Number)
    (x2-x1)^2 + (y2-y1)^2 <= d^2
end

# Compute neighbors within a certain distance of the state in the grid
function neighbors(grid::Grid, state::State, d::Unsigned)::Vector{State}
    actions = Vector{State}(undef, 0)
    for x = state.x-d:state.x+d
        for y = state.y-d:state.y+d
            if dist_check(state.x, state.y, x,y, d) && inbounds(grid,x,y)
                push!(actions, UAVState(x,y, ccw(state.h)))
                push!(actions, UAVState(x,y, cw(state.h)))
            end
        end
    end
    actions
end

in_bounds(grid::Grid, x::Int64, y::Int64) = in_bounds(grid, UAVState(x,y, :N))
function in_bounds(grid::Grid, state::State)
    if state.x > 0 && state.x < grid.width
        if state.y > 0 && state.y < grid.height
            true
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
    trajectory = extract_trajectory(problem, tree, state)

    (
        action=action,
        trajectory=trajectory,
        tree=tree
    )
end

function actions(model::AbstractSingleRobotProblem, state::MDPState)
    neighbors(model.grid, state.state) ## Need to add the look directions
end

function POMDPs.states(model::AbstractSingleRobotProblem)
    println("providing all actions")
    get_states(model.grid)
end
POMDPs.discount(model::AbstractSingleRobotProblem) = 1.0

function isterminal(model::AbstractSingleRobotProblem, state)
    state.depth == model.horizon
end
