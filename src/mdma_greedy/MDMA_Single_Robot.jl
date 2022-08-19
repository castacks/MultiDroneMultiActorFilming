using POMDPs
using POMDPModelTools
using POMDPPolicies
using POMDPSimulators
using DiscreteValueIteration
using SubmodularMaximization
using SparseArrays

# include("MDMA.jl")
# using MDMA

##################################################
## Override function solve_single_robot         ##
## Input: problem, state, n_iterations.etc      ##
## Output: Tuple:                               ##
## (action, trajectory, tree)                   ##
##################################################

## target_coverage.jl has analogs to MDMA_Detection.jl

const State = UAVState
const Trajectory = Vector{State}


# State and trajectory objects for convenience
struct Grid
    width::Int64
    height::Int64
    states::Array{State,2}
    transition_matrix::SparseMatrixCSC{Float64,Int64}

    # We will precompute some of the large objects that we use frequently
    function Grid(width, height)
        x = new(width, height, get_states(width, height),
                sparse([], [], Float64[], width*height, width*height)
                )
        x.transition_matrix .= generate_transition_matrix(x)

        x
    end
end
get_states(g::Grid) = g.states
dims(g::Grid) = (g.width, g.height)
num_states(g::Grid) = length(g.states)

function generate_transition_matrix(g::Grid)
    rows = Int64[]
    columns = Int64[]
    weights = Float64[]

    # Push uniform transition probabilities for each state
    for state in get_states(g)
        ns = neighbors(g, state)

        source = state_to_index(g, state)
        weight = 1 / length(ns)

        for neighbor in ns
            dest = state_to_index(g, neighbor)

            push!(columns, source)
            push!(rows, dest)
            push!(weights, weight)
        end
    end

    size = length(get_states(g))
    sparse(rows, columns, weights, size, size)
end

struct MDPState
    state::State
    depth::Int64
end
# Constructor for initial states
MDPState(state) = MDPState(state, 0)

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

# const grid_size = 100
# grid = Grid(grid_size, grid_size)
# sensor = ViewConeSensor()


function POMDPs.actions(model::AbstractSingleRobotProblem, state::UAVState)
    neighbors(model.grid, state.state)
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
