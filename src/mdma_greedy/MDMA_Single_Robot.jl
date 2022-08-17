using POMDPs
using POMDPModelTools
using POMDPPolicies
using POMDPSimulators
using DiscreteValueIteration
using SubmodularMaximization


##################################################
## Override function solve_single_robot         ##
## Input: problem, state, n_iterations.etc      ##
## Output: Tuple:                               ##
## (action, trajectory, tree)                   ##
##################################################

## target_coverage.jl has analogs to MDMA_Detection.jl

struct SingleRobotMultiTargetViewCoverageProblem <: AbstractSingleRobotProblem
    grid::Grid
    sensor::ViewConeSensor
    horizon::Int64
    targets::Vector{Target}
    prior_trajectories::Vector{Vector{UAVState}}
    function SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, targets, prior_trajectories = )
end

const State = Tuple(Int64, Int64, Symbol) # Override State definition to include facing direction
const Trajectory = Vector{State}

grid = Grid(grid_size, grid_size)
sensor = ViewConeSensor()


function POMDPs.actions(model::AbstractSingleRobotProblem, state::UAVState)
    neighbors(model.grid, state.state)
end

