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

const Trajectory = 
struct SingleRobotMultiTargetViewCoverageProblem
    grid::Grid
    sensor::ViewConeSensor
    horizon::Int64
    targets::Vector{Target}
    prior_trajectories::Vector{Vector{UAVState}}
    function SingleRobotMultiTargetViewCoverageProblem(grid, sensor, horizon, targets, prior_trajectories = )
end

close()

steps = 100
grid_size = 10
horizon = SubmodularMaximization.default_horizon
iterations = 1000

grid = Grid(grid_size, grid_size)
sensor = ViewConeSensor()




GridWorldState(x::Int64, y::Int64) = GridWorldState(x,y, false)
posequal(s1::, s2::GridWorldState) = s1.x == s2.x && s1.y == s2.y

mutable struct GridWorld <: MDP{GridWorldState, Symbol}
    size_x::Int64
    size_y::Int64
    reward_states::Vector{GridWorldState}
    reward_values::Vector{Float64}
    tprob::Float64
    discount_factor::Float64
end

# #we use key worded arguments so we can change any of the values we pass in 
# function GridWorld(;sx::Int64=100, # size_x
#                    sy::Int64=100, # size_y
#                    rs::Vector{GridWorldState}=[GridWorldState(4,3), GridWorldState(4,6), GridWorldState(9,3, true), GridWorldState(8,8, true)], # reward states
#                    rv::Vector{Float64}=rv = [-10.,-5,10,3], # reward values
#                    tp::Float64=0.7, # tprob
#                    discount_factor::Float64=0.9)
#     return GridWorld(sx, sy, rs, rv, tp, discount_factor)
# end


# function POMDPs.states(mdp::GridWorld)
#     s = GridWorldState[]

#     for d = 0:1, y = 1:mdp.size_y, x = 1:mdp.size_x
#         push!(s, GridWorldState(x,y,d))
#     end
#     return s
# end

# POMDPs.actions(mdp::GridWorld) = [:up, :down, :left, :right]

# # transition helpers
# function inbounds(mdp::GridWorld,x::Int64,y::Int64)
#     if 1 <= x <= mdp.size_x && 1 <= y <= mdp.size_y
#         return true
#     else
#         return false
#     end
# end

# inbounds(mdp::GridWorld, state::GridWorldState) = inbounds(mdp, state.x, state.y);

# function POMDPs.transition(mdp::GridWorld, state::GridWorldState, action::Symbol)
#     a = action
#     x = state.x
#     y = state.y

#     if state.done
#         return SparceCat([GridWorldState(x,y,true)], [1.0])
#     elseif state in mdp.reward_states
#         return SparseCat([GridWorldState(x,y,true)], [1.0])
#     end

#     neighbors = [
#         GridWorldState(x+1, y, false) # Right
#         GridWorldState(x-1, y, false) # Left
#         GridWorldState(x, y-1, false) # Down
#         GridWorldState(x, y+1, false) # Up
#     ]

#     targets = Dict(:right=>1, :left=>2, :down=>3, :up=>4) # See Performance Note below
#     target = targets[a]

#     probability = fill(0.0, 4)

#     if !inbounds(mdp, neighbors[target])
#         # If would transition out of bounds, stay in
#         # same cell with probability 1
#         return SparseCat([GridWorldState(x, y)], [1.0])
#     else
#         probability[target] = mdp.tprob

#         oob_count = sum(!inbounds(mdp, n) for n in neighbors) # number of out of bounds neighbors

#         new_probability = (1.0 - mdp.tprob)/(3-oob_count)

#         for i = 1:4 # do not include neighbor 5
#             if inbounds(mdp, neighbors[i]) && i != target
#                 probability[i] = new_probability
#             end
#         end
#     end

#     return SparseCat(neighbors, probability)

# end

# function POMDPs.reward(mdp::GridWorld, state::GridWorldState, action::Symbol, statep::GridWorldState)
#     if state.done
#         return 0.0
#     end
#     r = 0.0
#     n = length(mdp.reward_states)
#     for i = 1:n
#         if posequal(state, mdp.reward_states[i])
#             r += mdp.reward_values[i]
#         end
#     end
#     return r
# end

# POMDPs.discount(mdp::GridWorld) = mdp.discount_factor


# function POMDPs.stateindex(mdp::GridWorld, state::GridWorldState)
#     sd = Int(state.done + 1)
#     ci = CartesianIndices((mdp.size_x, mdp.size_y, 2))
#     return LinearIndices(ci)[state.x, state.y, sd]
# end

# function POMDPs.actionindex(mdp::GridWorld, act::Symbol)
#     if act==:up
#         return 1
#     elseif act==:down
#         return 2
#     elseif act==:left
#         return 3
#     elseif act==:right
#         return 4
#     end
#     error("Invalid GridWorld action: $act")
# end;

# POMDPs.isterminal(mdp::GridWorld, s::GridWorldState) = s.done
# POMDPs.initialstate(pomdp::GridWorld) = Deterministic(GridWorldState(1,1)) # TODO: define initialistate for states, not distributions?


# # mdp = GridWorld()
# # state_space = states(mdp)


# mdp = GridWorld()
# solver = ValueIterationSolver(max_iterations=100, belres=1e-3, verbose=true)
# policy = solve(solver, mdp)
