# This file contains shared type aliases between Multi Agent and Single Agent Code

using Random

export CoverageData
export Trajectory, MDPState, State, Sensor, MDMA_Grid, random_state
# This is a 2D array since the rows represent timestamps For example this is
# what the data looks like if you have a 4 timestep scene in which 0 means not
# covered and 1 means covered
# Coverage Data
# | Time | Target1 | Target2 | Target3 |
# |    1 | (T1,0)  | (T2,1)  | (T3,1)  |
# |    2 | (T1,0)  | (T2,1)  | (T3,1)  |
# |    3 | (T1,1)  | (T2,1)  | (T3,1)  |
# |    4 | (T1,1)  | (T2,1)  | (T3,1)  |
const CoverageData = Array{Tuple{Target,Float64},2}

# Single Agent Types
const State = UAVState
const Sensor = ViewConeSensor

struct MDPState
    state::State
    depth::Int64
    horizon::Int64
    prev::Union{MDPState,Nothing}
end

MDPState(state, horizon) = MDPState(state, 1, horizon, nothing)
MDPState(m::MDPState, s::State) = MDPState(s, m.depth + 1, m.horizon, nothing)
MDPState(m::MDPState) = MDPState(m.state, m.depth + 1, m.horizon, nothing)
MDPState(m::MDPState, a::MDPState) = MDPState(a.state, m.depth + 1, m.horizon, nothing)

const Trajectory = Vector{MDPState}

# State and trajectory objects for convenience
struct MDMA_Grid
    width::Int64
    height::Int64
    angle_divisions::Int64
    horizon::Int64
    states::Array{MDPState,4}

    # We will precompute some of the large objects that we use frequently
    function MDMA_Grid(width, height, horizon)
        x = new(width, height, 8, horizon, get_states(width, height, horizon))
        x
    end
end

function random_state(horizon, grid::MDMA_Grid)::MDPState
    rwidth = rand(1:grid.width)
    rheight = rand(1:grid.height)
    rdir = rand(cardinaldir)
    depth = 1

    MDPState(UAVState(rwidth, rheight, rdir), horizon)
end
