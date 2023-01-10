# This file contains shared type aliases between Multi Agent and Single Agent Code
export CoverageData
export Trajectory, MDPState, State, Sensor
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

const Trajectory = Vector{MDPState}
