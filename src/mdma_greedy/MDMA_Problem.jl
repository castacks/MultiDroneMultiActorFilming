using Test
using LinearAlgebra

# Sensor representing a cone of vision from a drone
# Has an FOV as well as a maximum distance.
struct ViewConeSensor
    fov::Float64 # Radians representing FOV
    cutoff::Float64 # Max Distance
end

const cardinaldir = Set([:E, :NE, :N, :NW, :W, :SW, :S, :SE])

struct Face
    normal::Vector{Float64} # 2d normal
    pos::Vector{Float64} # position
    size::Float64 # Size of face, total face length
    weight::Float64 # Observation Weight
    function Face(x::Float64, y::Float64, s::Float64, w::Float64, n::Vector{Float64})
        pos = [x; y]
        return new(n,pos,s,w)
    end
end


function rotMatrix(theta::Float64)
    [cos(theta) -sin(theta);
     sin(theta) cos(10)]
end

struct TargetState
    x::Float64
    y::Float64
    heading::Float64
    apothem::Float64
    faces::Array{Face}
    nfaces::UInt32

    function TargetState(x::Number, y::Number, h::Number, a::Number, n::UInt32)
        faces = Array{Face}[]
        dphi = pi/n
        for i = 1:n
            r_mat = rotMatrix(dphi*i)
            norm = [1;0]
            norm = r_mat*norm
            println(norm, dphi*i * 180/pi)
        end

        new(x,y,h,a,faces)
    end
end

function TargetState(x::Float64, y::Float64, h::Float64)
    TargetState(x,y,h,0.0,Array{Face,1}(Face(0,0,[1;1], 0.1, 10)))
end

mutable struct UAVState
    x::Int64
    y::Int64
    done::Bool
    heading::Symbol
    sensor::ViewConeSensor
    function UAVState(x::Int64, y::Int64, h::Symbol, s::ViewConeSensor)
        h in cardinaldir || throw(ArgumentError("invalid cardinaldir: $h"))
        new(x,y,h,s)
    end
end


struct ViewConeObservation
    n::Int64 # Number of actors detected
    distances::Vector{Float64} # Distances to actors
    faces::Vector{Face} # List of faces observed, not counting occlusions/etc
end



# @testset "Targets" begin
    
#     tstate = TargetState(0,0,0, 1, )
# end
