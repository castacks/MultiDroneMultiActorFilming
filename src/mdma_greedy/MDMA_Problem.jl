module MDMA_States

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

struct Target
    x::Float64
    y::Float64
    heading::Float64
    apothem::Float64 # Distance from center to center of each face
    faces::Array{Face}
    nfaces::UInt32

    function Target(x::Number, y::Number, h::Number, a::Number, n::UInt32)
        faces = Array{Face}(undef, n)
        dphi = (2*pi)/n
        for i = 1:n
            norm = [cos(dphi*i + h); sin(dphi*i + h)]
            pos = a*norm
            f = Face(pos[1], pos[2], 0.1, 1., norm)
            faces[i] = f
        end
        new(x,y,h,a,faces)
    end
end

function Target(x::Float64, y::Float64, h::Float64)
    Target(x,y,h,0.0,Array{Face,1}(Face(0,0,[1;1], 0.1, 10)))
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
end
