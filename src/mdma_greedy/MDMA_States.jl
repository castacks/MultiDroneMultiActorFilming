using Test
using LinearAlgebra
using SubmodularMaximization

export Target, ViewConeSensor, Face, rotMatrix, UAVState

# Sensor representing a cone of vision from a drone
# Has an FOV as well as a maximum distance.
struct ViewConeSensor
    fov::Float64 # Radians representing FOV
    cutoff::Float64 # Max Distance
end

const cardinaldir = Vector([:E, :NE, :N, :NW, :W, :SW, :S, :SE])

function dir_to_index(d::Symbol)
    if d in cardinaldir
        return findall(x->x==d, cardinaldir)[1]
    end
end

# Target Faces
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

# Target information
mutable struct Target
    x::Float64
    y::Float64
    heading::Float64
    apothem::Float64 # Distance from center to center of each face
    faces::Array{Face}
    nfaces::UInt32
    id::UInt32

    function Target(x::Number, y::Number, h::Number, a::Number, n::UInt32, id::UInt32)
        faces = Vector{Face}(undef, n)
        dphi = (2*pi)/n
        # Need to generate n faces with n normal vectors
        for i = 1:n
            norm = [cos(dphi*i + h); sin(dphi*i + h)]
            pos = a*norm
            f = Face(pos[1], pos[2], 0.1, 1., norm)
            faces[i] = f
        end
        new(x,y,h,a,faces, id)
    end
end

function Target(x::Number, y::Number, h::Number, id::Number)
    Target(x,y,h,1.0, UInt32(6), UInt32(id))
end

#  State struct for agents. Used specifically as part of the action space
struct UAVState
    x::Int64
    y::Int64
    heading::Symbol
    function UAVState(x::Integer, y::Integer, h::Symbol)
        h in cardinaldir || throw(ArgumentError("invalid cardinaldir: $h"))
        new(x,y,h)
    end
end


mutable struct ViewConeObservation
    n::Int64 # Number of actors detected
    distances::Vector{Float64} # Distances to actors
    faces::Vector{Face} # List of faces observed, not counting occlusions/etc
end


function drawTargets()
    f = Figure(resolution=(800,800))
    Axis(f[1,1], backgroundcolor="black")

    xs = LinRange(-10, 10, 20)
    ys = LinRange(-10, 10, 20)
    t = Target(5., 5., 0., 5., UInt32(6), 1)

end
