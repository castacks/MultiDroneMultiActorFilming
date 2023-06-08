using Test
using LinearAlgebra
using SubmodularMaximization

export Camera,
    Target,
    ViewConeSensor,
    PinholeCameraModel,
    Face,
    rotMatrix,
    UAVState,
    drone_height,
    target_height,
    multiply_face_weights

# Sensor representing a cone of vision from a drone
# Has an FOV as well as a maximum distance.
struct ViewConeSensor
    fov::Float64 # Radians representing FOV
    cutoff::Float64 # Max Distance
end

# Sensor representing pinhole camera
struct PinholeCameraModel
    intrinsics::Matrix{Float64}
    extrinsics::Matrix{Float64}
    resolution::Vector{Float64}
    fov::Float64
    cutoff::Float64
    function PinholeCameraModel(
        focal_length::Vector{Float64},
        resolution::Vector{Float64},
        lens_dim::Vector{Float64},
        skew::Float64,
        pitch::Float64,
        cutoff::Float64,
    )
        # world units to pixel units
        focal_length[1] = resolution[1] / lens_dim[1] * focal_length[1]
        focal_length[2] = resolution[2] / lens_dim[2] * focal_length[2]
        principal_point_offset = resolution / 2

        intrinsics = [[focal_length[1], 0, 0] [skew, focal_length[2], 0] [
            principal_point_offset[1],
            principal_point_offset[2],
            1,
        ]] #[[focal_length[1], skew, principal_point_offset[1]] [0, focal_length[2], principal_point_offset[2]] [0, 0, 1]]

        # One matrix is flipping from world coordinate to drone coordinate
        # then from drone to camera coordinate
        # Drone -> camera redifine the axis
        # Rotation on y axis
        extrinsics = [[0, sin(pitch), cos(pitch)] [-1, 0, 0] [0, cos(pitch), -sin(pitch)]]

        # extrinsics = [[0, sin(pitch), cos(pitch)] [-1, 0, 0] [0, cos(pitch), -sin(pitch)]] #[[1, 0, 0] [0, cos(pitch), -sin(pitch)] [0, sin(pitch), cos(pitch)]]


        # extrinsics = [[0, 0, 1] [-1, 0, 0] [0, 1, 0]] #[[1, 0, 0] [0, cos(pitch), -sin(pitch)] [0, sin(pitch), cos(pitch)]]
        # extrinsics = [[1, 0, 0] [0, cos(pitch), -sin(pitch)] [0, sin(pitch), cos(pitch)]]

        fov = 2 * atan(resolution[1], 2 * focal_length[1])
        println("Fov: $(fov)")
        return new(intrinsics, extrinsics, resolution, fov, cutoff)
    end
end

Camera = Union{ViewConeSensor,PinholeCameraModel}

const drone_height::Float64 = 5. # meters
const target_height::Float64 = 0 # meters
const cardinaldir = Vector([:E, :NE, :N, :NW, :W, :SW, :S, :SE])

function dir_to_index(d::Symbol)
    if d in cardinaldir
        return findall(x -> x == d, cardinaldir)[1]
    end
end

# Target Faces
mutable struct Face
    normal::Vector{Float64} # 3d normal
    pos::Vector{Float64} # position
    size::Float64 # Size of face, total face area
    weight::Float64 # Observation Weight
    function Face(x::Float64, y::Float64, s::Float64, w::Float64, n::Vector{Float64})
        pos = [x; y]
        return new(n, pos, s, w)
    end
end


function rotMatrix(theta::Float64)
    [
        cos(theta) -sin(theta)
        sin(theta) cos(10)
    ]
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
        # n represents the number of total faces
        faces = Vector{Face}(undef, n)

        # Change in angle
        dphi = (2 * pi) / (n - 1)
        side_length = 2 * a * tan(dphi / 2)
        # Need to generate n faces with n normal vectors
        for i = 1:(n-1)
            # Account for heading, which is z rotation
            theta = (dphi * i) + h
            norm = [cos(theta); sin(theta); 0.0]
            pos = a * norm
            # Make sure faces are relative to Target position
            f = Face(x + pos[1], y + pos[2], side_length * target_height, 1.0, norm) # Possible weight 0.5*cos(theta+pi)+0.5
            # Set front faces to twice the weight
            # if i in 1:2 || i == n-1
            #     f.weight = 4.0
            # end
            faces[i] = f
        end

        # Adding top face
        norm = [0.0; 0.0; 1.0]
        # Top face should be in center of target
        # NOTE: Setting weight to zero as a test!
        f = Face(x, y, 3 * sqrt(3) * side_length^2 / 2, 0.0, norm)
        faces[n] = f


        new(Float64(x), Float64(y), h, a, faces, length(faces), id)
    end
end

function Target(x::Number, y::Number, h::Number, id::Number)
    Target(Float64(x), Float64(y), h, 1.0, UInt32(7), UInt32(id))
end

function multiply_face_weights(t::Target, weight::Number)::Target
    for f in t.faces
        f.weight *= weight
    end
    t
end
#  State struct for agents. Used specifically as part of the action space
struct UAVState
    x::Float64
    y::Float64
    heading::Symbol
    function UAVState(x::Float64, y::Float64, h::Symbol)
        h in cardinaldir || throw(ArgumentError("invalid cardinaldir: $h"))
        new(x, y, h)
    end
end

UAVState(x::Integer, y::Integer, h::Symbol) = UAVState(Float64(x), Float64(y), h)

mutable struct ViewConeObservation
    n::Int64 # Number of actors detected
    distances::Vector{Float64} # Distances to actors
    faces::Vector{Face} # List of faces observed, not counting occlusions/etc
end


function drawTargets()
    f = Figure(resolution = (800, 800))
    Axis(f[1, 1], backgroundcolor = "black")

    xs = LinRange(-10, 10, 20)
    ys = LinRange(-10, 10, 20)
    t = Target(5.0, 5.0, 0.0, 5.0, UInt32(6), 1)

end
