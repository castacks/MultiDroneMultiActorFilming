struct FormationPlannerConfigs
    radius::AbstractFloat
    point_nearest::Bool
    initial_states::Vector{MDPState}
    grid::MDMA_Grid
    sensor::Camera
    target_trajectories::Array{Target,2}
    num_robots::Integer
    horizon::Integer
end

function formation_from_multi(radius::AbstractFloat, initial_states::Vector{MDPState}, mconfig::MultiDroneMultiActorConfigs)::FormationPlannerConfigs
    return FormationPlannerConfigs(
        radius,
        true,
        initial_states,
        mconfig.grid,
        mconfig.sensor,
        mconfig.target_trajectories,
        mconfig.num_robots,
        mconfig.horizon,
    )
end

function target_centroid(targets::Vector{Target})::Tuple{AbstractFloat, AbstractFloat}
    N = length(targets)
    x_sum = 0.0
    y_sum = 0.0
    for t in targets
        x_sum += t.x
        y_sum += t.y
    end
    centroid = (x_sum/N, y_sum/N)
    centroid
end

function heading_from_angle(angle::AbstractFloat)::Symbol
    # Duplicated each to make the math below easier
    symbols = [Symbol(:E), Symbol(:NE),Symbol(:NE), Symbol(:N), Symbol(:N), Symbol(:NW), Symbol(:NW), 
    Symbol(:W), Symbol(:W), Symbol(:SW), Symbol(:SW), Symbol(:S), Symbol(:S), Symbol(:SE), Symbol(:SE), Symbol(:E)]
    # 16 subregions, two of which assigned to each symbol
    dtheta = (2*pi) / 16
    index_fn(a) = Integer(floor(a / dtheta)) + 1
    return symbols[index_fn(angle)]
end

function place_uavs(centroid::Tuple{AbstractFloat,AbstractFloat}, time::Integer, config::FormationPlannerConfigs)::Vector{MDPState}
    theta = 0.0
    dtheta = 2*pi / config.num_robots
    output_states = MDPState[]
    targets = config.target_trajectories[time, :]
    for _ in 1:config.num_robots
        x = config.radius * cos(theta) + centroid[1]
        y = config.radius * sin(theta) + centroid[2]
        u_state = UAVState(x, y, Symbol(:E))
        state = MDPState(u_state, time, config.horizon, nothing)
        push!(output_states, align_nearest(state, targets))
        theta += dtheta
    end
    output_states
end

function align_nearest(state::MDPState, targets::Vector{Target})::MDPState
    min_d::Float64 = 100000000000000.0;
    closest_t = nothing
    angle = nothing
    for t in targets
        d = sqrt((state.state.x + t.x)^2 + (state.state.y + t.y)^2)
        if d < min_d
            min_d = d
            closest_t = t
            angle = absoluteAngle(t.x - state.state.x, t.y - state.state.y)
        end 
    end
    u_state = UAVState(state.state.x, state.state.y, heading_from_angle(angle))
    MDPState(u_state, state.depth, state.horizon, nothing)
end

function solve_formation(config::FormationPlannerConfigs)::Solution
    elements = Tuple{Int64, Vector{MDPState}}[]
    for robot_id in 1:config.num_robots
        println("Starting Robot $(robot_id)")
        robot_states = MDPState[]
        for time in 1:config.horizon
            targets = config.target_trajectories[time, :]
            centroid = target_centroid(targets)
            states = place_uavs(centroid, time, config)
            push!(robot_states, states[robot_id])
        end
        push!(elements, (robot_id, robot_states))
    end

    # TODO: Fix solution value
    Solution(1.0, elements)
end