using SubmodularMaximization
using PyPlot
# using MDMA
# include("MDMA.jl")
# using MDMA

close("all")

## Robot States are a tuple, x, y, heading, and bool for done
steps = 100
grid_size = 10

grid = Grid(grid_size, grid_size)

robot_pos = div.((grid_size, grid_size), 2)
robot_state = robot_pos
# robot_state = UAVState(robot_pos[1], robot_pos[2], :E)
# sensor = robot_state.sensor
sensor = RangingSensor()

initial = random_state(grid)
states = Array{typeof(initial)}(undef, steps)
states[1] = initial

plot_state_space(grid)
xlim([0, grid_size+1])
ylim([0, grid_size+1])

for ii = 2:steps
    println("Step ", ii)
    state = target_dynamics(grid, states[ii-1])
    states[ii] = state

    range_observation = generate_observation(grid, sensor, robot_state, state)

    plots=[]
    append!(plots, plot_robot(robot_state))
    append!(plots, plot_observation(robot_state, range_observation))
    append!(plots, plot_trajectory(states[1:ii]))

    sleep(0.1)

    # if ii < steps
    #     foreach(x->x.remove(), plots)
    # end
end
