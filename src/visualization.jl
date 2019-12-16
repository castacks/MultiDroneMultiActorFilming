using PyCall
using PyPlot
using Colors
matplotlib2tikz = pyimport("matplotlib2tikz")

###############
# visualization
###############
export to_file, save_fig

to_file(s) = replace(lowercase(s), " " => "_")

function save_fig(fig_path, title)
  matplotlib2tikz.save("$(fig_path)/$(to_file(title)).tex",
                       figureheight="\\figureheight",
                       figurewidth="\\figurewidth",
                       show_info=false)
end

# standard markers
station_center = "+"
sensor_center = (5, 2, 0)
selected_sensor = "*"
event_marker = "x"

agent_scale = 32.0

station_style = "--"


rgb_tuple(color::RGB) = (red(color), green(color), blue(color))

generate_colors(agents) = distinguishable_colors(length(agents) + 1)[2:end]

function visualize_agents(agents, colors)
  centers = map(get_center, agents);

  map(agents, colors) do agent, color
    center = get_center(agent)

#   scatter([center[1]], [center[2]], color = rgb_tuple(color),
#           s = 4*agent_scale, marker = station_center)

    map(get_block(agent)) do element
      plot_element(element; color = rgb_tuple(color))
    end

    plot_circle(Circle(agent.center, agent.radius);
                linestyle=station_style, color = rgb_tuple(color),
                linewidth = 3.0)
  end

  nothing
end

function visualize_solution(p::PartitionProblem, X::ElementArray, agent_colors)
  elements = map(X) do x
    get_element(p.partition_matroid, x)
  end

  colors = [agent_colors[x[1]] for x in X]
  visualize_solution(elements, colors)
end

visualize_solution(p::PartitionProblem, s::Solution, agent_colors) =
  visualize_solution(p, s.elements, agent_colors)

# visualization
function circle_points(p; radius=1, scale=1, dth=0.1)
  scale*hcat(map(x->p+[radius*cos(x);radius*sin(x)], 0:dth:(2*pi)+dth)...)
end

function plot_circle(p; scale=1, radius=1, color="k", linestyle="-", linewidth=1.0, kwargs...)
  c = circle_points(p, scale=scale, radius=radius)
  plot(c[1,:][:], c[2,:][:], color=color, linestyle=linestyle, linewidth=linewidth, kwargs...)
end