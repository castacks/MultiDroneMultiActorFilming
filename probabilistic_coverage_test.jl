using PyPlot
using SubmodularMaximization

########
# Params
########

num_events = 400
num_agents = 80
num_sensors = 6
nominal_area = 1.0

max_success_probability = 1.0
sensor_radius = sqrt(nominal_area / (num_agents * pi))
station_radius = 3*sensor_radius

#############
# environment
#############

figure()
events = generate_events(num_events)


########
# agents
########

agent_specification = ProbabilisticAgentSpecification(max_success_probability,
                                                      sensor_radius,
                                                      station_radius,
                                                      num_sensors)

agents = generate_agents(agent_specification, num_agents)
f(x) = mean_detection_probability(x, events)
problem = PartitionProblem(f, agents)


####################
# visualize scenario
####################

visualize_pdf(standard_mixture())
visualize_events(events)
colors = generate_colors(agents)
visualize_agents(agents, colors)
xlim([0, 1])
ylim([0, 1])
title("Scenario")

#############
# evaluateion
#############
function evaluate_solver(solver, name)
  println("$name solver running")
  @time solution = solver(problem)

  figure()
  xlim([0, 1])
  ylim([0, 1])
  colors = generate_colors(agents)
  visualize_agents(agents, colors)
  visualize_solution(problem, solution, colors)
  visualize_events(events)

  coverage = solution.value

  title("$name Solver ($coverage)")

  @show coverage
end

# evaluate basic solvers
#evaluate_solver(solve_myopic, "Myopic")
#evaluate_solver(solve_random, "Random")
evaluate_solver(solve_sequential, "Sequential")

# evaluate partition solvers
for num_partitions in [2, 4, 8]
  solve_n(p) = solve_n_partitions(num_partitions, p)
  evaluate_solver(solve_n, "Partition-$num_partitions")
end

@show mean_weight(problem)
@show total_weight(problem)