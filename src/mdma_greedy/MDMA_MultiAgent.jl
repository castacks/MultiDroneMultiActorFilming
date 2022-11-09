
struct MultiRobotTargetTrackingProblem{F<:AbstractFilter} <: AbstractMultiRobotProblem
  # Target tracking problems are defined by vectors of robot states
  partition_matroid::Vector{State}

  configs::MultiRobotTargetTrackingConfigs

  function MultiRobotTargetTrackingProblem(robot_states::Vector{State},
                 target_filters::Vector{F},
                 configs::MultiRobotTargetTrackingConfigs) where F <: AbstractFilter

    if length(robot_states) > num_robots_sparse_filtering_threshold
      sparse_filters = map(x->SparseFilter(x, threshold=sparsity_threshold),
                           target_filters)

      filter_means = compute_filter_means(sparse_filters)

      new{SparseFilter{Int64}}(robot_states, sparse_filters, filter_means,
                               configs)
    else
      filter_means = compute_filter_means(target_filters)

      new{F}(robot_states, target_filters, filter_means, configs)
    end
  end
end

function solve_block(p::MultiRobotTargetTrackingProblem, block::Integer,
                     selections::Vector)
  configs = p.configs

  trajectories = map(last, selections)

  # Select filters in range
  local targets_in_range
  if p.configs.robot_target_range_limit == Inf
    targets_in_range = p.target_filters
  else
    targets_in_range = filter_targets_in_range(p, block)
  end

  problem = SingleRobotTargetTrackingProblem(configs.grid, configs.sensor,
                                             configs.horizon, targets_in_range,
                                             prior_trajectories=trajectories,
                                             num_information_samples=
                                             configs.solver_information_samples
                                            )

  state = get_state(p, block)

  solution = solve_single_robot(problem, state,
                                n_iterations=configs.solver_iterations)

  (block, solution.trajectory)
end
