approx_pomdp = QuickPOMDP(
    # State stored as:
    # (mode, plan_type, wrong_count)
    # (x, y, theta) are the continuous state space
    # mode = [:healthy, :turn_bias]
    # plan_type = [:healthy, :turn_bias]
    # wrong_count = Number of times in a row (mode != plan_type). Used to determine observation.

    # For continuous spaces, don't set "state"
    initialstate = Deterministic((:healthy, :nominal, 0)),
    
    actions = [:continue_plan, :replan_nominal, :replan_failure],
    observations = [(error, plan_type) for error in (:small_error, :medium_error, :large_error) for plan_type in (:healthy, :turn_bias)],

    transition = function(s, a)
        x, mode, plan_index, controls, expected_path = s

        # Replan under nominal model
        if a == :replan_nominal
            if tracking # If tracking, record the current plan before replanning
                push!(pathHistory, expected_path[1:plan_index])
            end
            return Deterministic((x, mode, 1, create_plan(:healthy, x)...))
        # Replan under failure-aware model
        elseif a == :replan_failure
            if tracking # If tracking, record the current plan before replanning
                push!(pathHistory, expected_path[1:plan_index])
            end
            return Deterministic((x, mode, 1, create_plan(:turn_bias, x)...))

        # Continue current plan
        else
            u = get_planned_control(controls, plan_index)

            # If no control left, stay in place
            if u === nothing
                return Deterministic((x, mode, plan_index, controls, expected_path))
            end

            # Propagate actual state using current control
            x_next = propagate_unicycle(x, u, mode, dt)
            
            # increment plan index 
            next_plan_index = plan_index + 1

            # Hidden dynamics: mode probabilistically switches between healthy and biased
            if mode == :healthy
                return SparseCat(
                    [
                        (x_next, :healthy, next_plan_index, controls, expected_path),
                        (x_next, :turn_bias, next_plan_index, controls, expected_path)
                    ],
                    [1 - fail_chance, fail_chance]
                )
            elseif mode == :turn_bias
                return SparseCat(
                    [
                        (x_next, :turn_bias, next_plan_index, controls, expected_path),
                        (x_next, :healthy, next_plan_index, controls, expected_path)
                    ],
                    [1 - fail_chance, fail_chance]
                )
            else
                error("Unknown mode in state")
            end
        end
    end,

    observation = function(a, sp)
        # If action was not to continue, error is small by definition
        if a != :continue_plan
            return Deterministic(:small_error)
        end

        # Extract state
        x, mode, plan_index, controls, expected_path = sp
        # If reached goal or in collision, observation doesn't matter
        if in_collision(x, obstacles) || reached_goal(x, goal_state)
            return Deterministic(:small_error)
        else
            x_plan = get_planned_state(expected_path, plan_index)
            if x_plan === nothing
                return Deterministic(:large_error)
            end

            z_true = tracking_error_level(x, x_plan)

            if z_true == :small_error
                return SparseCat(
                    [:small_error, :medium_error, :large_error],
                    [0.85, 0.10, 0.05]
                )

            elseif z_true == :medium_error
                return SparseCat(
                    [:small_error, :medium_error, :large_error],
                    [0.10, 0.80, 0.10]
                )

            else
                return SparseCat(
                    [:small_error, :medium_error, :large_error],
                    [0.05, 0.10, 0.85]
                )
            end
        end
    end,

    reward = function(s, a, sp)
        x, mode, plan_index, controls, expected_path = sp

        if in_collision(x, obstacles)
            if tracking # If tracking, record the current plan before terminating
                push!(pathHistory, expected_path)
            end
            return -collision_penalty

        elseif reached_goal(x, goal_state)
            if tracking # If tracking, record the current plan before terminating
                push!(pathHistory, expected_path)
            end
            return goal_reward

        else
            # tracking error relative to current planned state
            x_plan = get_planned_state(expected_path, plan_index)

            if x_plan === nothing
                pos_err, heading_err = tracking_error(x, expected_path[end])
            else
                pos_err, heading_err = tracking_error(x, x_plan)
            end
            tracking_penalty = pos_err + 0.5 * heading_err # NOTE: We should rethink this

            # penalize replanning
            if a == :continue_plan
                plan_penalty = 0.0
            else
                plan_penalty = replan_cost
            end

            return -tracking_penalty - plan_penalty
        end
    end,
    
    discount = 0.99,
    isterminal = s -> in_collision(s[1], obstacles) || reached_goal(s[1], goal_state)
)