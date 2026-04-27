# ----- Constants for ApPrOMDP -----
# Number of mismatches between plan_type and wrong_count before reporting medium and large errors
# This "approximates" calculating the error between the true trajectory and expected trajectory
const num_fails_med::Int = 2
const num_fails_large::Int = 5

approx_pomdp = QuickPOMDP(
    # State stored as:
    # (mode, plan_type, wrong_count)
    # (x, y, theta) are the continuous state space
    # mode = [:healthy, :turn_bias]
    # plan_type = [:healthy, :turn_bias]
    # wrong_count = Number of times (mode != plan_type) before replanning. Used to determine observation and reward.

    # Discrete state space
    states = [(mode, plan_type, wrong_count) for mode in (:healthy, :turn_bias) for plan_type in (:healthy, :turn_bias) for wrong_count in 0:num_fails_large],
    initialstate = Deterministic((:healthy, :healthy, 0)),
    
    actions = [:continue_plan, :replan_nominal, :replan_failure],
    observations = [(error, plan_type) for error in (:small_error, :medium_error, :large_error) for plan_type in (:healthy, :turn_bias)],

    transition = function(s, a)
        mode, plan_type, wrong_count = s

        # If replanning, set wrong_count to 0
        if a == :replan_nominal
            return Deterministic((mode, :healthy, 0))
        # Replan under failure-aware model
        elseif a == :replan_failure
            return Deterministic((mode, :turn_bias, 0))
        # Continuing with plan
        else
            # Increment wrong_count (until num_fails_large) if our dynamics are not consistent with our model. This is used to determine observation
            if (mode != plan_type) && wrong_count < num_fails_large
                wrong_count += 1
            end

            # Hidden dynamics: mode probabilistically switches between healthy and biased
            make_state = mode -> (mode, plan_type, wrong_count) # Helper to construct state
            if mode == :healthy
                return SparseCat(
                    [
                        make_state(:healthy),
                        make_state(:turn_bias)
                    ],
                    [1 - fail_chance, fail_chance]
                )
            elseif mode == :turn_bias
                return SparseCat(
                    [
                        make_state(:turn_bias),
                        make_state(:healthy)
                    ],
                    [1 - fail_chance, fail_chance]
                )
            else
                error("Unknown mode in state")
            end
        end
    end,

    observation = function(a, sp)
        # Extract state
        mode, plan_type, wrong_count = sp

        # Helper to construct observation. plan_type is deterministically observed
        obs = error -> (error, plan_type)

        # If action was to replan, error is small by definition
        if a == :replan_nominal || a == :replan_failure
            return Deterministic(obs(:small_error))
        end

        # To simulate z_true in the main POMDP, look at number of 
        if wrong_count < num_fails_med # Small error
            return SparseCat(
                [obs(:small_error), obs(:medium_error), obs(:large_error)],
                [0.85, 0.10, 0.05]
            )

        elseif wrong_count < num_fails_large # Medium error
            return SparseCat(
                [obs(:small_error), obs(:medium_error), obs(:large_error)],
                [0.10, 0.80, 0.10]
            )

        else # Large Error
            return SparseCat(
                [obs(:small_error), obs(:medium_error), obs(:large_error)],
                [0.05, 0.10, 0.85]
            )
        end
    end,

    reward = function(s, a, sp)
        mode, plan_type, wrong_count = sp

        # Replanning will have 0 tracking error cost and only the replanning cost
        if a == :replan_nominal || a == :replan_failure
            return -replan_cost
        end

        # To approximate main POMDP tracking reward, use wrong_count to determine if error is small, medium, or large.
        # Then, assume the reward is the smallest possible error in the error categories
        if wrong_count < num_fails_med # Small error
            pos_err = 0
            heading_err = 0

        elseif wrong_count < num_fails_large # Medium error
            pos_err = measThresholds.small_pos
            heading_err = measThresholds.small_heading

        else # Large Error
            pos_err = measThresholds.med_pos
            heading_err = measThresholds.med_heading
        end
        tracking_penalty = tracking_penalty_func(pos_err, heading_err)

        return -tracking_penalty
    end,
    
    discount = 0.999
)