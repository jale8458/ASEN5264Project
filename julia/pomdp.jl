using POMDPs
using QuickPOMDPs: QuickPOMDP
using POMDPTools: Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator, DiscreteUpdater, UnderlyingMDP
using Statistics: mean, std
using Plots
using ProgressMeter
# Solvers
# using SARSOP: SARSOPSolver
using QMDP: QMDPSolver
using DiscreteValueIteration: ValueIterationSolver
using BasicPOMCP

const p_fail = 0.01

wheel_failure_pomdp = QuickPOMDP(
    # Enumerate all state combinations (e.g. (:healthy, fail_penalty = 1), (:stuck, fail_penalty =3) )
    actions = [:continue_plan, :replan_nominal, :replan_failure],
    observations = [:small_error, :large_error],

    transition = function(s, a)
        mode, fail_penalty = s # unpack state

        # Latent wheel state switches with small probability
        if mode == :healthy
            # State is healthy and action is correct
            if a == :healthy_action
                return SparseCat(
                    [(:healthy, 1), (:stuck, 1)], # Reset fail_penalty to 1 and state transition
                    [1-p_fail, p_fail])
            else
                # State is healthy and action is incorrect 
                return SparseCat(
                    [(:healthy, min(fail_penalty+1, max_fail_penalty)), (:stuck, min(fail_penalty+1, max_fail_penalty))], # Increment fail_penalty and state transition
                    [1-p_fail, p_fail]
                )
            end
        else # mode == :stuck
            if a == :stuck_action
                # State is stuck and action is correct
                return SparseCat(
                    [(:stuck, 1), (:healthy, 1)], # Reset fail_penalty to 1 and state transition
                    [1-p_fail, p_fail])
            else
                return SparseCat(
                    [(:stuck, min(fail_penalty+1, max_fail_penalty)), (:healthy, min(fail_penalty+1, max_fail_penalty))], # Increment fail_penalty and state transition
                    [1-p_fail, p_fail]
                )
            end
        end
    end,

    observation = function(s, a, sp)
        # Observation based on PRIOR state only
        mode, fail_penalty = s
        
        if mode == :healthy 
            # Sensor is 80% accurate 
            return SparseCat([:wheel_healthy, :wheel_stuck], [0.80, 0.20])
        else
            return SparseCat([:wheel_stuck, :wheel_healthy], [0.80, 0.20])
        end
    end, 

    reward = function (s, a)
        mode, fail_penalty = s # unpack state

        correct_prediction = (a == :healthy_action && mode == :healthy) || 
                             (a == :stuck_action && mode == :stuck)
        if correct_prediction
            return 1.0
        else
            return -2.0 * fail_penalty 
        end
    end,
           
    initialstate = Deterministic((:healthy, 1)), # Realistic assumption that initial state is healthy
    discount = 0.95,
    isterminal = s -> false
)