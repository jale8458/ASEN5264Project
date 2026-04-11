using POMDPs: actions, @gen, isterminal, discount, statetype, actiontype, simulate, states, initialstate, solve
using QuickPOMDPs: QuickPOMDP
using POMDPTools: Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator, DiscreteUpdater
using Statistics: mean, std
using Plots
using SARSOP: SARSOPSolver
using QMDP: QMDPSolver

############
# Baseline POMDP
############

const max_fail_penalty = 5

wheel_failure_pomdp = QuickPOMDP(
    # Enumerate all state combinations (e.g. (:healthy, fail_penalty = 1), (:stuck, fail_penalty =3) )
    states = [(mode, fail_penalty) for mode in (:healthy, :stuck) for fail_penalty in 1:max_fail_penalty],
    actions = [:healthy_action, :stuck_action],
    observations = [:wheel_healthy, :wheel_stuck],

    transition = function(s, a)
        mode, fail_penalty = s # unpack state

        # Latent wheel state switches with small probability
        if mode == :healthy
            # State is healthy and action is correct
            if a == :healthy_action
                return SparseCat(
                    [(:healthy, 1), (:stuck, 1)], # Reset fail_penalty to 1 and state transition
                    [0.99, 0.01])
            else
                # State is healthy and action is incorrect 
                return SparseCat(
                    [(:healthy, min(fail_penalty+1, max_fail_penalty)), (:stuck, min(fail_penalty+1, max_fail_penalty))], # Increment fail_penalty and state transition
                    [0.99, 0.01]
                )
            end
        else # mode == :stuck
            if a == :stuck_action
                # State is stuck and action is correct
                return SparseCat(
                    [(:stuck, 1), (:healthy, 1)], # Reset fail_penalty to 1 and state transition
                    [0.99, 0.01])
            else
                return SparseCat(
                    [(:stuck, min(fail_penalty+1, max_fail_penalty)), (:healthy, min(fail_penalty+1, max_fail_penalty))], # Increment fail_penalty and state transition
                    [0.99, 0.01]
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

############
# Solvers
############

# Updater
up = DiscreteUpdater(wheel_failure_pomdp)

# Policies
π_healthy = FunctionPolicy(b -> :healthy_action)
π_stuck = FunctionPolicy(b -> :stuck_action)
π_qmdp = solve(QMDPSolver(), wheel_failure_pomdp)

############
# Monte Carlo evaluation
############

# Parameters
numRuns = 5000
maxSteps = 500

# Always assumes wheel is healthy
results_healthy = [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_healthy, up) for _ in 1:numRuns]
println("Always healthy policy:")
@show μ_healthy = mean(results_healthy)
@show SEM_healthy = std(results_healthy) / sqrt(length(results_healthy))

# Always assumes wheel is stuck
results_stuck = [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_stuck, up) for _ in 1:numRuns]
println("\nAlways stuck policy:")
@show μ_stuck = mean(results_stuck)
@show SEM_stuck = std(results_stuck) / sqrt(length(results_stuck))

# QMDP
results_qmdp = [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_qmdp, up) for _ in 1:numRuns]
println("\nQMDP policy:")
@show μ_QMDP = mean(results_qmdp)
@show SEM_stuck = std(results_qmdp) / sqrt(length(results_qmdp))
