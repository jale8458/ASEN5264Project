using POMDPs
using QuickPOMDPs: QuickPOMDP
using POMDPTools: Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator, DiscreteUpdater, UnderlyingMDP
using Statistics: mean, std
using Plots
using ProgressMeter
# Solvers
using SARSOP: SARSOPSolver
using QMDP: QMDPSolver
using DiscreteValueIteration: ValueIterationSolver
using BasicPOMCP

############
# Baseline POMDP
############
@info "Generating POMDP"

const max_fail_penalty = 5
const p_fail = 0.01

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

############
# Solvers
############
@info "Generating Solvers"

# Updater
up = DiscreteUpdater(wheel_failure_pomdp)

# Policies
π_healthy = FunctionPolicy(b -> :healthy_action)
π_stuck = FunctionPolicy(b -> :stuck_action)
π_qmdp = solve(QMDPSolver(), wheel_failure_pomdp)

# Value Iteration from POMDP
function value_iteration(m)
    solver = ValueIterationSolver(max_iterations=1000, belres=1e-6)
    policy = solve(solver, UnderlyingMDP(m))
    return policy.util  # this IS the value vector you index with stateindex
end

# POMCP Solver
function pomcp_solve(m) # this function makes capturing m in the rollout policy more efficient
    # Use the value iteration estimate as the estimate value
    V = value_iteration(m)
    solver = POMCPSolver(tree_queries=500,
                         max_depth=20,
                         c=1.0,
                         default_action= ExceptionRethrow(),
                         estimate_value= (m, s, h::BeliefNode, steps) -> V[stateindex(m,s)])
    return solve(solver, m)
end

# POMCP policy
π_pomcp = pomcp_solve(wheel_failure_pomdp)

##### NOTE: SARSOP only works when sp is the observation
# using SARSOP

# solver = SARSOPSolver(precision=1e-6, timeout=30.0)
# π_sarsop = solve(solver, wheel_failure_pomdp)
# results_sarsop = @showprogress "Running SARSOP Policy" [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_sarsop, up) for _ in 1:numRuns]
# @info "Always stuck policy:"
# @show μ_sarsop = mean(results_sarsop)
# @show SEM_sarsop = std(results_sarsop) / sqrt(length(results_stuck))

############
# Monte Carlo evaluation
############
# @info "Monte Carlo Evaluations"

# # Parameters
# numRuns = 100
# maxSteps = 500

# # Always assumes wheel is healthy
# results_healthy = @showprogress "Running Healthy Policy" [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_healthy, up) for _ in 1:numRuns]
# @info "Always healthy policy:"
# @show μ_healthy = mean(results_healthy)
# @show SEM_healthy = std(results_healthy) / sqrt(length(results_healthy))

# # Always assumes wheel is stuck
# results_stuck = @showprogress "Running Stuck Policy" [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_stuck, up) for _ in 1:numRuns]
# @info "Always stuck policy:"
# @show μ_stuck = mean(results_stuck)
# @show SEM_stuck = std(results_stuck) / sqrt(length(results_stuck))

# # QMDP
# results_qmdp = @showprogress "Running QMDP Policy" [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_qmdp, up) for _ in 1:numRuns]
# @info "QMDP policy:"
# @show μ_QMDP = mean(results_qmdp)
# @show SEM_QMDP = std(results_qmdp) / sqrt(length(results_qmdp))

# # POMCP
# results_pomcp = @showprogress "Running POMCP Policy" [simulate(RolloutSimulator(max_steps=maxSteps), wheel_failure_pomdp, π_pomcp, up) for _ in 1:numRuns]
# @info "POMCP policy:"
# @show μ_POMCP = mean(results_pomcp)
# @show SEM_POMCP = std(results_pomcp) / sqrt(length(results_pomcp))