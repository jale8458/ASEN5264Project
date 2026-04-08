using QuickPOMDPs: QuickPOMDP
using POMDPTools: Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator
using Statistics: mean, std
using Plots
import POMDPs
using POMDPs: actions, @gen, isterminal, discount, statetype, actiontype, simulate, states, initialstate

############
# Baseline POMDP
############

const max_fails = 5

wheel_failure_pomdp = QuickPOMDP(
    # Enumerate all state combinations (e.g. (:healthy, num_fails = 0), (:stuck, num_fails =3) )
    states = [(mode, num_fails) for mode in (:healthy, :stuck) for num_fails in 0:max_fails],
    actions = [:healthy_action, :stuck_action],
    observations = [:wheel_healthy, :wheel_stuck],

    transition = function(s, a)
        mode, num_fails = s # unpack state

        # Latent wheel state switches with small probability
        if mode == :healthy
            # State is healthy and action is correcrt
            if a == :healthy_action
                return SparseCat(
                    [(:healthy, 0), (:stuck, min(num_fails+1, max_fails))], # Reset num_fails to 0 and state transition
                    [0.99, 0.01])
            else
                # State is healthy and action is incorrect 
                return SparseCat(
                    [(:healthy, min(num_fails+1, max_fails)), (:stuck, 0)], # Increment num_fails and state transition
                    [0.99, 0.01]
                )
            end
        else
            if a == :stuck_action
                # State is stuck and action is correct
                return SparseCat(
                    [(:stuck, 0), (:healthy, min(num_fails+1, max_fails))], # Reset num_fails to 0 and state transition
                    [0.99, 0.01])
            else
                return SparseCat(
                    [(:stuck, min(num_fails+1, max_fails)), (:healthy, 0)], # Increment num_fails and state transition
                    [0.99, 0.01]
                )
            end
        end
    end,

    observation = function(a, sp)
        next_mode, num_fails = sp
        
        if next_mode == :healthy 
            # Sensor is 80% accurate 
            return SparseCat([:wheel_healthy, :wheel_stuck], [0.80, 0.20])
        else
            return SparseCat([:wheel_stuck, :wheel_healthy], [0.80, 0.20])
        end
    end, 

    reward = function (s, a, sp)
        mode, num_fails = sp # unpack state

        correct_prediction = (a == :healthy_action && mode == :healthy) || 
                             (a == :stuck_action && mode == :stuck)
        if correct_prediction
            return 2.0 - num_fails
        else
            return -1.0 * num_fails 
        end
    end,
           
    initialstate = Deterministic((:healthy, 0)), # Realistic assumption that initial state is healthy
    discount = 0.95,
    isterminal = s -> false

)

# Evaluate with a policy that always takes healthy action
function always_healthy(mdp, s)
    return :healthy_action
end

# Evaluate with a policy that always takes stuck action
function always_stuck(mdp, s)
    return :stuck_action
end

# Rollout function 
function rollout(mdp, policy_function, s, max_steps=100)
    # fill this in with code from the assignment document
    r_total = 0.0
    t = 0
    while !isterminal(mdp, s) && t < max_steps
        a = policy_function(mdp, s) 
        s, r = @gen(:sp,:r)(mdp, s, a)
        r_total += discount(mdp)^t*r
        t += 1 
    end 
    return r_total 
end

############
# Monte Carlo evaluation
############

# evaluate with a policy that always waits
results_healthy = [
    rollout(wheel_failure_pomdp, always_healthy, rand(initialstate(wheel_failure_pomdp)), 100)
    for _ in 1:100000
]

println("Always healthy policy:")
@show sample_mean_healthy = mean(results_healthy)
@show SEM_healthy = std(results_healthy) / sqrt(length(results_healthy))


results_stuck = [
    rollout(wheel_failure_pomdp, always_stuck, rand(initialstate(wheel_failure_pomdp)), 100)
    for _ in 1:100000
]

println("\nAlways stuck policy:")
@show sample_mean_stuck = mean(results_stuck)
@show SEM_stuck = std(results_stuck) / sqrt(length(results_stuck))
