using QuickPOMDPs: QuickPOMDP
using POMDPTools # : Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator, HistoryRecorder, DiscreteUpdater, UnderlyingMDP
using Statistics: mean, std
using Plots
using POMDPs: actions, @gen, isterminal, discount, statetype, actiontype, simulate, states, initialstate
using ProgressMeter

# Custom Imports
include("dynamics.jl")
include("collision_checker.jl")
include("planner_interface.jl")
include("error_tracking.jl")
include("policies.jl")
include("plotter.jl")

# ----- Custom Runtime setup for Windows -----
# ENV["PYTHONHOME"] = raw"C:\Users\ckuru\AppData\Local\Programs\Python\Python312"
# ENV["PYTHONPATH"] = raw"C:\Users\ckuru\AppData\Local\Programs\Python\Python312\Lib"

# ENV["PATH"] =
#     raw"C:\Users\ckuru\.julia\dev\libcxxwrap_julia_jll\override\bin;" *
#     raw"C:\Users\ckuru\AppData\Local\Programs\Python\Python312;" *
#     raw"C:\vcpkg\installed\x64-windows\bin;" *
#     raw"C:\Users\ckuru\ASEN5264\ASEN5264Project\lib;" *
#     Sys.BINDIR * ";" *
#     ENV["PATH"]

# Load custom ompl planning library into CppOMPL module, if not already loaded
if !isdefined(Main, :CppOMPL)
    include(joinpath(@__DIR__, "loadCppModule.jl"))
end

# Directories
# const ENV_DIR = joinpath(@__DIR__, "OMPL/environments")
const ENV_DIR = raw"/Users/Jacob/Downloads/School/ASEN 5264/ASEN5264Project/OMPL/environments"
const obs_file = joinpath(ENV_DIR, "test.csv")
const endpoints_file = joinpath(ENV_DIR, "test_ends.csv")
# ----- Constants -----
const max_fails = 5
const dt = 0.1
const turn_bias = 0.8
const collision_penalty = 100.0
const goal_reward = 100.0
const replan_cost = 2.0
const wheel_radius = 0.5
const fail_chance = 0.01

# Max steps for MC simulations
const maxSteps = 1000

# Measurement Thresholds
const measThresholds = thresholds(0.25, pi/30, 0.75, pi/6)

# Bounds checking 
const xmin = 0.0
const xmax = 10.0
const ymin = 0.0
const ymax = 10.0

# -------------- Plan History ---------------
# Keep track of previous plans for Plotting
const tracking = true # If true, will track path plan history in pathHistory
const pathHistory = Vector{Vector{Vector{Float64}}}()

# -------------- Initialize the problem ---------------
start_state, goal_state = load_start_goal(endpoints_file)
obstacles = get_obstacles_csv(obs_file)

main_pomdp = QuickPOMDP(
    # Continuous state stored as:
    # ((x, y, theta), mode, plan_type, plan_index, controls, expected_path)
    # (x, y, theta) are the continuous state space
    # mode = [:healthy, :turn_bias]
    # plan_type = [:healthy, :turn_bias]
    # plan_index = Int representing index of plan
    # controls = Vector{Vector{Float64}} sequence of controls to goal under current dynamics. Should be applied at interval dt
    # expected_path = Vector{Vector{Float64}} expected path following controls if dynamics stay constant

    # For continuous spaces, don't set "state"
    initialstate = Deterministic((start_state, :healthy, :healthy, 1, create_plan(:healthy, start_state)...)),
    
    actions = [:continue_plan, :replan_nominal, :replan_failure],
    observations = [(error, plan_type) for error in (:small_error, :medium_error, :large_error) for plan_type in (:healthy, :turn_bias)],

    transition = function(s, a)
        x, mode, plan_type, plan_index, controls, expected_path = s

        # Replan under nominal model
        if a == :replan_nominal
            if tracking # If tracking, record the current plan before replanning
                push!(pathHistory, expected_path[1:plan_index])
            end
            return Deterministic((x, mode, :healthy, 1, create_plan(:healthy, x)...)) # For nominal trajectory, make plan with healthy wheel dynamics
        # Replan under failure-aware model
        elseif a == :replan_failure
            if tracking # If tracking, record the current plan before replanning
                push!(pathHistory, expected_path[1:plan_index])
            end
            return Deterministic((x, mode, :turn_bias, 1, create_plan(:turn_bias, x)...))

        # Continue current plan
        else
            u = get_planned_control(controls, plan_index)

            # If no control left, stay in place
            if u === nothing
                return Deterministic((x, mode, plan_type, plan_index, controls, expected_path))
            end

            # Propagate actual state using current control
            x_next = propagate_unicycle(x, u, mode, dt)
            
            # increment plan index 
            next_plan_index = plan_index + 1

            # Hidden dynamics: mode probabilistically switches between healthy and biased
            make_state = mode -> (x_next, mode, plan_type, next_plan_index, controls, expected_path) # Helper to construct state
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
        x, mode, plan_type, plan_index, controls, expected_path = sp

        # Helper to construct observation. plan_type is deterministically observed
        obs = error -> (error, plan_type)

        # If action was not to continue, error is small by definition
        if a != :continue_plan
            return Deterministic(obs(:small_error))
        end

        # If reached goal or in collision, observation doesn't matter
        if in_collision(x, obstacles) || reached_goal(x, goal_state)
            return Deterministic(obs(:small_error))
        else
            x_plan = get_planned_state(expected_path, plan_index)
            if x_plan === nothing
                return Deterministic(obs(:large_error))
            end

            z_true = tracking_error_level(x, x_plan)

            if z_true == :small_error
                return SparseCat(
                    [obs(:small_error), obs(:medium_error), obs(:large_error)],
                    [0.85, 0.10, 0.05]
                )

            elseif z_true == :medium_error
                return SparseCat(
                    [obs(:small_error), obs(:medium_error), obs(:large_error)],
                    [0.10, 0.80, 0.10]
                )

            else
                return SparseCat(
                    [obs(:small_error), obs(:medium_error), obs(:large_error)],
                    [0.05, 0.10, 0.85]
                )
            end
        end
    end,

    reward = function(s, a, sp)
        x, mode, plan_type, plan_index, controls, expected_path = sp

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
    
    discount = 1.00,
    isterminal = s -> in_collision(s[1], obstacles) || reached_goal(s[1], goal_state)
)

# Do a test with the always continue policy

# Plot a single run of π_continue policy, which is the baseline
history = simulate(HistoryRecorder(max_steps=maxSteps), main_pomdp, π_continue) # history is a SimHistory object

display(plot_plan_with_actual(pathHistory, first.(state_hist(history))))

@show collect(observation_hist(history))[end-10:end]