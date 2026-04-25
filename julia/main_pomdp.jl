using QuickPOMDPs: QuickPOMDP
using POMDPTools: Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator, DiscreteUpdater, UnderlyingMDP
using Statistics: mean, std
using Plots
import POMDPs
using POMDPs: actions, @gen, isterminal, discount, statetype, actiontype, simulate, states, initialstate
using ProgressMeter

# Custom Imports
include("collision_checker.jl")
include("planner_interface.jl")
include("dynamics.jl")
include("error_tracking.jl")
include("policies.jl")
include("plotter.jl")

# ----- Custom Runtime setup for Windows -----
ENV["PYTHONHOME"] = raw"C:\Users\ckuru\AppData\Local\Programs\Python\Python312"
ENV["PYTHONPATH"] = raw"C:\Users\ckuru\AppData\Local\Programs\Python\Python312\Lib"

ENV["PATH"] =
    raw"C:\Users\ckuru\.julia\dev\libcxxwrap_julia_jll\override\bin;" *
    raw"C:\Users\ckuru\AppData\Local\Programs\Python\Python312;" *
    raw"C:\vcpkg\installed\x64-windows\bin;" *
    raw"C:\Users\ckuru\ASEN5264\ASEN5264Project\lib;" *
    Sys.BINDIR * ";" *
    ENV["PATH"]

# Load custom ompl planning library into CppOMPL module, if not already loaded
if !isdefined(Main, :CppOMPL)
    include(joinpath(@__DIR__, "loadCppModule.jl"))
end

# Directories
# const ENV_DIR = joinpath(@__DIR__, "OMPL/environments")
const ENV_DIR = raw"C:\Users\ckuru\ASEN5264\ASEN5264Project\OMPL\environments"
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

# Bounds checking 
const xmin = 0.0
const xmax = 10.0
const ymin = 0.0
const ymax = 10.0

# -------------- Initialize the problem ---------------
start_state, goal_state = load_start_goal(endpoints_file)
obstacles = get_obstacles_csv(obs_file)

# Do a test with just the nominal SST plan 
set_active_plan(:nominal, start_state)
plot_original_plan()

main_pomdp = QuickPOMDP(
    # Continuous state stored as:
    # ((x, y, theta), mode, plan_index, plan_type)
    states = [(start_state, :healthy, 1, :nominal)],
    actions = [:continue_plan, :replan_nominal, :replan_failure],
    observations = [:small_error, :medium_error, :large_error, :collision_obs, :goal_obs],

    transition = function(s, a)
        x, mode, plan_index, plan_type = s

        # Replan under nominal model
        if a == :replan_nominal
            set_active_plan(:nominal, x)
            return Deterministic((x, mode, 1, :nominal))
        # Replan under failure-aware model
        elseif a == :replan_failure
            set_active_plan(:failure, x)
            return Deterministic((x, mode, 1, :failure))

        # Continue current plan
        else
            u = get_planned_control(plan_index)
            # ----- placeholder for what to do if no more controls to execute: 
            duration = get_control_duration(plan_index)

            # If no control left, stay in place
            if u === nothing ||  duration === nothing
                return Deterministic((x, mode, plan_index, plan_type))
            end

            # Propagate actual state using current control
            x_next = propagate_unicycle(x, u, mode, duration)

            # ----- placeholder for what to do if no more states left: 
            
            # increment plan index 
            next_plan_index = plan_index + 1

            # Hidden dynamics: mode probabilistically switches between healthy and biased
            if mode == :healthy
                return SparseCat(
                    [
                        (x_next, :healthy, next_plan_index, plan_type),
                        (x_next, :turn_bias, next_plan_index, plan_type)
                    ],
                    [0.99, 0.01]
                )
            else
                return SparseCat(
                    [
                        (x_next, :turn_bias, next_plan_index, plan_type),
                        (x_next, :healthy, next_plan_index, plan_type)
                    ],
                    [0.99, 0.01]
                )
            end
        end
    end,

    observation = function(a, sp)
        x, mode, plan_index, plan_type = sp
        if in_collision(x, obstacles)
            return Deterministic(:collision_obs)
        elseif reached_goal(x,goal_state)
            return Deterministic(:goal_obs)
        else
            x_plan = get_planned_state(plan_index)
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
        x, mode, plan_index, plan_type = sp

        if in_collision(x, obstacles)
            return -collision_penalty

        elseif reached_goal(x, goal_state)
            return goal_reward

        else
            # tracking error relative to current planned state
            x_plan = get_planned_state(plan_index)

            if x_plan === nothing
                tracking_penalty = 10.0
            else
                pos_err, heading_err = tracking_error(x, x_plan)
                tracking_penalty = pos_err + 0.5 * heading_err
            end

            # penalize replanning
            if a == :continue_plan
                plan_penalty = 0.0
            else
                plan_penalty = replan_cost
            end

            return -tracking_penalty - plan_penalty
        end
    end,

    initialstate = Deterministic((start_state, :healthy, 1, :nominal)),
    discount = 0.95,
    isterminal = s -> in_collision(s[1], obstacles) || reached_goal(s[1], goal_state)
)

# Do a test with the always continue policy
# Right now I have the vectors populated before calling the POMDP.
# Should the POMDP call the initial planner?
set_active_plan(:nominal, start_state)

r, actual_path = rollout_with_path(
    main_pomdp,
    always_continue,
    rand(initialstate(main_pomdp)),
    100
)

@show r

plot_plan_with_actual(actual_path)

