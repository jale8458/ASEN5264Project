using QuickPOMDPs: QuickPOMDP
using POMDPTools: Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator, DiscreteUpdater, UnderlyingMDP
using Statistics: mean, std
using Plots
import POMDPs
using POMDPs: actions, @gen, isterminal, discount, statetype, actiontype, simulate, states, initialstate
using ProgressMeter
include("main_pomdp_helpers.jl")
using Debugger
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
const obs_file = joinpath(ENV_DIR, "normalParking.csv")
const endpoints_file = joinpath(ENV_DIR, "normalParkingEndpoints.csv")
# ----- Constants -----
const max_fails = 5
const dt = 0.1
const turn_bias = 0.8
const collision_penalty = 100.0
const goal_reward = 100.0
const replan_cost = 2.0
const wheel_radius = 0.5
const fail_chance = 0.5

# Bounds checking 
const xmin = 0.0
const xmax = 10.0
const ymin = 0.0
const ymax = 10.0

# ----- Current active plan returned by planner -----
current_path = Vector{Vector{Float64}}()
current_controls = Vector{Vector{Float64}}()
current_control_durations = Float64[]

# Helper function: set active plan
function set_active_plan(plan_type, x)
    if plan_type == :nominal
        angle_bias = 0.0
    else
        angle_bias = turn_bias
    end

    SSTResult, control, controlDurations, path = Main.CppOMPL.PlanWithSSTFromState(
        "normalParking.csv",
        "normalParkingEndpoints.csv",
        x[1], x[2], x[3], 
        angle_bias,
        5.0
    )
    global current_controls = [control[:, i] for i in 1:size(control, 2)]
    global current_control_durations = collect(controlDurations)
    global current_path = [path[:, i] for i in 1:size(path, 2)]

end

# ----- Need helper functions to get planned state at next index -----
# Helper function: get planned state at index k
function get_planned_state(k)
    if 1 <= k <= length(current_path)
        return current_path[k]
    else
        return nothing
    end
end

# Helper function: get planned control at index k
function get_planned_control(k)
    if 1 <= k <= length(current_controls)
        return current_controls[k]
    else
        return nothing
    end
end

function get_control_duration(k)
    if 1 <= k <= length(current_control_durations)
        return current_control_durations[k]
    else
        return nothing
    end
end
# Helper function: propagate actual state using unicycle dynamics
function unicycle_dynamics(x,u,mode)
    px, py, theta = x
    v=  u[1]
    omega = u[2]

    if mode == :healthy
        omega_eff = omega
    else
        omega_eff = omega + turn_bias
    end

    return (
        wheel_radius * v * cos(theta), 
        wheel_radius * v * sin(theta),
        omega_eff
    )
end

function propagate_unicycle(x, u, mode, dt)
    # Use RK4 propagator with unicycle dynamics
    k1 = unicycle_dynamics(x, u, mode)

    x2 = (
        x[1] + 0.5*dt*k1[1],
        x[2] + 0.5*dt*k1[2],
        wrap_angle(x[3] + 0.5*dt*k1[3])
    )
    k2 = unicycle_dynamics(x2, u, mode)

    x3 = (
        x[1] + 0.5*dt*k2[1],
        x[2] + 0.5*dt*k2[2],
        wrap_angle(x[3] + 0.5*dt*k2[3])
    )
    k3 = unicycle_dynamics(x3, u, mode)

    x4 = (
        x[1] + dt*k3[1],
        x[2] + dt*k3[2],
        wrap_angle(x[3] + dt*k3[3])
    )
    k4 = unicycle_dynamics(x4, u, mode)

    x_next = x[1] + dt/6 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
    y_next = x[2] + dt/6 * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2])
    theta_next = wrap_angle(x[3] + dt/6 * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]))
    return (x_next, y_next, theta_next)
end

# Helper function: position + wrapped heading error
function tracking_error(x_actual, x_plan)
    dx = x_actual[1]-x_plan[1]
    dy = x_actual[2]-x_plan[2]
    dtheta = wrap_angle(x_actual[3] - x_plan[3])

    pos_err = sqrt(dx^2 + dy^2)
    heading_err = abs(dtheta)

    return pos_err, heading_err
end

# Helper function: decide if tracking error is large
function large_tracking_error(x_actual, x_plan; pos_thresh=0.5, heading_thresh=0.35)
    pos_err, heading_err = tracking_error(x_actual, x_plan)
    return pos_err > pos_thresh || heading_err > heading_thresh
end

############
# Plotting helpers
############
function path_matrix_from_current_path()
    xs = [p[1] for p in current_path]
    ys = [p[2] for p in current_path]
    return xs, ys
end

function plot_obstacles!(obstacles)
    for obs in obstacles
        xs = [p[1] for p in obs]
        ys = [p[2] for p in obs]

        # close polygon
        push!(xs, obs[1][1])
        push!(ys, obs[1][2])

        plot!(xs, ys, label=false, linewidth=2)
    end
end

function plot_original_plan(; title_str="Original SST Plan")
    xs, ys = path_matrix_from_current_path()

    p = plot(
        xs, ys,
        label="SST planned path",
        linewidth=3,
        marker=:circle,
        aspect_ratio=:equal,
        xlabel="x",
        ylabel="y",
        title=title_str,
        xlims=(0, 10),
        ylims=(0, 10)
    )

    plot_obstacles!(obstacles)

    scatter!([start_state[1]], [start_state[2]], label="start", markersize=6)
    scatter!([goal_state[1]], [goal_state[2]], label="goal", markersize=6)

    return p
end

function rollout_with_path(mdp, policy, s, max_steps=100)
    path = [s[1]]
    t = 0
    r_total = 0.0

    while !isterminal(mdp, s) && t < max_steps
        a = policy(mdp, s)
        s, r = @gen(:sp, :r)(mdp, s, a)

        push!(path, s[1])

        r_total += discount(mdp)^t * r
        t += 1
    end

    return r_total, path
end

function plot_plan_with_actual(actual_path)
    xs_plan = [p[1] for p in current_path]
    ys_plan = [p[2] for p in current_path]

    xs_actual = [p[1] for p in actual_path]
    ys_actual = [p[2] for p in actual_path]

    p = plot(
        xs_plan, ys_plan,
        label="SST Plan",
        linewidth=3,
        marker=:circle,
        aspect_ratio=:equal,
        xlabel="x",
        ylabel="y",
        xlims=(0,10),
        ylims=(0,10),
        title="Plan vs Execution"
    )

    plot!(
        xs_actual, ys_actual,
        label="POMDP Execution",
        linewidth=3,
        marker=:diamond
    )

    plot_obstacles!(obstacles)

    scatter!([start_state[1]], [start_state[2]], label="start")
    scatter!([goal_state[1]], [goal_state[2]], label="goal")

    return p
end

# -------------- Initialize the problem ---------------
start_state, goal_state = load_start_goal(endpoints_file)
obstacles = get_obstacles_csv(obs_file)

set_active_plan(:nominal, start_state)
plot_original_plan()

main_pomdp = QuickPOMDP(

    # Continuous state stored as:
    # ((x, y, theta), mode, num_fails, plan_index, plan_type)
    states = [(start_state, :healthy, 0, 1, :nominal)],
    actions = [:continue_plan, :replan_nominal, :replan_failure],
    observations = [:small_error, :large_error],

    transition = function(s, a)
        x, mode, num_fails, plan_index, plan_type = s

        # Replan under nominal model
        if a == :replan_nominal
            set_active_plan(:nominal, x)
            return Deterministic((x, mode, 0, 1, :nominal))
        # Replan under failure-aware model
        elseif a == :replan_failure
            set_active_plan(:failure, x)
            return Deterministic((x, mode, 0, 1, :failure))

        # Continue current plan
        else
            u = get_planned_control(plan_index)
            # ----- placeholder for what to do if no more controls to execute: 
            duration = get_control_duration(plan_index)

            # If no control left, stay in place and accumulate failure count
            if u === nothing ||  duration === nothing
                return Deterministic((x, mode, min(num_fails + 1, max_fails), plan_index, plan_type))
            end

            # Propagate actual state using current control
            x_next = propagate_unicycle(x, u, mode, duration)
            # Compare against planned next state
            x_plan_next = get_planned_state(plan_index + 1)

            # ----- placeholder for what to do if no more states left: 
            if x_plan_next === nothing
                next_num_fails = min(num_fails + 1, max_fails)
            else
                if large_tracking_error(x_next, x_plan_next)
                    next_num_fails = min(num_fails + 1, max_fails)
                else
                    next_num_fails = 0
                end
            end
            
            # increment plan index 
            next_plan_index = plan_index + 1

            # Hidden mode dynamics: healthy can fail, failed stays failed
            if mode == :healthy
                return SparseCat(
                    [
                        (x_next, :healthy, next_num_fails, next_plan_index, plan_type),
                        (x_next, :turn_bias, next_num_fails, next_plan_index, plan_type)
                    ],
                    [1 - fail_chance, fail_chance]
                )
            else
                return Deterministic((x_next, :turn_bias, next_num_fails, next_plan_index, plan_type))
            end
        end
    end,

    observation = function(a, sp)
        # placeholder for observations
        x, mode, num_fails, plan_index, plan_type = sp
        if in_collision(x, obstacles)
            return Deterministic(:collision_obs)
        elseif reached_goal(x,goal_state)
            return Deterministic(:goal_obs)
        else
            x_plan = get_planned_state(plan_index)
            if x_plan === nothing
                return Deterministic(:large_error)
            else
                if large_tracking_error(x, x_plan)
                    return SparseCat([:large_error, :small_error], [0.90, 0.10])
                else
                    return SparseCat([:small_error, :large_error], [0.90, 0.10])
                end
            end
        end
    end,

    reward = function(s, a, sp)
        x, mode, num_fails, plan_index, plan_type = sp
        x_prev, _, _, _, _ = s

        if in_collision(x, obstacles)
            return -collision_penalty
        elseif reached_goal(x, goal_state)
            return goal_reward
        else
            pos_err_prev, heading_err_prev = tracking_error(x_prev, goal_state)
            pos_err_next, heading_err_next = tracking_error(x, goal_state)

            # Reward progress toward goal?
            progress_reward = (pos_err_prev - pos_err_next) + 0.5 * (heading_err_prev - heading_err_next)

            # Penalize replanning
            if a == :continue_plan
                plan_penalty = 0.0
            else
                plan_penalty = replan_cost
            end

            # Penalize repeated failures
            fail_penalty = num_fails

            return progress_reward - plan_penalty - fail_penalty
        end
    end,

    initialstate = Deterministic((start_state, :healthy, 0, 1, :nominal)),
    discount = 0.95,
    isterminal = s -> in_collision(s[1], obstacles) || reached_goal(s[1], goal_state)
)


# SOlution
function always_continue(mdp, s)
    return :continue_plan
end

# up = DiscreteUpdater(main_pomdp)
replan_states = []
function failure_threshold_policy(mdp, s)
    x, _, num_fails, _, plan_type = s

    if num_fails ≥ 2 && plan_type != :failure
        print("Replan triggered at state = $x, num_fails = $num_fails")
        push!(replan_states, x)
        return :replan_failure
        
    else
        return :continue_plan
    end
end


always_continue_policy = FunctionPolicy(s -> :continue_plan)

set_active_plan(:nominal, start_state)

r, actual_path = rollout_with_path(
    main_pomdp,
    failure_threshold_policy,
    rand(initialstate(main_pomdp)),
    100
)

@show r

display(plot_plan_with_actual(actual_path))

function check_failure_plan_execution()
    println("\n--- Checking failure-aware plan execution ---")

    # Start from a forced failure state
    x0 = start_state
    set_active_plan(:failure, x0)

    u = get_planned_control(1)
    duration = get_control_duration(1)
    x_plan_next = Tuple(get_planned_state(2))

    x_prop_failed = propagate_unicycle(x0, u, :turn_bias, duration)

    @show u
    @show duration
    @show x_prop_failed
    @show x_plan_next
    @show tracking_error(x_prop_failed, x_plan_next)

    @assert !large_tracking_error(x_prop_failed, x_plan_next) "Failure-aware plan does not match failed dynamics"

    println("Failure-aware propagator matches failure-aware plan.")
end

numRuns = 10
maxSteps = 100


# # MC Evaluation
# results_baseline = [
#     begin
#         set_active_plan(:nominal, start_state)
#         simulate(
#             RolloutSimulator(max_steps=maxSteps),
#             mdp,
#             always_continue_policy,
#             rand(initialstate(main_pomdp))
#         )
#     end
#     for _ in 1:numRuns
# ]
# @show mean(results_baseline)
# @show std(results_baseline)