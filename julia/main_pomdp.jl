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
        "test.csv",
        "test_ends.csv",
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

# Helper function: decide level of tracking error 
function tracking_error_level(x_actual, x_plan;
                              small_pos_thresh=0.25,
                              med_pos_thresh=0.75,
                              small_heading_thresh=0.15,
                              med_heading_thresh=0.45)

    pos_err, heading_err = tracking_error(x_actual, x_plan)

    if pos_err <= small_pos_thresh && heading_err <= small_heading_thresh
        return :small_error
    elseif pos_err <= med_pos_thresh && heading_err <= med_heading_thresh
        return :medium_error
    else
        return :large_error
    end
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


# Naive Solutions
function always_continue(mdp, s)
    return :continue_plan
end

always_continue_policy = FunctionPolicy(s -> :continue_plan)

set_active_plan(:nominal, start_state)

r, actual_path = rollout_with_path(
    main_pomdp,
    always_continue,
    rand(initialstate(main_pomdp)),
    100
)

@show r

plot_plan_with_actual(actual_path)

