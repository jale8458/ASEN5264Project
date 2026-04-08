using QuickPOMDPs: QuickPOMDP
using POMDPTools: Deterministic, Uniform, SparseCat, FunctionPolicy, RolloutSimulator
using Statistics: mean, std
using Plots
import POMDPs
using POMDPs: actions, @gen, isterminal, discount, statetype, actiontype, simulate, states, initialstate

# Load custom ompl planning library into CppOMPL module, if not already loaded
if !isdefined(Main, :CppOMPL)
    include(joinpath(@__DIR__, "loadCppModule.jl"))
end

const max_fails = 5
const dt = 0.1
const turn_bias = 0.35
const collision_penalty = -100.0
const goal_reward = 100.0
const replan_cost = -2.0

# Current active plan returned by planner
current_path = Vector{Vector{Float64}}()
current_controls = Vector{Vector{Float64}}()

# ----- Helper functions -----
# Helper function: wrap heading error to [-pi, pi]
function wrap_angle(theta)
    return atan(sin(theta),cos(theta))
end

# Helper function: set active plan
function set_active_plan(plan_type)
    if plan_type == :nominal
        result = CppOMPL.PlanWithSST("normalParking.csv")
    else
        result = CppOMPL.PlanWithSST("normalParking.csv")   # placeholder for panner with failed dynamics
    end
    global current_path = result.pathPoints
    global current_controls = result.controls
end

# ----- Need helper functions to get planned state at next index -----
# Helper function: get planned state at index k
function get_planned_state()
# Helper function: get planned control at index k
function get_planned_control()

# Helper function: propagate actual state using unicycle dynamics
function propagate_unicycle(x,u,mode)
    px, py, theta = x
    v=  u[1]
    omega = u[2]

    if mode == :healthy
        omega_eff = omega
    else
        omega_eff = omega+turn_bias
    end

    x_next = px+v *cos(theta)*dt
    y_next = py +v *sin(theta)*dt
    theta_next = wrap_angle(theta+omega_eff*dt)

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
function tracking_error(x_actual, x_plan)

# Helper function: collision check placeholder
function in_collision(x)

# Helper function: goal check 
function reached_goal(x)

main_pomdp = QuickPOMDP(

    # Continuous state stored as:
    # ((x, y, theta), mode, num_fails, plan_index, plan_type)
    states = [((0.0, 7.0, 0.0), :healthy, 0, 1, :nominal)],
    actions = [:continue_plan, :replan_nominal, :replan_failure],
    observations = [:small_error, :large_error, :collision_obs, :goal_obs],

    transition = function(s, a)
        x, mode, num_fails, plan_index, plan_type = s

        # Replan under nominal model
        if a == :replan_nominal
            set_active_plan(:nominal)
            return Deterministic((x, mode, num_fails, 1, :nominal))
        # Replan under failure-aware model
        elseif a == :replan_failure
            set_active_plan(:failure)
            return Deterministic((x, mode, num_fails, 1, :failure))

        # Continue current plan
        else
            u = get_planned_control(plan_index)
            # ----- placeholder for what to do if no more controls to execute: 
            # If no control left, stay in place and accumulate failure count
            if u === nothing
                return Deterministic((x, mode, min(num_fails + 1, max_fails), plan_index, plan_type))
            end

            # Propagate actual state using current control
            x_next = propagate_unicycle(x, u, mode)
            # Compare against planned next state
            x_plan_next = get_planned_state(plan_index + 1)

            # ----- placeholder for what to do if no more states left: 
            if x_plan_next === nothing
                next_num_fails = num_fails


            else
                if large_tracking_error(x_next, x_plan_next)
                    next_num_fails = min(num_fails + 1, max_fails)
                else
                    next_num_fails = 0
                end
            end

            # Hidden mode dynamics: healthy can fail, failed stays failed
            if mode == :healthy
                return SparseCat(
                    [
                        (x_next, :healthy, next_num_fails, plan_index + 1, plan_type),
                        (x_next, :turn_bias, next_num_fails, plan_index + 1, plan_type)
                    ],
                    [0.99, 0.01]
                )
            else
                return Deterministic((x_next, :turn_bias, next_num_fails, plan_index + 1, plan_type))
            end
        end
    end,

    observation = function(a, sp)
        # placeholder for observations
        x, mode, num_fails, plan_index, plan_type = sp
        if in_collision(x)
            return Deterministic(:collision_obs)
        elseif reached_goal(x)
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

        if in_collision(x)
            return -collision_penalty
        elseif reached_goal(x)
            return goal_reward
        else
            goal = (5.0, 4.0, -pi/2)

            pos_err_prev, heading_err_prev = tracking_error(x_prev, goal)
            pos_err_next, heading_err_next = tracking_error(x, goal)

            # Reward progress toward goal?
            progress_reward = (pos_err_prev - pos_err_next) + 0.5*(heading_err_prev - heading_err_next)

            # Penalize replanning
            if a == :continue_plan
                plan_penalty = 0.0
            else
                plan_penalty = replan_cost
            end

            # Penalize repeated failures
            fail_penalty = num_fails

            return progress_reward + plan_penalty + fail_penalty
        end
    end,

    initialstate = Deterministic(((0.0, 7.0, 0.0), :healthy, 0, 1, :nominal)),
    discount = 0.95,
    isterminal = s -> in_collision(s[1]) || reached_goal(s[1])
)
