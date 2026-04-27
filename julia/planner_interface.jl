
# Helper function: Create an active plan
function create_plan(mode, x; plan_time = 5.0, logOutput = true)
    if mode == :healthy
        angle_bias = 0.0
    elseif mode == :turn_bias
        angle_bias = turn_bias
    else
        error("Unknown planning mode in create_plan call")
    end

    control, controlDurations, path = Main.CppOMPL.PlanWithSSTFromState(obs_file, endpoints_file, x[1], x[2], x[3], angle_bias, plan_time, logOutput)

    # Number of timesteps to execute each control
    nTimesteps = round.(Int,controlDurations/dt)

    # Split all controls into dt = 0.1 timesteps
    current_controls = [control[:, i] for i in 1:size(control, 2) for _ in 1:nTimesteps[i]]
    # Repropagate dynamics at resolution dt = 0.1
    current_path = [collect(x)]
    for ctrl in current_controls
        push!(current_path, collect(propagate_unicycle(current_path[end], ctrl, mode, dt)))
    end

    return current_controls, current_path
end

# ----- Need helper functions to get planned state at next index -----
# Helper function: get planned state at index k
function get_planned_state(current_path, k)
    if 1 <= k <= length(current_path)
        return current_path[k]
    else
        return nothing
    end
end

# Helper function: get planned control at index k
function get_planned_control(current_controls, k)
    if 1 <= k <= length(current_controls)
        return current_controls[k]
    else
        return nothing
    end
end