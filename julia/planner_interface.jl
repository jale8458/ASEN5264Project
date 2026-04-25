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