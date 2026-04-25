# Thresholds struct
struct thresholds
    small_pos::Float64
    small_heading::Float64
    med_pos::Float64
    med_heading::Float64
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
function tracking_error_level(x_actual, x_plan; thresholdStruct::thresholds = thresholds(0.25, pi/30, 0.75, pi/6))

    pos_err, heading_err = tracking_error(x_actual, x_plan)

    if pos_err <= thresholdStruct.small_pos && heading_err <= thresholdStruct.small_heading
        return :small_error
    elseif pos_err <= thresholdStruct.med_pos && heading_err <= thresholdStruct.med_heading
        return :medium_error
    else
        return :large_error
    end
end
