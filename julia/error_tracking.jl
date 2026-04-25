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
