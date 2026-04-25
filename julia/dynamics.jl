# Helper function: propagate actual state using unicycle dynamics
function unicycle_dynamics(x,u,mode)
    px, py, theta = x
    v=  u[1]
    omega = u[2]

    if mode == :healthy
        omega_eff = omega
    elseif mode == :turn_bias
        omega_eff = omega + turn_bias
    else
        error("Unknown bias mode in dynamics")
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


# Helper function: wrap heading error to [-pi, pi]
function wrap_angle(theta)
    return atan(sin(theta),cos(theta))
end
