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

