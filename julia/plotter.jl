############
# Plotting helpers
############
function path_matrix_from_path(path)
    xs = [p[1] for p in path]
    ys = [p[2] for p in path]
    return xs, ys
end

function plot_obstacles!(plot, obstacles)
    for obs in obstacles
        xs = [p[1] for p in obs]
        ys = [p[2] for p in obs]

        # close polygon
        push!(xs, obs[1][1])
        push!(ys, obs[1][2])

        plot!(plot, xs, ys, label=false, linewidth=2)
    end
end

function plot_plans!(p; title_str="SST Plans")
    # p is a plot object
    for (i, path) in enumerate(pathHistory)
        xs, ys = path_matrix_from_path(path)

        plot!(p,
            xs, ys,
            label="SST Plan $i",
            linewidth=3,
            marker=:circle,
            aspect_ratio=:equal,
            xlabel="x",
            ylabel="y",
            title=title_str,
            xlims=(0, 10),
            ylims=(0, 10)
        )
    end

    plot_obstacles!(p, obstacles)

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

    xs_actual = [p[1] for p in actual_path]
    ys_actual = [p[2] for p in actual_path]

    pathPlot = plot()
    plot_plans!(pathPlot)

    plot!(pathPlot,
        xs_actual, ys_actual,
        label="POMDP Execution",
        linewidth=3,
        marker=:diamond
    )

    plot_obstacles!(pathPlot, obstacles)

    return pathPlot
end

