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

function plot_goal_circle!(goal_state, pos_tol = 0.5; label="goal", color=:green, alpha = 0.3)
    theta = range(0, 2π, length=100)
    x = goal_state[1] .+ pos_tol .* cos.(theta)
    y = goal_state[2] .+ pos_tol .* sin.(theta)
    plot!(Shape(x, y), label=false, color=color, fillalpha=alpha)
    scatter!([goal_state[1]], [goal_state[2]], label=label, color=color, markersize=4)
end

function plot_plans!(p, pathHistory; title_str="SST Plans")
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
    plot_goal_circle!(goal_state)

    return p
end

function plot_plan_with_actual(pathHistory, actual_path)

    xs_actual = [p[1] for p in actual_path]
    ys_actual = [p[2] for p in actual_path]

    pathPlot = plot(size=(600,600), margin=0Plots.mm, left_margin=2Plots.mm, bottom_margin=2Plots.mm, dpi=300)
    plot_plans!(pathPlot, pathHistory)

    plot!(pathPlot,
        xs_actual, ys_actual,
        label="POMDP Execution",
        linewidth=3,
        marker=:diamond
    )

    return pathPlot
end

