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

function plot_plans!(p, pathHistory)
    # p is a plot object
    for (i, path) in enumerate(pathHistory)
        xs, ys = path_matrix_from_path(path)

        plot!(p,
            xs, ys,
            label=(i == 1) ? "SST Plan" : "",
            linewidth=6,
            color=:red
        )
        # Add X at each replan
        if i < length(pathHistory)
            scatter!(p,
                [xs[end]], [ys[end]],
                marker=:x,
                markersize=7,
                markerstrokewidth=3,
                color=:blue,
                label=(i == 1) ? "Replan Points" : ""
            )
        end
    end

    plot_obstacles!(p, obstacles)

    scatter!([start_state[1]], [start_state[2]], label="start", markersize=6)
    plot_goal_circle!(goal_state)

    return p
end

function plot_plan_with_actual(pathHistory, actual_path; title_str="Actual Path Against Plan Paths")

    xs_actual = [p[1] for p in actual_path]
    ys_actual = [p[2] for p in actual_path]

    pathPlot = plot(aspect_ratio=:equal, xlabel="x", ylabel="y", title=title_str, xlims=(xmin, xmax), ylims=(ymin, ymax), size=(600,600), margin=0Plots.mm, left_margin=2Plots.mm, bottom_margin=2Plots.mm, dpi=300)
    plot_plans!(pathPlot, pathHistory)

    plot!(pathPlot,
        xs_actual, ys_actual,
        label="POMDP Execution",
        linewidth=3,
        color=:black
    )

    return pathPlot
end

function plot_environment()
    pathPlot = plot(aspect_ratio=:equal, xlabel="x", ylabel="y", title="Environment", xlims=(xmin, xmax), ylims=(ymin, ymax), size=(600,600), margin=0Plots.mm, left_margin=2Plots.mm, bottom_margin=2Plots.mm, dpi=300)

    plot_obstacles!(pathPlot, obstacles)

    scatter!([start_state[1]], [start_state[2]], label="start", markersize=6)
    plot_goal_circle!(goal_state)

    return pathPlot
end