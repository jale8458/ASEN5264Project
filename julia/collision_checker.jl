# ----- Helper functions -----
# Get obstacles from file
function get_obstacles_csv(filename)
    obstacles = Vector{Vector{Vector{Float64}}}()

    for line in eachline(filename)
        isempty(strip(line)) && continue

        parts = split(line, ",")
        obstacle_num = parse(Int, strip(parts[1]))
        x = parse(Float64, strip(parts[2]))
        y = parse(Float64, strip(parts[3]))

        while length(obstacles) < obstacle_num + 1
            push!(obstacles, Vector{Vector{Float64}}())
        end

        push!(obstacles[obstacle_num + 1], [x, y])  # Julia is 1-indexed
    end

    return obstacles
end

# Collison checkers
function point_in_polygon(point, polygon)
    px, py = point
    inside = false
    n = length(polygon)

    j = n
    for i in 1:n
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        if ((yi > py) != (yj > py)) &&
           (px < (xj - xi) * (py - yi) / (yj - yi) + xi)
            inside = !inside
        end

        j = i
    end

    return inside
end

function in_collision(x, obstacles)
    px, py, theta = x

    if px < xmin || px > xmax || py < ymin || py > ymax
        return true
    end

    for obstacle in obstacles
        if point_in_polygon((px, py), obstacle)
            return true
        end
    end

    return false
end

# Get start and endpoint from csv file:
function load_start_goal(filename)
    start = nothing
    goal = nothing

    for line in eachline(filename)
        isempty(strip(line)) && continue

        vals = split(line, ",")
        label = Symbol(strip(vals[1]))
        x = parse(Float64, strip(vals[2]))
        y = parse(Float64, strip(vals[3]))
        theta = parse(Float64, strip(vals[4]))

        if label == :start
            start = (x, y, theta)
        elseif label == :goal
            goal = (x, y, theta)
        end
    end

    return start, goal
end

function reached_goal(x, goal_state; pos_tol=0.5, heading_tol=pi/12.0)
    pos_err, heading_err = tracking_error(x, goal_state)
    return pos_err <= pos_tol && heading_err <= heading_tol
end

