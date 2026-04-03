import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrow
from matplotlib.animation import FuncAnimation
import numpy as np

########## Normal plotting paths with obstacles

def plotPath2D(path, lowBounds, highBounds, q_init = None, q_goal = None):

    fig, ax = plt.subplots()
    
    # Plots 2D path from init to goal
    xPath = [p[0] for p in path]
    yPath = [p[1] for p in path]
    ax.plot(xPath, yPath, marker='o', zorder=2)
    if q_init is not None:
        ax.scatter(q_init[0], q_init[1], c='green', label='Start', zorder=3)
        ax.text(q_init[0]-0.05, q_init[1]+0.05, 'Start', fontsize=8)
    if q_goal is not None:
        ax.scatter(q_goal[0], q_goal[1], c='green', label='Goal', zorder=3)
        ax.text(q_goal[0]-0.05, q_goal[1]+0.05, 'Goal', fontsize=8)

    ax.autoscale(False)
    ax.set_xlim(lowBounds[0],highBounds[0])
    ax.set_ylim(lowBounds[1],highBounds[1])
    
    return fig, ax

def plotPathWithObstacles2D(path, lowBounds, highBounds, obstacles, q_init = None, q_goal = None):

    fig, ax = plotPath2D(path, lowBounds, highBounds, q_init, q_goal)
    print("Plotted Path")
    # Plot obstacles as polygons
    for obstacle in obstacles:
        obstaclePolygon = Polygon(obstacle, facecolor = '#187678', edgecolor='black',)
        ax.add_patch(obstaclePolygon)

def plotObstacles2D(q_init, q_goal, lowBounds, highBounds, obstacles):
    fig, ax = plt.subplots()
    ax.autoscale(False)

    ax.scatter(q_init[0], q_init[1], c='green', label='Start', zorder=3)
    ax.scatter(q_goal[0], q_goal[1], c='green', label='Goal', zorder=3)
    ax.text(q_init[0]-0.05, q_init[1]+0.05, 'Start', fontsize=8)
    ax.text(q_goal[0]-0.05, q_goal[1]+0.05, 'Goal', fontsize=8)
    
    ax.set_xlim(lowBounds[0],highBounds[0])
    ax.set_ylim(lowBounds[1],highBounds[1])

    # Plot obstacles as polygons
    for obstacle in obstacles:
        obstaclePolygon = Polygon(obstacle, facecolor = '#187678', edgecolor='black',)
        ax.add_patch(obstaclePolygon)

########## Plotting Car Path ##########

def getCarCorners(x, y, yaw, carLength, carWidth):
    cl = carLength / 2.0
    cw = carWidth / 2.0
    cosT = np.cos(yaw)
    sinT = np.sin(yaw)
    corners = np.array([
        [x + cl*cosT - cw*sinT,  y + cl*sinT + cw*cosT],  # front-left
        [x + cl*cosT + cw*sinT,  y + cl*sinT - cw*cosT],  # front-right
        [x - cl*cosT + cw*sinT,  y - cl*sinT - cw*cosT],  # rear-right
        [x - cl*cosT - cw*sinT,  y - cl*sinT + cw*cosT],  # rear-left
    ])
    return corners

def plotCarPath2D(path, lowBounds, highBounds, carLength, carWidth):
    fig, ax = plt.subplots()

    # Plot path line
    xPath = [p[0] for p in path]
    yPath = [p[1] for p in path]
    ax.plot(xPath, yPath, 'b--', alpha=0.4, zorder=1)

    # Draw car at each state
    for p in path:
        x, y, yaw = p[0], p[1], p[2]
        corners = getCarCorners(x, y, yaw, carLength, carWidth)
        car = Polygon(corners, closed=True, facecolor='lightblue', edgecolor='blue', alpha=0.4, zorder=2)
        ax.add_patch(car)

    # Draw first state in path as start
    if len(path) > 0:
        first = path[0]
        corners = getCarCorners(first[0], first[1], first[2], carLength, carWidth)
        car = Polygon(corners, closed=True, facecolor='green', edgecolor='darkgreen', alpha=0.8, zorder=3)
        ax.add_patch(car)
        ax.text(first[0], first[1], 'Start', fontsize=8, zorder=4)

    # Draw last state in path as end
    if len(path) > 0:
        last = path[-1]
        corners = getCarCorners(last[0], last[1], last[2], carLength, carWidth)
        car = Polygon(corners, closed=True, facecolor='red', edgecolor='darkred', alpha=0.8, zorder=3)
        ax.add_patch(car)
        ax.text(last[0], last[1], 'End', fontsize=8, zorder=4)

    ax.autoscale(False)
    ax.set_xlim(lowBounds[0], highBounds[0])
    ax.set_ylim(lowBounds[1], highBounds[1])

    return fig, ax

def plotCarPathWithObstacles2D(path, lowBounds, highBounds, obstacles, carLength, carWidth):
    fig, ax = plotCarPath2D(path, lowBounds, highBounds, carLength, carWidth)
    print("Plotted Path")
    # Plot obstacles as polygons
    for obstacle in obstacles:
        obstaclePolygon = Polygon(obstacle, facecolor = '#187678', edgecolor='black',)
        ax.add_patch(obstaclePolygon)

########## Animate car path ##########

def animateCarPath2D(path, lowBounds, highBounds, obstacles, carLength, carWidth, interval=100, save_path=None):
    fig, ax = plt.subplots()
    ax.set_xlim(lowBounds[0], highBounds[0])
    ax.set_ylim(lowBounds[1], highBounds[1])
    ax.set_aspect('equal')

    # Plot obstacles as polygons
    for obstacle in obstacles:
        obstaclePolygon = Polygon(obstacle, facecolor = '#187678', edgecolor='black',)
        ax.add_patch(obstaclePolygon)

    # Plot full path line as background
    xPath = [p[0] for p in path]
    yPath = [p[1] for p in path]
    ax.plot(xPath, yPath, 'b--', alpha=0.2, zorder=1)

    # Animated car patch
    car_patch = Polygon(getCarCorners(path[0][0], path[0][1], path[0][2], carLength, carWidth),
                        closed=True, facecolor='lightblue', edgecolor='blue', zorder=3)
    ax.add_patch(car_patch)

    def update(frame):
        p = path[frame]
        corners = getCarCorners(p[0], p[1], p[2], carLength, carWidth)
        car_patch.set_xy(corners)
        return car_patch,

    anim = FuncAnimation(fig, update, frames=len(path), interval=interval, blit=True)

    # Save animation, if applicable
    if save_path is not None:
        if save_path.endswith('.mp4'):
            anim.save(save_path, writer='ffmpeg')
    return fig, anim


########## Other misc funcs ##########

def plot_boxplot(data_arrays, labels=None, title=None, ylabel=None, xlabel=None):
    """
    data_arrays : list[list[float]]
        Each element is one dataset to appear as a box in the boxplot.
    labels : list[str] or None
        Labels for each dataset.
    title : str or None
        Plot title.
    ylabel : str or None
        Label for y-axis.
    """
    fig, ax = plt.subplots()
    ax.boxplot(data_arrays, labels=labels)
    
    if title:
        ax.set_title(title)
    if ylabel:
        ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_xlabel(xlabel)

    # plt.grid(True, linestyle="--", alpha=0.5)
    ax.tight_layout()

def plot_barGraph(data_array, labels=None, title=None, ylabel=None, xlabel=None):
    """
    data_arrays : list[float]
        Each element is one dataset to appear as a box in the boxplot.
    labels : list[str] or None
        Labels for each dataset.
    title : str or None
        Plot title.
    ylabel : str or None
        Label for y-axis.
    """
    fig, ax = plt.subplots()

    if labels is not None:
        ax.bar(labels, data_array)
    else:
        ax.bar(range(len(data_array)), data_array)
    
    if title:
        ax.set_title(title)
    if ylabel:
        ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_xlabel(xlabel)

    # plt.grid(True, linestyle="--", alpha=0.5)
    ax.tight_layout()


def showFigures():
    plt.show()


