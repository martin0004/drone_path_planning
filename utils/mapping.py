import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from shapely.geometry import Polygon
from typing import List, Set, Tuple


def get_obstacle_data(file_path: str) -> np.array:
    """Load csv data representing obstacles centers and half sizes."""

    # Ignore 2 first lines, which contain headers.
    obstacles_data = np.loadtxt(file_path, delimiter=",", skiprows=2)

    return obstacles_data


def get_obstacle_polygons(obstacle_data: np.array) -> List[Tuple[Polygon, float]]:
    """Build polygons representing obstacles."""

    obstacle_polygons = []

    for i in range(obstacle_data.shape[0]):

        north, east, alt, north_half, east_half, alt_half = obstacle_data[i,:]

        p1 = (north - north_half, east - east_half)
        p2 = (north - north_half, east + east_half)
        p3 = (north + north_half, east + east_half)
        p4 = (north + north_half, east - east_half)

        alt_max = alt + alt_half

        corners = [p1, p2, p3, p4]

        poly = Polygon( corners )
        obstacle_polygons.append( (poly,alt_max) )

    return obstacle_polygons


def get_local_map_size(obstacle_data: np.array):
    """Map min/max coordinates in local ECEF frame."""

    north_min = np.floor( np.min(obstacle_data[:,0] - obstacle_data[:,3] ) )
    north_max = np.ceil( np.max(obstacle_data[:,0] + obstacle_data[:,3] ) )

    east_min = np.floor( np.min(obstacle_data[:,1] - obstacle_data[:,4] ) )
    east_max = np.ceil( np.max(obstacle_data[:,1] + obstacle_data[:,4] ) )

    return (north_min, north_max, east_min, east_max)


def get_grid(obstacle_data: np.array,
             altitude: float,
             safety_distance: float,
             cell_size: float) -> np.array:
    """Build a 2D grid representing free space and obstacles
       at a specific altitude.

    Inputs:

        obstacle_data:    obstacle centers and half sizes [m]
        altitude:         altitude [m]
        safety_distance:  safety distance from obstacles [m]
        cell_size:        size of 2D cell [m]

    Outputs:

        grid:             2D array
                              values: 0 = free space, 1 = obstacle
                              axes:   0 = north, 1 = east

    """

    # Sanity check: altitude >= 0.
    if altitude < 0.0:
        print("Warning: input altitude negative, set to 0.0")
        altitude = 0.0

    # Map min/max coordinates in local ECEF frame.
    north_min, north_max, east_min, east_max = get_local_map_size(obstacle_data)

    # Grid dimensions
    n_north = int(np.ceil((north_max - north_min) / cell_size))
    n_east = int(np.ceil((east_max - east_min) / cell_size))

    # Initialize grid
    grid = np.zeros((n_north, n_east))

    # Populate grid with obstacles

    for obs in obstacle_data:

        north, east, alt, north_half, east_half, alt_half = obs

        # Min/max indices which bound obstacle in grid.

        if  ( altitude <= (alt + alt_half + safety_distance) ) \
         and ( altitude >= (alt - alt_half - safety_distance) ):

            north_idx_min = int(np.floor((north - north_half - safety_distance - north_min) / cell_size))
            north_idx_max = int(np.ceil((north + north_half + safety_distance - north_min) / cell_size))
            east_idx_min  = int(np.floor((east - east_half - safety_distance - east_min) / cell_size))
            east_idx_max  = int(np.ceil((east + east_half + safety_distance - east_min) / cell_size))

            grid[north_idx_min:north_idx_max+1, east_idx_min:east_idx_max+1] = 1

    return grid


def show_map(grid: np.array,
             grid_home: Tuple[int, int] = None,
             grid_start: Tuple[int, int] = None,
             grid_goal: Tuple[int, int] = None,             
             median_axes: np.array = None,
             state_space: Set[Tuple[int,int]] = None,
             graph: nx.DiGraph = None,
             path: List[Tuple[int,int]] = None,
             altitude: float = None,
             title: str = None):
    """Displays the 2D map of the planning problem."""

    fig = plt.figure()

    # Grid of free space and obstacles

    ax = plt.subplot(1,1,1)
    ax.imshow(grid, cmap="Greys", origin="lower")

    # Title

    if title is not None:
        if altitude is not None:
            title += "\n Altitude = " + str(altitude) + " m"
        ax.set_title(title)

    ax.set_xlabel("East [grid index]")
    ax.set_ylabel("North [grid index]")

    # Median axes

    if median_axes is not None:
        ax.scatter(median_axes[:,1], median_axes[:,0], s=1, c="b", marker=".", label="Median Axes") 

    # State space

    if state_space is not None:

        # Convert state space to numpy array
        l = list(state_space)
        a = np.array(l)

        ax.scatter(a[:,1], a[:,0], s = 5, c = "m", marker = "o", label="State Space")   

    # Graph

    if graph is not None:

        for (n1, n2) in graph.edges():

            ax.plot([ n1[1], n2[1] ], [ n1[0], n2[0] ], c="y")    

    # Path

    if path is not None:

        # Convert path to numpy array
        a = np.array(path)

        ax.plot(a[:,1], a[:,0], c="r", label="Path")


    # Home / start / goal locations

    # Note: local grid frame -> axis 0 = north, 1 = east
    #       matplotlib chart -> axis 0 = horizontal, 1 = vertical
    #       so there is a little inversion to do when using ax.plot...

    if grid_home is not None:
        ax.scatter(grid_home[1], grid_home[0], s=100, c="r", marker="s", label="Home")

    if grid_start is not None:    
        ax.scatter(grid_start[1], grid_start[0], s=200, c="g", marker="*", label="Start")

    if grid_goal is not None:    
        ax.scatter(grid_goal[1], grid_goal[0], s=200, c="b", marker="*", label="Goal")


    # Show map!

    plt.legend()
    plt.show()



