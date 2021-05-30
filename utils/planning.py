from bresenham import bresenham
from networkx.generators import line
from config.config import DRONE_MAX_ALTITUDE, OBSTACLE_DATA_FILE
from utils.frames import *
from utils.mapping import get_obstacle_data, get_obstacle_polygons
import networkx as nx
import numpy as np
from numpy.core.defchararray import index
import random
from shapely.geometry import LineString, Polygon
from skimage.morphology import medial_axis
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
from sklearn.neighbors import KDTree
from skimage.util import invert
import time
from typing import List, Iterable, Set, Tuple


def get_random_free_point(grid: np.array) -> Tuple[int, int]:
    """Get random free point in an obstacle grid."""

    while True:

        point = (np.random.randint(grid.shape[0]),
                 np.random.randint(grid.shape[1]))

        if not point_is_in_collision(point, grid):

            return point



def point_is_in_collision(point: Iterable[int],
                          grid: np.array
                          ) -> bool:
    """Check if a point in the local grid frame collides with an obstacle."""
    
    in_collision = bool(grid[point[0],point[1]])

    return in_collision


def line_is_in_collision(point_a: Iterable[int],
                         point_b: Iterable[int],
                         grid: np.array
                         ) -> bool:
    """Check if a line in a local grid frame collides with an obstacle."""                 

    # Indexes of points between both points
    idx = np.array(list(bresenham(point_a[0], point_a[1], point_b[0], point_b[1])))

    # The variable below will be "True" if any of the indexes found above
    # correspond to a "1" in grid, which means there is an obstacle
    in_collision = np.any(grid[idx[:,0],idx[:,1]])

    return in_collision           


def get_median_axes(grid: np.array) -> np.array:

    # Median axes
    # (2D array, same size as grid, True = medial axis location)
    median_axes_grid = medial_axis(invert(grid))

    # Indices of non-zero items
    # (2D array, each line is an array [index_x,index_y] of a True item)

    median_axes_idx = np.transpose(np.nonzero(median_axes_grid))   

    return median_axes_idx


def get_nearest_state(state: Tuple[int, int],
                      states: Iterable[Tuple[int,int]]
                      ) -> Tuple[Tuple[int,int], float]:

    # KDTree requires numpy array
    # so convert inputs

    state = np.array(state)
    states = np.array(states)

    tree = KDTree(states)
    d, idx = tree.query([state], k=1)

    # Keep only info for 1st item
    d = d[0][0]
    idx = idx[0][0]

    nearest_state = tuple(states[idx])

    return (nearest_state, d)


def rrt(state_space: Set[Tuple[int,int]],
        start_state: Tuple[int, int],
        goal_state: Tuple[int, int],
        grid: np.array,
        d_max: float
        ) -> List[Tuple[int,int]]:
    """Derive a path using the RRT algorithm."""

    # Convert inputs to arrays

    state_space = np.array(list(state_space))
    start_state = np.array(start_state, dtype=int)
    goal_state = np.array(goal_state, dtype=int)

    n_states = state_space.shape[0]

    # Initalize path, nodes and edges.

    # Notes
    #
    # state = grid location the drone can be at (array, sometimes converted to tuple)
    # path  = sequence of states from start state to goal state (array)
    # node  = state which is also part of the graph (array)
    # edge = segment connecting 2 nodes (array)

    nodes = np.array([ start_state ] , dtype=int)
    edges = np.array([ [ start_state, start_state ] ], dtype=int)

    i_max = 100000
    print_interval = 100
    for i in range(i_max):

        # Print user message every other interval
        if not ( i % print_interval):
            print("Searching for path - Iteration ", i, " / ", i_max)
        
        # Select random state in state space
        # (skip if state already in graph)

        idx = random.randrange(n_states)
        new_state = state_space[idx,:]

        # Check if new_state is already in the graph
        # (i.e. check if new_state is a row of nodes)        
        if np.where(nodes==new_state, True, False).all(axis=1).any():       
            continue 

        # Select node in graph nearest to new state
        # (skip if state beyond max allowable disance)

        # Distances to all visited states
        d = np.linalg.norm( nodes-new_state, axis=1 )   
        # Index of nearest visited state
        index_nearest = np.argmin(d)   
        # Nearest state within max distance?
        if d[index_nearest] > d_max:
            continue       
        nearest_state = nodes[index_nearest]

        # Check if any obstacles between nearest state and new state
        # (skip if obstacle)
        if line_is_in_collision(nearest_state, new_state, grid):
            continue

        # Add new node and edge
        nodes = np.append(nodes, [new_state], axis=0)
        edges = np.append(edges, [[nearest_state, new_state]], axis=0)

        # Check if found goal state
        if np.array_equal(new_state, goal_state):
            break

    # Build path

    g = nx.DiGraph()

    for n in nodes:
        g.add_node(tuple(n))

    for e in edges:
        g.add_edge(tuple(e[0]), tuple(e[1]))    

    path = []
    n = tuple(goal_state)
    path.append(n)

    while n != tuple(start_state):
        
        n = next(g.predecessors(tuple(n)))
        path.append(n)

    path.reverse()    
    path = np.array(path)

    return path, g


def get_optimized_path(path: np.array, grid: np.array) -> np.array:
    """Remove intermediate nodes from a path."""

    # Number of nodes in path
    n = path.shape[0]

    # Special case
    # If path has less than 3 nodes, return it as-is
    if n < 3:
        return path

    # Indexes of nodes to keep after optimization
    # (initialized as path first node index)
    indices_to_keep = np.array([0])

    # Loop through all nodes.
    # Check if any intermediate node_b can be ignored.

    # node_a -> node on the path -> index i
    # node_b -> node after after node_a
    #        -> node before node_c -> index j-1
    # node_c -> node after node_b -> index j
    
    i = -1
    while True:

        # Increment the index of node_a
        i = i+1

        # Stop optimizing if there are less than
        # 2 nodes beyond node_a        
        if i > (n-3):
            break

        # Select node_a
        node_a = path[i]

        for j in range(i+2,n):

            node_c = path[j]

            # Check if simplifying the path by removing
            # node_b would cause a collision
            if line_is_in_collision(node_a, node_c, grid):
                
                # node_b cannot be removed.
                # Add its index to the list of indices to keep.
                indices_to_keep = np.append(indices_to_keep, j-1)   

                # Make node_a equal to node_b.
                i = j-2   # j-2 because i will be incremented by 1 before selecting node_a again
                
                # Skip back to the outer loop.
                break
            else:
                
                # node_b can be removed.
                # Skip to the next inner loop iteration, moving
                # node_c one step towards the end of the path.
                continue

    # Add path last node index to list of indices to keep
    indices_to_keep = np.append(indices_to_keep, j)

    # Build optimized path from initial path using indices to keep.
    optimized_path = path[indices_to_keep,:]

    return optimized_path














def get_waypoints_from_path(path: Iterable[Tuple[int,int]],
                            global_alt: int,
                            north_min: float,
                            east_min: float,
                            cell_size: float
                            ) -> np.array: # np.array[int,[Any,4]]
    """Build waypoints from a path."""

    # Waypoints are actually a list of position commands.
    # Position commands are expressed in a local NEU (North-East-Up) frame,
    # plus a heading command.
    #
    # [ north, east, alt (positive up), heading]
    #
    # north = north location (integer, meters)
    # east = east location (integer, meters)
    # alt = altitude (positive up, integer, meters) - same as altitude in global geodetic frame
    # heading = orientation (*), integer, radians
    #
    # (*) The udacidrone API documentation is not clear on how this heading is measured.
    #     In the starter code provided by Udacity, this value was left to 0,
    #     which makes sense for a waypoint since we only want a location in space.
    #     So heading will be set to 0 in this function.
    #
    # The path is made of points expressed in the local grid frame,
    # so it is an iterable of tuples of 2 integers.
    
    # Convert path nodes from grid frame to local ECEF frame
    local_path = [ grid_to_local(p, (-1)*global_alt, north_min, east_min, cell_size) for p in path ]

    # Convert to waypoint frame
    int_global_alt = int(global_alt)
    waypoints = [ [ int(p[0]), int(p[1]), int_global_alt, 0 ] for p in local_path  ]

    return np.array(waypoints,dtype=int)