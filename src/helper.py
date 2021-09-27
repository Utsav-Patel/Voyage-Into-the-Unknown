import numpy as np
import random
from src.Maze import Maze
from queue import Queue, PriorityQueue
from constants import NUM_COLS, NUM_ROWS, INF, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, X, Y
from sortedcontainers import SortedSet
import matplotlib.pyplot as plt


def avg(lst: list):
    """
    This function computes average of the given list. If the length of list is zero, it will return zero.
    :param lst: list for which you want to compute average
    :return: average of the given list
    """
    if len(lst) == 0:
        return 0
    return sum(lst) / len(lst)


def generate_grid_manually():
    """
    This is the function to generate grid manually. This is helpful for the initial testing and problem 1.
    :return: Manually generated numpy array.
    """
    array = np.zeros((5, 5))

    array[1][1] = 1
    array[1][2] = 1
    array[1][3] = 1
    array[2][3] = 1
    array[3][3] = 1
    array[3][4] = 1
    return array


def generate_grid_with_probability_p(p):
    """
    This function will generate the uniform random grid of size NUM_ROWS X NUM_COLS.
    :param p: probability of cell being blocked
    :return: Grid of size NUM_ROWS X NUM_COLS with each cell having uniform probability of being blocked is p.
    """
    from constants import NUM_COLS, NUM_ROWS
    randomly_generated_array = np.random.uniform(low=0.0, high=1.0, size=NUM_ROWS * NUM_COLS).reshape(NUM_ROWS,
                                                                                                      NUM_COLS)
    randomly_generated_array[STARTING_POSITION_OF_AGENT[0]][STARTING_POSITION_OF_AGENT[1]] = 0
    randomly_generated_array[GOAL_POSITION_OF_AGENT[0]][GOAL_POSITION_OF_AGENT[1]] = 0
    randomly_generated_array[randomly_generated_array < (1 - p)] = 0
    randomly_generated_array[randomly_generated_array >= (1 - p)] = 1
    return randomly_generated_array


def compute_trajectory_length_from_path(paths: list):
    """
    This function will compute the trajectory length from the list of paths returned by any repeated forward algorithm
    :param paths: list of paths
    :return: trajectory length
    """

    trajectory_length = 0
    for path in paths:
        trajectory_length += len(path)
    trajectory_length -= len(paths)
    return trajectory_length


def length_of_path_from_source_to_goal(maze_array: np.array, start_pos: tuple, goal_pos: tuple):
    """
    This function will return length of path from source to goal if it exists otherwise it will return INF
    :param maze_array: binary Maze Array
    :param start_pos: Starting position of the maze from where you want to start
    :param goal_pos: Goal position of the maze where you want to reach
    :return: Shortest distance from the source to goal on the given maze array
    """

    # Initialize queue to compute distance
    q = Queue()

    # Initialize distance array
    distance_array = np.full((NUM_ROWS, NUM_COLS), INF)

    # Adding starting position to the queue and assigning its distance to zero
    q.put(start_pos)
    distance_array[start_pos[0]][start_pos[1]] = 0

    # Keep popping value from the queue until it gets empty
    while not q.empty():
        current_node = q.get()

        # If goal position is found, we should return its distance
        if current_node == goal_pos:
            return distance_array[goal_pos[0]][goal_pos[1]]

        # Iterating over valid neighbours of current node
        for ind in range(len(X)):
            neighbour = (current_node[0] + X[ind], current_node[1] + Y[ind])
            if check(neighbour, NUM_COLS, NUM_ROWS) and \
                    (distance_array[neighbour[0]][neighbour[1]] > distance_array[current_node[0]][current_node[1]] + 1) \
                    and (maze_array[neighbour[0]][neighbour[1]] == 0):
                q.put(neighbour)
                distance_array[neighbour[0]][neighbour[1]] = distance_array[current_node[0]][current_node[1]] + 1

    return distance_array[goal_pos[0]][goal_pos[1]]


def manhattan_distance(pos1: tuple, pos2: tuple):
    """
    Compute Manhattan distance between two points
    :param pos1: Coordinate of first point
    :param pos2: Coordinate of second point
    :return: Manhattan distance between two points
    """
    distance = 0
    for ind in range(len(pos1)):
        distance += abs(pos1[ind] - pos2[ind])
    return distance


def euclidean_distance(pos1: tuple, pos2: tuple):
    """
    Compute Euclidean distance between two points
    :param pos1: Coordinate of first point
    :param pos2: Coordinate of second point
    :return: Euclidean distance between two points
    """
    distance = 0
    for ind in range(len(pos1)):
        distance += (pos1[ind] - pos2[ind]) * (pos1[ind] - pos2[ind])
    return np.sqrt(distance)


def chebyshev_distance(pos1: tuple, pos2: tuple):
    """
    Compute Chebyshev distance between two points
    :param pos1: Coordinate of first point
    :param pos2: Coordinate of second point
    :return: Chebyshev distance between two points
    """
    distance = 0
    for ind in range(len(pos1)):
        distance = max(distance, abs(pos1[ind] - pos2[ind]))
    return distance


def h_function(function_for_h):
    """
    Find and return appropriate function to compute heuristic distance from the target (goal).
    :return: H function
    """
    if function_for_h == 'manhattan':
        return manhattan_distance
    elif function_for_h == 'euclidean':
        return euclidean_distance
    elif function_for_h == 'chebychev':
        return chebyshev_distance
    else:
        raise Exception("Function for H is not exist. Please choose from manhattan, euclidean, or chebychev")


def check(pos: tuple, num_cols: int, num_rows: int):
    """
    Check whether current point is in the grid or not
    :param pos: current point
    :param num_cols: Number of columns of the grid
    :param num_rows: Number of rows of the grid
    :return: True if the current point is in the grid otherwise False
    """
    if (0 <= pos[0] < num_rows) and (0 <= pos[1] < num_cols):
        return True
    return False


def compute_heuristics(maze: Maze, goal_pos: tuple, h_func):
    """
    Compute Heuristic for the current maze
    :param maze: Maze
    :param goal_pos: This is the goal state where we want to reach
    :param h_func: Heuristic function we want to use
    :return: None as we are updating in the same maze object
    """

    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            if not maze.maze[row][col].is_blocked:
                maze.maze[row][col].h = h_func((row, col), goal_pos)


def create_maze_array_from_paths(paths: list, maze: Maze, is_field_of_view_explored: bool = True):
    """
    This function will create a numpy maze array from the paths we have visited to generate new maze
    :param paths: This is list of list. Each list contains the path which was found by each A* run
    :param maze: Maze which is explored by agent
    :param is_field_of_view_explored: this function will explore field of view if this attribute is true
    :return: Maze array of shape NUM_ROWS X NUM_COLS which contains the information about the discovered path
    """
    maze_array = np.ones((NUM_ROWS, NUM_COLS))
    for path in paths:
        for node in path:
            if not maze.maze[node[0]][node[1]].is_blocked:
                maze_array[node[0]][node[1]] = 0
            if is_field_of_view_explored:
                for ind in range(len(X)):
                    neighbour = (node[0] + X[ind], node[1] + Y[ind])
                    if check(neighbour, NUM_COLS, NUM_ROWS) and (not maze.maze[neighbour[0]][neighbour[1]].is_blocked):
                        maze_array[neighbour[0]][neighbour[1]] = 0

    return maze_array


def compute_g(maze: Maze, start_pos: tuple):
    """
    Compute g(n) from starting position for the given maze to all positions
    :param maze: Maze
    :param start_pos: This is the starting position of the robot from where it will start applying A* search
    :return: None are we are updating in the same maze object
    """

    # Initialize g(n) for the starting position
    maze.maze[start_pos[0]][start_pos[1]].g = 0

    # Initialize Queue to use BFS to compute g(n) for all positions
    q = Queue()
    q.put(start_pos)

    # Iterating over queue until it gets empty
    while not q.empty():
        current_node = q.get()
        for val in range(len(X)):
            neighbour = (current_node[0] + X[val], current_node[1] + Y[val])

            # Check whether the neighbour is in the grid or not. Also, neighbour's g(n) should be greater than current
            # node's g(n) + 1 otherwise we can conclude that it's been visited. Also, neighbour should not be blocked.
            if (check(neighbour, maze.num_cols, maze.num_rows)) and \
                    (maze.maze[current_node[0]][current_node[1]].g + 1 < maze.maze[neighbour[0]][neighbour[1]].g) \
                    and (not maze.maze[neighbour[0]][neighbour[1]].is_blocked):
                maze.maze[neighbour[0]][neighbour[1]].g = maze.maze[current_node[0]][current_node[1]].g + 1
                q.put(neighbour)


def astar_search(maze: Maze, start_pos: tuple, goal_pos: tuple):
    """
    Function to compute A* search
    :param maze: Maze object
    :param start_pos: starting position of the maze from where we want to start A* search
    :param goal_pos: Goal state (position) where we want to reach
    :return: Returning the path from goal_pos to start_pos if it exists
    """

    # Initialize a set for visited nodes
    visited_nodes = set()

    # Initialize a sorted set to pop least value element from the set
    sorted_set = SortedSet()

    # Initialize a dictionary to store a random value assigned to each node. This dictionary would be helpful to know
    # the value of a node when we want to remove a particular node from the sorted set
    node_to_random_number_mapping = dict()

    # Initialize another dictionary to store parent information
    parents = dict()

    # Initialize g and f for the starting position
    maze.maze[start_pos[0]][start_pos[1]].g = 0
    maze.maze[start_pos[0]][start_pos[1]].f = maze.maze[start_pos[0]][start_pos[1]].h

    # Assigning a random number to start position to the starting position and adding to visited nodes
    node_to_random_number_mapping[start_pos] = random.uniform(0, 1)
    visited_nodes.add(start_pos)

    # Add start position node into the sorted set. We are giving priority to f(n), h(n), and g(n) in the decreasing
    # order. Push random number for random selection if there is conflict between two nodes
    # (If f(n), g(n), and h(n) are same for two nodes)
    sorted_set.add(((maze.maze[start_pos[0]][start_pos[1]].f, maze.maze[start_pos[0]][start_pos[1]].h,
                     maze.maze[start_pos[0]][start_pos[1]].g, node_to_random_number_mapping[start_pos]), start_pos))

    parents[start_pos] = start_pos

    num_explored_nodes = 0

    # Running the loop until we reach our goal state or the sorted set is empty
    while sorted_set.__len__() != 0:
        # Popping first (shortest) element from the sorted set
        current_node = sorted_set.pop(index=0)

        # Increase the number of explored nodes
        num_explored_nodes += 1

        # If we have found the goal position, we can return parents and total explored nodes
        if current_node[1] == goal_pos:
            return parents, num_explored_nodes

        # Otherwise, we need to iterate through each child of the current node
        for val in range(len(X)):
            neighbour = (current_node[1][0] + X[val], current_node[1][1] + Y[val])

            # Neighbour should not go outside our maze and it should not be blocked if we want to visit that particular
            # neighbour
            if check(neighbour, maze.num_cols, maze.num_rows) and (
                    not maze.maze[neighbour[0]][neighbour[1]].is_blocked):

                # If neighbour is being visited first time, we should change its g(n) and f(n) accordingly. Also, we
                # need to assign a random value to it for the time of conflict. In the end, we will add all those things
                # into the sorted set and update its parent
                if neighbour not in visited_nodes:
                    maze.maze[neighbour[0]][neighbour[1]].g = maze.maze[current_node[1][0]][current_node[1][1]].g + 1
                    maze.maze[neighbour[0]][neighbour[1]].f = maze.maze[neighbour[0]][neighbour[1]].g + \
                                                              maze.maze[neighbour[0]][neighbour[1]].h
                    node_to_random_number_mapping[neighbour] = random.uniform(0, 1)
                    visited_nodes.add(neighbour)
                    sorted_set.add(((maze.maze[neighbour[0]][neighbour[1]].f, maze.maze[neighbour[0]][neighbour[1]].h,
                                     maze.maze[neighbour[0]][neighbour[1]].g, node_to_random_number_mapping[neighbour]),
                                    neighbour))
                    parents[neighbour] = current_node[1]

                # If a particular neighbour is already visited, we should compare its f(n) value to its previous f(n)
                # value. If current computed f(n) value is less than the previously computed value, we should remove
                # previously computed value and add new value to the sorted set
                else:
                    neighbour_g = maze.maze[current_node[1][0]][current_node[1][1]].g + 1
                    neighbour_f = maze.maze[neighbour[0]][neighbour[1]].h + neighbour_g
                    if neighbour_f < maze.maze[neighbour[0]][neighbour[1]].f:

                        # The following if condition is needed only when the heuristic is inadmissible otherwise a
                        # neighbour has to be in the sorted set if we are able to find out less value of f(n) for that
                        # particular neighbour
                        if ((maze.maze[neighbour[0]][neighbour[1]].f, maze.maze[neighbour[0]][neighbour[1]].h,
                             maze.maze[neighbour[0]][neighbour[1]].g, node_to_random_number_mapping[neighbour]),
                             neighbour) in sorted_set:
                            sorted_set.remove(
                                ((maze.maze[neighbour[0]][neighbour[1]].f, maze.maze[neighbour[0]][neighbour[1]].h,
                                  maze.maze[neighbour[0]][neighbour[1]].g, node_to_random_number_mapping[neighbour]),
                                 neighbour))
                        maze.maze[neighbour[0]][neighbour[1]].g = neighbour_g
                        maze.maze[neighbour[0]][neighbour[1]].f = neighbour_f
                        node_to_random_number_mapping[neighbour] = random.uniform(0, 1)
                        sorted_set.add(
                            ((maze.maze[neighbour[0]][neighbour[1]].f, maze.maze[neighbour[0]][neighbour[1]].h,
                              maze.maze[neighbour[0]][neighbour[1]].g, node_to_random_number_mapping[neighbour]),
                             neighbour))
                        parents[neighbour] = current_node[1]

    return parents, num_explored_nodes


def bfs_search(maze: Maze, start_pos: tuple, goal_pos: tuple):
    """
    This function applies BFS(Breadth First Search) on the given maze
    :param maze: Maze on which we want to apply BFS
    :param start_pos: Starting position of the maze from where you want to start applying BFS
    :param goal_pos: Goal position where you want to reach
    :return: (parents: dictionary which contains the parent of each child in the path, number of explored nodes through
            this BFS)
    """
    # Initialize a set which would be useful to check whether a particular cell is visited or not
    visited_nodes = set()

    # Initialize a queue which would be useful to apply BFS. We used PriorityQueue because at the time of conflicts of
    # the weights, we can select randomly.
    queue = PriorityQueue()

    # Initialize a dictionary to store a parent of each child
    parents = dict()

    # Initialize g value of the starting cell and add it to visited nodes and queue
    maze.maze[start_pos[0]][start_pos[1]].g = 0
    visited_nodes.add(start_pos)
    queue.put(((maze.maze[start_pos[0]][start_pos[1]].g, random.uniform(0, 1)), start_pos))

    parents[start_pos] = start_pos
    num_explored_nodes = 0

    # We will iterate through the queue until we will get goal node or there is no element in the queue to iterate over
    while not queue.empty():

        # Pop the first element of the queue
        current_node = queue.get()
        # Increase the count of explored cells
        num_explored_nodes += 1

        # If we have received a goal node, then we are done and returning parents and total number of explored nodes
        if current_node[1] == goal_pos:
            return parents, num_explored_nodes

        # Need to add each neighbour of the current_node to queue so we can iterate over
        for val in range(len(X)):
            neighbour = (current_node[1][0] + X[val], current_node[1][1] + Y[val])

            # First, we need to check whether dimension of the neighbour exist or not. If it does, it should not be
            # blocked
            if check(neighbour, maze.num_cols, maze.num_rows) and (
                    not maze.maze[neighbour[0]][neighbour[1]].is_blocked):

                # If the neighbour is already in the visited set, we should not add it to the queue.
                if neighbour not in visited_nodes:

                    # If the neighbour is not visited, we should visit it and update its details accordingly
                    maze.maze[neighbour[0]][neighbour[1]].g = maze.maze[current_node[1][0]][current_node[1][1]].g + 1
                    visited_nodes.add(neighbour)
                    queue.put(((maze.maze[neighbour[0]][neighbour[1]].g, random.uniform(0, 1)), neighbour))
                    parents[neighbour] = current_node[1]

    return parents, num_explored_nodes


def repeated_forward(maze: Maze, maze_array: np.array, start_pos: tuple, goal_pos: tuple,
                     is_field_of_view_explored: bool = True, backtrack_length: int = 0, algorithm: str = 'astar',
                     is_backtrack_strategy_on: bool = False):
    """
    This is the repeated forward function which can be used with any algorithm (astar or bfs). This function will
    repeatedly call corresponding algorithm function until it reaches goal or finds out there is no path till goal.
    :param maze: Maze array of agent
    :param maze_array: Original (Full) Maze array
    :param start_pos: starting position of the maze from where agent want to start
    :param goal_pos: goal state where agent want to reach
    :param is_field_of_view_explored: It will explore field of view if this attribute is true otherwise it won't.
    :param backtrack_length: How many times you want to backtrack for each run.
    :param algorithm: Either A* or BFS
    :param is_backtrack_strategy_on: If you want to run strategy 2, this attribute should be set to true
    :return: This function will return final paths on which agent moved to reach goal or empty list if agent can't find
            path to goal. Second is total number of processed nodes while running the algorithm.
    """

    # defining the following two attributes to find which would be useful to return values
    final_paths = list()
    total_explored_nodes = 0
    num_backtracks = 0

    # Running the while loop until we will get a path from start_pos to goal_pos or we have figured out there is no path
    # from start_pos to goal_pos
    while True:
        # Choose which algorithm you want to use for search
        if algorithm == 'astar':
            parents, num_explored_nodes = astar_search(maze, start_pos, goal_pos)
        elif algorithm == 'bfs':
            parents, num_explored_nodes = bfs_search(maze, start_pos, goal_pos)
        else:
            raise Exception("algorithm should be either astar or bfs")

        # Adding up number of nodes explored (processed) in the last call to algorithm.
        total_explored_nodes += num_explored_nodes

        # If goal_pos doesn't exist in parents which means path is not available so returning empty list.
        if goal_pos not in parents:
            return list(), total_explored_nodes, num_backtracks

        # parents contains parent of each node through path from start_pos to goal_pos. To store path from start_pos to
        # goal_pos, we need to store child of each node starting from start_pos.
        cur_pos = goal_pos
        children = dict()

        children[cur_pos] = cur_pos

        # Storing child of each node so we can iterate from start_pos to goal_pos
        while cur_pos != parents[cur_pos]:
            children[parents[cur_pos]] = cur_pos
            cur_pos = parents[cur_pos]

        # Setting current position to starting position so we can start iterating from start_pos
        cur_pos = start_pos

        current_path = [cur_pos]
        last_cell_which_is_not_in_dead_end = cur_pos

        # Iterating from start_pos to goal_pos if we won't get any blocks in between otherwise we are terminating the
        # iteration.
        while cur_pos != children[cur_pos]:

            if is_backtrack_strategy_on:
                path_exist_from_the_last_point = 0

            # Explore the field of view and update the blocked nodes if there's any in the path.
            if is_field_of_view_explored:
                for ind in range(len(X)):
                    neighbour = (cur_pos[0] + X[ind], cur_pos[1] + Y[ind])
                    if (check(neighbour, NUM_COLS, NUM_ROWS)) and (maze_array[neighbour[0]][neighbour[1]] == 1):
                        maze.maze[neighbour[0]][neighbour[1]].is_blocked = True

                    # Here, we are finding whether the current node is a part of the dead end or not. If there is a path
                    # exists other than its child and parent, then this node should not be part of dead end because
                    # there is another path available which you can explore.
                    if is_backtrack_strategy_on and (check(neighbour, NUM_COLS, NUM_ROWS)) and \
                            (children[cur_pos] != neighbour) and (parents[cur_pos] != neighbour) and \
                            (maze_array[neighbour[0]][neighbour[1]] == 0):
                        path_exist_from_the_last_point += 1

            if is_backtrack_strategy_on:

                # If we can find such a node which we can explore later using current node, then this node should not be
                # part of the dead end path.
                if path_exist_from_the_last_point > 0:
                    last_cell_which_is_not_in_dead_end = cur_pos

            # If we encounter any block in the path, we have to terminate the iteration
            if maze_array[children[cur_pos][0]][children[cur_pos][1]] == 1:
                break
            cur_pos = children[cur_pos]
            current_path.append(cur_pos)

        # If we are able to find the goal state, we should return the path and total explored nodes.
        if cur_pos == goal_pos:
            final_paths.append(current_path)
            return final_paths, total_explored_nodes, num_backtracks
        else:

            # Change the start node to last unblocked node and backtrack if it is set to any positive integer.
            maze.maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
            num_backtrack = min(len(current_path) - 1, backtrack_length)
            cur_pos = current_path[-1]

            while num_backtrack > 0:
                cur_pos = parents[cur_pos]
                current_path.append(cur_pos)
                num_backtrack -= 1

            if is_backtrack_strategy_on:

                # We keep backtracking cell until we reached a cell from where we can explore a new path. Also, we are
                # manually blocking those dead end nodes because they are not useful anymore.
                while cur_pos != last_cell_which_is_not_in_dead_end:
                    num_backtracks += 1
                    maze.maze[cur_pos[0]][cur_pos[1]].is_blocked = True
                    cur_pos = parents[cur_pos]
                    current_path.append(cur_pos)

            final_paths.append(current_path)
            start_pos = cur_pos


def single_plot(x, y, title, xlabel, ylabel, savefig_name, fontsize: int = 10):
    """
    This function is used to plot a single plot
    :param x: X axis list
    :param y: Y axis list
    :param title: title of the plot
    :param xlabel: x-label of the plot
    :param ylabel: y-label of the plot
    :param savefig_name: name of the figure which you want to use save
    :param fontsize: change size of the all font (title, x-label, and y-label)
    :return:
    """
    fig, axs = plt.subplots()
    axs.plot(x, y, marker='.', ms=10.0, c='blue', mfc='red')
    axs.set_title(title, fontsize=fontsize)
    axs.set_xlabel(xlabel, fontsize=fontsize)
    axs.set_ylabel(ylabel, fontsize=fontsize)
    plt.savefig(savefig_name)
    plt.show()


def multiple_plot(x, y, title, xlabel, ylabel, savefig_name, legends, fontsize: int = 10):
    """
    This function is used to add multiple plots on y axis
    :param x: X axis list
    :param y: Y axis list of list
    :param title: title of the plot
    :param xlabel: x-label of the plot
    :param ylabel: y-label of the plot
    :param savefig_name: name of the figure which you want to use save
    :param legends: add legends to this multiple plots
    :param fontsize: change size of the all font (title, x-label, and y-label)
    :return:
    """
    fig, axs = plt.subplots()
    for array in y:
        axs.plot(x, array, marker='.', ms=10.0, mfc='red')
    axs.legend(legends)
    axs.set_title(title, fontsize=fontsize)
    axs.set_xlabel(xlabel, fontsize=fontsize)
    axs.set_ylabel(ylabel, fontsize=fontsize)
    plt.savefig(savefig_name)
    plt.show()
