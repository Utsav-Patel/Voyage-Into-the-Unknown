import numpy as np
from src.Maze import Maze
from queue import Queue
from queue import PriorityQueue
from constants import NUM_COLS, NUM_ROWS


def generate_grid_manually():
    """
    This is the function to generate  grid manually. This is helpful for the initial testing and problem 1.
    :return: Manually generated numpy array.
    """
    array = np.zeros((5,5))
    array[1][4] = 1
    array[1][3] = 1
    array[2][2] = 1
    array[3][1] = 1
    array[4][3] = 1
    return array


def generate_grid_with_probability_p(p):
    from constants import NUM_COLS, NUM_ROWS
    randomly_generated_array = np.random.uniform(low=0.0, high=1.0, size=NUM_ROWS * NUM_COLS).reshape(NUM_ROWS, NUM_COLS)
    randomly_generated_array[0][0] = 0
    randomly_generated_array[NUM_ROWS-1][NUM_COLS-1] = 0
    randomly_generated_array[randomly_generated_array < (1-p)] = 0
    randomly_generated_array[randomly_generated_array >= (1-p)] = 1
    return randomly_generated_array


def is_path_exist_from_source_to_goal(maze_array: np.array, start_pos: tuple, goal_pos: tuple):
    x = [0, 1, 0, -1]
    y = [1, 0, -1, 0]

    q = Queue()
    q.put(start_pos)
    visited_nodes = set()

    while not q.empty():
        current_node = q.get()
        if current_node == goal_pos:
            return True
        visited_nodes.add(current_node)

        for ind in range(len(x)):
            neighbour = (current_node[0] + x[ind], current_node[1] + y[ind])
            if check(neighbour, NUM_COLS, NUM_ROWS) and (neighbour not in visited_nodes) \
                    and (maze_array[neighbour[0]][neighbour[1]] == 0):
                q.put(neighbour)
                visited_nodes.add(neighbour)

    return False


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


def h_function():
    """
    Find and return appropriate function to compute heuristic distance from the target (goal).
    :return: H function
    """
    from constants import FUNCTION_FOR_H
    if FUNCTION_FOR_H == 'manhattan':
        return manhattan_distance
    elif FUNCTION_FOR_H == 'euclidean':
        return euclidean_distance
    elif FUNCTION_FOR_H == 'chebychev':
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


def compute_g(maze: Maze, start_pos: tuple):
    """
    Compute Shortest distance from starting position for the given maze
    :param maze: Maze
    :param start_pos: This is the starting position of the robot from where it will start applying A* search
    :param g_func: Shortest path function we want to use
    :return: None are we are updating in the same maze object
    """
    x = [0, 1, 0, -1]
    y = [1, 0, -1, 0]

    maze.maze[start_pos[0]][start_pos[1]].g = 0

    q = Queue()
    q.put(start_pos)

    while not q.empty():
        current_node = q.get()
        for val in range(len(x)):
            neighbour = (current_node[0] + x[val], current_node[1] + y[val])
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
    x = [0, 1, 0, -1]
    y = [1, 0, -1, 0]

    compute_g(maze, start_pos)

    for row in range(maze.num_rows):
        for col in range(maze.num_cols):
            maze.maze[row][col].f = maze.maze[row][col].g + maze.maze[row][col].h

    visited_nodes = set()
    priority_queue = PriorityQueue()
    priority_queue.put((maze.maze[start_pos[0]][start_pos[1]].f, start_pos, start_pos))

    parents = dict()

    while not priority_queue.empty():
        current_node = priority_queue.get()
        if current_node[1] in visited_nodes:
            continue

        parents[current_node[1]] = current_node[2]
        visited_nodes.add(current_node[1])

        if current_node[1] == goal_pos:
            return parents

        for val in range(len(x)):
            neighbour = (current_node[1][0] + x[val], current_node[1][1] + y[val])
            if check(neighbour, maze.num_cols, maze.num_rows) and \
                    (not maze.maze[neighbour[0]][neighbour[1]].is_blocked):
                priority_queue.put((maze.maze[neighbour[0]][neighbour[1]].f, neighbour, current_node[1]))

    return parents
