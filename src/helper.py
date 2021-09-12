import numpy as np
from src.Maze import Maze
from queue import Queue
from queue import PriorityQueue


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
    distance = 0
    for ind in range(len(pos1)):
        distance += (pos1[ind] - pos2[ind]) * (pos1[ind] - pos2[ind])
    return np.sqrt(distance)


def chebyshev_distance(pos1: tuple, pos2: tuple):
    distance = 0
    for ind in range(len(pos1)):
        distance = max(distance, abs(pos1[ind] - pos2[ind]))
    return distance


def g_function():
    from constants import FUNCTION_FOR_G
    if FUNCTION_FOR_G == 'manhattan':
        return manhattan_distance
    elif FUNCTION_FOR_G == 'euclidean':
        return euclidean_distance
    elif FUNCTION_FOR_G == 'chebychev':
        return chebyshev_distance
    else:
        raise Exception("Function for G is not exist. Please choose from manhattan, euclidean, or chebychev")


def h_function():
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
    if (0 <= pos[0] < num_rows) and (0 <= pos[1] < num_cols):
        return True
    return False


def compute_heuristics(maze: Maze, start_pos, h_func):
    x = [0, 1, 0, -1]
    y = [1, 0, -1, 0]

    maze.maze[start_pos[0]][start_pos[1]].h = 0

    from queue import Queue
    q = Queue()
    q.put(start_pos)

    while not q.empty():
        current_node = q.get()
        for val in range(len(x)):
            neighbour = (current_node[0] + x[val], current_node[1] + y[val])
            if (check(neighbour, maze.num_cols, maze.num_rows)) and \
                (maze.maze[current_node[0]][current_node[1]].h + 1 < maze.maze[neighbour[0]][neighbour[1]].h) \
                    and (not maze.maze[neighbour[0]][neighbour[1]].is_blocked):

                maze.maze[neighbour[0]][neighbour[1]].h = maze.maze[current_node[0]][current_node[1]].h + 1
                q.put(neighbour)


def compute_g(maze: Maze, start_pos, g_func):
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
    x = [0, 1, 0, -1]
    y = [1, 0, -1, 0]

    compute_g(maze, start_pos, g_function())
    compute_heuristics(maze, goal_pos, h_function())

    for row in range(maze.num_rows):
        for col in range(maze.num_cols):
            maze.maze[row][col] = maze.maze[row][col].g + maze.maze[row][col].h

    visited_nodes = set()
    priority_queue = PriorityQueue()
    priority_queue.put((maze.maze[start_pos[0]][start_pos[1]].f, start_pos))

    parents = map()
    parents[start_pos] = start_pos

    while not priority_queue.empty():
        current_node = priority_queue.get()
        if (maze.maze[current_node[1][0]][current_node[1][1]].f <= current_node[0]) or\
                (current_node[1] in visited_nodes):
            continue
        visited_nodes.add(current_node)
        maze.maze[current_node[1][0]][current_node[1][1]].f = current_node[0]

        for val in range(x):
            neighbour = (current_node[1][0] + x[val], current_node[1][1] + y[val])
            if check(neighbour, maze.num_cols, maze.num_rows) and \
                    (not maze.maze[neighbour[0]][neighbour[1]].is_blocked):
                priority_queue.put(neighbour)