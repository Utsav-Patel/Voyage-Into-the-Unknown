"""
This file uses the general implementation of the A* Algorithm and runs the code for the 1st problem (Implementation).
"""

from src.helper import generate_grid_manually, astar_search, compute_heuristics, h_function, check
from constants import NUM_COLS, NUM_ROWS, INF, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT
from src.Maze import Maze

maze_array = generate_grid_manually()
print(maze_array)

maze = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze, GOAL_POSITION_OF_AGENT, h_function())
final_paths = list()
start_pos = STARTING_POSITION_OF_AGENT

x = [0, 1, 0, -1]
y = [1, 0, -1, 0]

while True:
    parents = astar_search(maze, start_pos, GOAL_POSITION_OF_AGENT)

    if GOAL_POSITION_OF_AGENT not in parents:
        print("Path doesn't exist to the goal")
        break

    cur_pos = GOAL_POSITION_OF_AGENT
    children = dict()

    children[cur_pos] = cur_pos

    while cur_pos != parents[cur_pos]:
        children[parents[cur_pos]] = cur_pos
        cur_pos = parents[cur_pos]

    cur_pos = start_pos

    current_path = [cur_pos]
    while cur_pos != children[cur_pos]:

        # Explore the field of view and update the blocked nodes
        for ind in range(len(x)):
            neighbour = (cur_pos[0] + x[ind], cur_pos[1] + y[ind])
            if (check(neighbour, NUM_COLS, NUM_ROWS)) and (maze_array[neighbour[0]][neighbour[1]] == 1):
                maze.maze[neighbour[0]][neighbour[1]].is_blocked = True

        if maze_array[children[cur_pos][0]][children[cur_pos][1]] == 1:
            break
        cur_pos = children[cur_pos]
        current_path.append(cur_pos)
    final_paths.append(current_path)

    if cur_pos == GOAL_POSITION_OF_AGENT:
        print('Hurray! Path is found using Forward Path A* Algorithm')
        print(final_paths)
        break
    else:
        maze.maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
        for ind in range(NUM_ROWS):
            for ind2 in range(NUM_COLS):
                maze.maze[ind][ind2].g = INF
        start_pos = cur_pos
