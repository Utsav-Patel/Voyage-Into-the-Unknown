from src.helper import generate_grid_manually, generate_grid_with_probability_p, astar_search, compute_heuristics, \
    h_function
from constants import GENERATE_GRID_MANUALLY, NUM_COLS, NUM_ROWS, INF
from src.Maze import Maze

maze_array = None
if GENERATE_GRID_MANUALLY:
    maze_array = generate_grid_manually()
else:
    maze_array = generate_grid_with_probability_p()

maze = Maze(NUM_COLS, NUM_ROWS)

print(maze_array)

compute_heuristics(maze, (NUM_ROWS - 1, NUM_COLS - 1), h_function())

final_paths = list()
start_pos = (0, 0)

while True:
    parents = astar_search(maze, start_pos, (len(maze_array) - 1, len(maze_array[0]) - 1))

    if (len(maze_array) - 1, len(maze_array[0]) - 1) not in parents:
        print("Path doesn't exist to the goal")
        break

    path_string = ""
    cur_pos = (len(maze_array) - 1, len(maze_array[0]) - 1)
    children = dict()

    children[cur_pos] = cur_pos

    while cur_pos != parents[cur_pos]:
        path_string = str(cur_pos) + path_string
        children[parents[cur_pos]] = cur_pos
        cur_pos = parents[cur_pos]

    cur_pos = start_pos

    current_path = [cur_pos]
    while cur_pos != children[cur_pos]:
        if maze_array[children[cur_pos][0]][children[cur_pos][1]] == 1:
            break
        cur_pos = children[cur_pos]
        current_path.append(cur_pos)
    final_paths.append(current_path)

    if cur_pos == (NUM_ROWS - 1, NUM_COLS - 1):
        print('Hurray! Path is found using Forward Path A* Algorithm')
        print(final_paths)
        break
    else:
        maze.maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
        for ind in range(NUM_ROWS):
            for ind2 in range(NUM_COLS):
                maze.maze[ind][ind2].g = INF
        start_pos = cur_pos
