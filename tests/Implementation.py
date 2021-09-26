"""
This file uses the general implementation of the Repeated Forward A* Algorithm and runs the code for the
1st problem (Implementation). Also, we checked our latest update on any helper function using this file.
"""

from src.helper import generate_grid_manually, repeated_forward, compute_heuristics, h_function
from constants import NUM_COLS, NUM_ROWS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT
from src.Maze import Maze

maze_array = generate_grid_manually()
print(maze_array)

maze = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze, GOAL_POSITION_OF_AGENT, h_function("manhattan"))

final_paths = repeated_forward(maze, maze_array, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT,
                               is_backtrack_strategy2_on=True)[0]
# for row in range(NUM_ROWS):
#     for col in range(NUM_COLS):
#         print(maze.maze[row][col].f, end=" ")
#     print()

if len(final_paths) == 0:
    print("Path doesn't exist to the goal")
else:
    print('Hurray! Path is found using Repeated Forward A* Algorithm')
    print(final_paths)