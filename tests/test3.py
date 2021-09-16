"""
This is the test file to solve the problem 5.
"""
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, length_of_path_from_source_to_goal, \
    manhattan_distance, euclidean_distance, chebyshev_distance, compute_heuristics, repeated_forward_astar_search
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT

mazes = [Maze(NUM_COLS, NUM_ROWS), Maze(NUM_COLS, NUM_ROWS), Maze(NUM_COLS, NUM_ROWS)]
compute_heuristics(mazes[0], GOAL_POSITION_OF_AGENT, manhattan_distance)
# for row in range(NUM_ROWS):
#     for col in range(NUM_COLS):
#         print(mazes[0].maze[row][col].h, end=" ")
#     print()
compute_heuristics(mazes[1], GOAL_POSITION_OF_AGENT, euclidean_distance)
compute_heuristics(mazes[2], GOAL_POSITION_OF_AGENT, chebyshev_distance)
maze_path_lengths_for_different_heuristics = [[], [], []]

for probability_of_having_block in np.linspace(0.25, 0.45, 1):

    print('Running for ', probability_of_having_block)

    for run_num in range(1):
        maze_array = generate_grid_with_probability_p(probability_of_having_block)
        print(maze_array)
        for maze_index in range(len(mazes)):
            mazes[maze_index].reset_except_h()
            for x in range(NUM_ROWS):
                for y in range(NUM_COLS):
                    if maze_array[x][y] == 1:
                        mazes[maze_index].maze[x][y].is_blocked = True
            final_paths = repeated_forward_astar_search(mazes[maze_index], maze_array,
                                                        STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

            if not len(final_paths) == 0:
                print(final_paths)
                length_of_path = 0
                for path in final_paths:
                    length_of_path += len(path)
                length_of_path -= len(final_paths)
                maze_path_lengths_for_different_heuristics[maze_index].append(length_of_path)

for maze_index in range(len(maze_path_lengths_for_different_heuristics)):
    print(sum(maze_path_lengths_for_different_heuristics[maze_index]) / len(maze_path_lengths_for_different_heuristics[maze_index]))
    print(maze_path_lengths_for_different_heuristics[maze_index])
    # for row in range(NUM_ROWS):
    #     for col in range(NUM_COLS):
    #         print(mazes[maze_index].maze[row][col].g, end=" ")
    #     print()