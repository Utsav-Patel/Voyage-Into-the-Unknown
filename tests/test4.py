"""
This file contains the experiment mentioned in the problem 6 and 7.
"""

import numpy as np
import matplotlib.pyplot as plt
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward_astar_search, manhattan_distance, \
    compute_heuristics, length_of_path_from_source_to_goal, create_maze_array_from_maze
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF

final_discovered_maze = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(final_discovered_maze, GOAL_POSITION_OF_AGENT, manhattan_distance)

values_of_probabilities = []
avg_trajectory_lengths = []
avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = []
avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = []
avg_num_cells_processed_by_repeated_astar = []


def avg(lst: list):
    return sum(lst) / len(lst)


for probability_of_having_block in np.linspace(0.00, 0.33, 10):

    values_of_probabilities.append(probability_of_having_block)
    print('Running for ', probability_of_having_block)

    trajectory_lengths = []
    length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = []
    shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = []
    num_cells_processed_by_repeated_astar = []

    run_num = 0
    while run_num < 30:
        maze_array = generate_grid_with_probability_p(probability_of_having_block)
        final_discovered_maze.reset_except_h()
        # print(maze_array)
        distance_from_start_to_goal_on_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)
        if distance_from_start_to_goal_on_full_grid >= INF:
            continue

        run_num += 1
        final_paths, total_explored_nodes = repeated_forward_astar_search(final_discovered_maze, maze_array,
                                                                          STARTING_POSITION_OF_AGENT,
                                                                          GOAL_POSITION_OF_AGENT)

        len_of_paths_travelled_by_repeated_forward_astar = 0

        for path in final_paths:
            len_of_paths_travelled_by_repeated_forward_astar += len(path)
        len_of_paths_travelled_by_repeated_forward_astar -= len(final_paths)

        distance_from_start_to_goal_on_discovered_grid = length_of_path_from_source_to_goal(create_maze_array_from_maze(final_discovered_maze),
                                                                                            STARTING_POSITION_OF_AGENT,
                                                                                            GOAL_POSITION_OF_AGENT)

        trajectory_lengths.append(len_of_paths_travelled_by_repeated_forward_astar)
        length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(len_of_paths_travelled_by_repeated_forward_astar / distance_from_start_to_goal_on_discovered_grid)
        shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(distance_from_start_to_goal_on_discovered_grid / distance_from_start_to_goal_on_full_grid)
        num_cells_processed_by_repeated_astar.append(total_explored_nodes)

    assert run_num == 30

    avg_trajectory_lengths.append(avg(trajectory_lengths))
    avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(avg(length_of_trajectory_by_shortest_path_in_final_discovered_gridworld))
    avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(avg(shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld))
    avg_num_cells_processed_by_repeated_astar.append(avg(num_cells_processed_by_repeated_astar))


fig, axs = plt.subplots(2, 2)
title_font_size=10
axs[0, 0].plot(values_of_probabilities, avg_trajectory_lengths)
axs[0, 0].set_title('Density vs Average Trajectory Length', fontsize=title_font_size)
axs[0, 1].plot(values_of_probabilities, avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld,
               'tab:orange')
axs[0, 1].set_title('Density vs Average (Length of (Trajectory / Shortest Path in Final Discovered Gridworld))',
                    fontsize=title_font_size)
axs[1, 0].plot(values_of_probabilities,
               avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld, 'tab:green')
axs[1, 0].set_title('Density vs Average (Length of Shortest Path in (Final Discovered / Full ) Gridworld',
                    fontsize=title_font_size)
axs[1, 1].plot(values_of_probabilities, avg_num_cells_processed_by_repeated_astar, 'tab:red')
axs[1, 1].set_title('Density vs Average Number of Cells Processed by Repeated A*', fontsize=title_font_size)

# for ax in axs.flat:
#     ax.set(xlabel='Density', ylabel='y-label')
#
# # Hide x labels and tick labels for top plots and y ticks for right plots.
# for ax in axs.flat:
#     ax.label_outer()

plt.show()