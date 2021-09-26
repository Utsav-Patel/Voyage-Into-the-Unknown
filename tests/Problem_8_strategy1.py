"""
This file contains the experiment mentioned in the problem 8.
"""

import numpy as np
import matplotlib.pyplot as plt
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, manhattan_distance, \
    compute_heuristics, length_of_path_from_source_to_goal
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF

backtrack_size = 6

final_discovered_maze = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(final_discovered_maze, GOAL_POSITION_OF_AGENT, manhattan_distance)

values_of_probabilities = list()
avg_trajectory_lengths = list()
avg_computation_time = list()
avg_num_cells_processed_by_repeated_astar = list()

for ind in range(backtrack_size):
    avg_trajectory_lengths.append(list())
    avg_computation_time.append(list())
    avg_num_cells_processed_by_repeated_astar.append(list())


def avg(lst: list):
    return sum(lst) / len(lst)


for probability_of_having_block in np.linspace(0.00, 0.33, 11):

    values_of_probabilities.append(probability_of_having_block)
    print('Running for ', probability_of_having_block)

    trajectory_lengths = list()
    num_cells_processed_by_repeated_astar = list()
    computation_time = list()

    for ind in range(backtrack_size):
        trajectory_lengths.append(list())
        num_cells_processed_by_repeated_astar.append(list())
        computation_time.append(list())

    run_num = 0
    while run_num < 30:
        maze_array = generate_grid_with_probability_p(probability_of_having_block)
        # print(maze_array)
        distance_from_start_to_goal_on_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)
        if distance_from_start_to_goal_on_full_grid >= INF:
            continue

        run_num += 1

        for current_backtrack_size in range(backtrack_size):
            final_discovered_maze.reset_except_h()
            final_paths, total_explored_nodes, total_time_in_seconds = \
                repeated_forward(final_discovered_maze, maze_array, STARTING_POSITION_OF_AGENT,
                                 GOAL_POSITION_OF_AGENT, backtrack_length=current_backtrack_size)

            len_of_paths_travelled_by_repeated_forward_astar = 0

            if current_backtrack_size == 5 and probability_of_having_block >= 0.30:
                print(final_paths)
                print(total_explored_nodes)

            for path in final_paths:
                len_of_paths_travelled_by_repeated_forward_astar += len(path)
            len_of_paths_travelled_by_repeated_forward_astar -= len(final_paths)

            trajectory_lengths[current_backtrack_size].append(len_of_paths_travelled_by_repeated_forward_astar)
            computation_time[current_backtrack_size].append(total_time_in_seconds)
            num_cells_processed_by_repeated_astar[current_backtrack_size].append(total_explored_nodes)

    assert run_num == 30

    for current_backtrack_size in range(backtrack_size):
        avg_trajectory_lengths[current_backtrack_size].append(avg(trajectory_lengths[current_backtrack_size]))
        avg_computation_time[current_backtrack_size].append(avg(computation_time[current_backtrack_size]))
        avg_num_cells_processed_by_repeated_astar[current_backtrack_size].append(
            avg(num_cells_processed_by_repeated_astar[current_backtrack_size]))

fig, axs = plt.subplots(2)
title_font_size = 10

for current_backtrack_size in range(backtrack_size):
    axs[0].plot(values_of_probabilities, avg_trajectory_lengths[current_backtrack_size], label=current_backtrack_size)
    axs[0].set_title('Density vs Average Trajectory Length', fontsize=title_font_size)
    axs[1].plot(values_of_probabilities, avg_computation_time[current_backtrack_size], label=current_backtrack_size)
    axs[1].set_title('Density vs Average computational time', fontsize=title_font_size)

axs[0].legend()
axs[1].legend()
# axs[1, 0].plot(values_of_probabilities,
#                avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld, 'tab:green')
# axs[1, 0].set_title('Density vs Average (Length of Shortest Path in (Final Discovered / Full ) Gridworld',
#                     fontsize=title_font_size)
# axs[1, 1].plot(values_of_probabilities, avg_num_cells_processed_by_repeated_astar, 'tab:red')
# axs[1, 1].set_title('Density vs Average Number of Cells Processed by Repeated A*', fontsize=title_font_size)

# for ax in axs.flat:
#     ax.set(xlabel='Density', ylabel='y-label')
#
# # Hide x labels and tick labels for top plots and y ticks for right plots.
# for ax in axs.flat:
#     ax.label_outer()

plt.show()
