"""
This file contains the experiment mentioned in the problem 6, 7 and extra credit question with same experiment using
repeated BFS.
"""

import numpy as np
import matplotlib.pyplot as plt
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, manhattan_distance, \
    compute_heuristics, length_of_path_from_source_to_goal, create_maze_array_from_paths
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF
from datetime import datetime

print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

final_discovered_maze = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(final_discovered_maze, GOAL_POSITION_OF_AGENT, manhattan_distance)

avg_trajectory_lengths = list()
avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = list()
avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = list()
avg_num_cells_processed_by_repeated_astar = list()

start_value_of_probability = 0.0
end_value_of_probability = 0.2
num_uniform_samples = 101
num_times_run_for_each_probability = 100
is_field_of_view_explored = False
is_extra_credit_question = True
algorithm = 'astar'

if is_field_of_view_explored:
    problem_no = '6'
else:
    problem_no = '7'

if is_extra_credit_question:
    problem_no = 'extra_credit_' + problem_no
    algorithm = 'bfs'

values_of_probabilities = np.linspace(start_value_of_probability, min(0.33, end_value_of_probability),
                                      num_uniform_samples)


def avg(lst: list):
    return sum(lst) / len(lst)


for probability_of_having_block in values_of_probabilities:
    print('Running for ', probability_of_having_block)

    trajectory_lengths = list()
    length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = list()
    shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = list()
    num_cells_processed_by_repeated_astar = list()

    run_num = 0
    while run_num < num_times_run_for_each_probability:
        maze_array = generate_grid_with_probability_p(probability_of_having_block)
        final_discovered_maze.reset_except_h()
        # print(maze_array)
        distance_from_start_to_goal_on_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)
        if distance_from_start_to_goal_on_full_grid >= INF:
            continue

        run_num += 1
        final_paths, total_explored_nodes = repeated_forward(final_discovered_maze, maze_array,
                                                             STARTING_POSITION_OF_AGENT,
                                                             GOAL_POSITION_OF_AGENT,
                                                             is_field_of_view_explored=is_field_of_view_explored,
                                                             algorithm=algorithm)

        len_of_paths_travelled_by_repeated_forward_astar = 0

        for path in final_paths:
            len_of_paths_travelled_by_repeated_forward_astar += len(path)
        len_of_paths_travelled_by_repeated_forward_astar -= len(final_paths)

        # if not is_field_of_view_explored:
        #     len_of_paths_travelled_by_repeated_forward_astar += 2 * (len(final_paths) - 1)

        distance_from_start_to_goal_on_discovered_grid = length_of_path_from_source_to_goal(
            create_maze_array_from_paths(final_paths, final_discovered_maze,
                                         is_field_of_view_explored=is_field_of_view_explored),
            STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

        trajectory_lengths.append(len_of_paths_travelled_by_repeated_forward_astar)
        length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(
            len_of_paths_travelled_by_repeated_forward_astar / distance_from_start_to_goal_on_discovered_grid)
        shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(
            distance_from_start_to_goal_on_discovered_grid / distance_from_start_to_goal_on_full_grid)
        num_cells_processed_by_repeated_astar.append(total_explored_nodes)

    assert run_num == num_times_run_for_each_probability

    avg_trajectory_lengths.append(avg(trajectory_lengths))
    avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(
        avg(length_of_trajectory_by_shortest_path_in_final_discovered_gridworld))
    avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(
        avg(shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld))
    avg_num_cells_processed_by_repeated_astar.append(avg(num_cells_processed_by_repeated_astar))

print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))


def single_plot(x, y, title, xlabel, ylabel, savefig_name):
    fig, axs = plt.subplots()
    title_font_size = 10
    axs.plot(x, y, marker='.', ms=10.0, c='blue', mfc='red')
    axs.set_title(title, fontsize=title_font_size)
    axs.set_xlabel(xlabel, fontsize=title_font_size)
    axs.set_ylabel(ylabel, fontsize=title_font_size)
    plt.savefig(savefig_name)
    plt.show()


single_plot(values_of_probabilities * 100, avg_trajectory_lengths, 'Density vs Average Trajectory Length',
            'Density (in %)', 'Average Trajectory Length', problem_no + '_Average Trajectory Length' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')

single_plot(values_of_probabilities * 100, avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld,
            'Density vs Average (Length of (Trajectory / Shortest Path in Final Discovered Gridworld))',
            'Density (in %)', 'Ratio of two lengths',
            problem_no + '_trajectory_by_shortest_path_in_final_discovered_grid' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')

single_plot(values_of_probabilities * 100,
            avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld,
            'Density vs Average (Length of Shortest Path in (Final Discovered / Full ) Gridworld',
            'Density (in %)', 'Ratio of two lengths',
            problem_no + '_shortest_path_in_final_discovered_by_full_gridworld' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')

single_plot(values_of_probabilities * 100, avg_num_cells_processed_by_repeated_astar,
            'Density vs Average Number of Cells Processed by Repeated A*',
            'Density (in %)', 'Average Number of Cells Processed',
            problem_no + '_avg_number_of_cells_processed' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
