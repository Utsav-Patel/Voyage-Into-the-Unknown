"""
This file contains the experiment of problem 9.
"""

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, manhattan_distance, \
    compute_heuristics, euclidean_distance, chebyshev_distance, length_of_path_from_source_to_goal, \
    create_maze_array_from_paths
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF

print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

maze_for_manhattan = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze_for_manhattan, GOAL_POSITION_OF_AGENT, manhattan_distance)

maze_for_euclidean = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze_for_euclidean, GOAL_POSITION_OF_AGENT, euclidean_distance)

maze_for_chebyshev = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze_for_chebyshev, GOAL_POSITION_OF_AGENT, chebyshev_distance)

final_maze = Maze(NUM_COLS, NUM_ROWS)
heuristic = 'manhattan'
factor = 2
filename = heuristic + "_" + str(factor) + "_"

for row in range(NUM_ROWS):
    for col in range(NUM_COLS):
        if heuristic == 'manhattan':
            final_maze.maze[row][col].h = maze_for_manhattan.maze[row][col].h * factor
        elif heuristic == 'euclidean':
            final_maze.maze[row][col].h = maze_for_euclidean.maze[row][col].h * factor
        elif heuristic == 'chebyshev':
            final_maze.maze[row][col].h = maze_for_chebyshev.maze[row][col].h * factor
        else:
            raise Exception("We are supporting only manhattan, euclidean and chebyshev as of now")

start_value_of_probability = 0.0
end_value_of_probability = 0.2
num_uniform_samples = 101
num_times_run_for_each_probability = 100
is_field_of_view_explored = True

avg_trajectory_lengths = [list(), list()]
avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = [list(), list()]
avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = [list(), list()]
avg_num_cells_processed_by_repeated_astar = [list(), list()]
avg_computation_time = [list(), list()]
avg_ratio_of_computation_time = list()

values_of_probabilities = np.linspace(start_value_of_probability, min(0.33, end_value_of_probability),
                                      num_uniform_samples)


def avg(lst: list):
    return sum(lst) / len(lst)


def compute_required_fields():
    maze.reset_except_h()

    start_time = datetime.now()
    final_paths, total_explored_nodes = repeated_forward(maze, maze_array,
                                                         STARTING_POSITION_OF_AGENT,
                                                         GOAL_POSITION_OF_AGENT)
    end_time = datetime.now()
    total_seconds = (end_time - start_time).total_seconds()

    len_of_paths_travelled_by_repeated_forward_astar = 0

    for path in final_paths:
        len_of_paths_travelled_by_repeated_forward_astar += len(path)
    len_of_paths_travelled_by_repeated_forward_astar -= len(final_paths)

    if not is_field_of_view_explored:
        len_of_paths_travelled_by_repeated_forward_astar += 2 * (len(final_paths) - 1)

    distance_from_start_to_goal_on_discovered_grid = length_of_path_from_source_to_goal(
        create_maze_array_from_paths(final_paths, maze,
                                     is_field_of_view_explored=is_field_of_view_explored),
        STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

    return len_of_paths_travelled_by_repeated_forward_astar, \
           len_of_paths_travelled_by_repeated_forward_astar / distance_from_start_to_goal_on_discovered_grid, \
           distance_from_start_to_goal_on_discovered_grid / distance_from_start_to_goal_on_full_grid, \
           total_explored_nodes, total_seconds


for probability_of_having_block in values_of_probabilities:
    print('Running for ', probability_of_having_block)

    trajectory_lengths = [list(), list()]
    length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = [list(), list()]
    shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = [list(), list()]
    num_cells_processed_by_repeated_astar = [list(), list()]
    computation_time = [list(), list()]
    ratio_computation_time = list()

    run_num = 0
    while run_num < num_times_run_for_each_probability:
        maze_array = generate_grid_with_probability_p(probability_of_having_block)
        distance_from_start_to_goal_on_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)
        if distance_from_start_to_goal_on_full_grid >= INF:
            continue

        run_num += 1

        for ind in range(2):
            if ind == 0:
                if heuristic == 'manhattan':
                    maze = maze_for_manhattan
                elif heuristic  == 'euclidean':
                    maze = maze_for_euclidean
                else:
                    maze = maze_for_chebyshev
            else:
                maze = final_maze
            # for row in range(NUM_ROWS):
            #     for col in range(NUM_COLS):
            #         if maze_array[row][col] == 1:
            #             maze.maze[row][col].is_blocked = True
            #         else:
            #             maze.maze[row][col].is_blocked = False

            len_of_paths_travelled_by_repeated_forward_astar, \
            length_of_trajectory_by_shortest_path_in_final_discovered_gridworld_val, \
            shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld_val, \
            num_cells_processed_by_repeated_astar_val, \
            total_seconds = compute_required_fields()

            trajectory_lengths[ind].append(len_of_paths_travelled_by_repeated_forward_astar)
            length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind].append(
                length_of_trajectory_by_shortest_path_in_final_discovered_gridworld_val)
            shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind].append(
                shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld_val)
            num_cells_processed_by_repeated_astar[ind].append(num_cells_processed_by_repeated_astar_val)
            computation_time[ind].append(total_seconds)

        ratio_computation_time.append(computation_time[0][-1] / computation_time[1][-1])

    for ind in range(2):
        avg_trajectory_lengths[ind].append(avg(trajectory_lengths[ind]))
        avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind].append(avg(length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind]))
        avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind].append(avg(shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind]))
        avg_num_cells_processed_by_repeated_astar[ind].append(avg(num_cells_processed_by_repeated_astar[ind]))
        avg_computation_time[ind].append(avg(computation_time[ind]))
    avg_ratio_of_computation_time.append(avg(ratio_computation_time))


legends = ['admissible' + '_' + heuristic, 'inadmissible' + '_' + heuristic]
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))


def multiple_plot(x, y, title, xlabel, ylabel, savefig_name):
    fig, axs = plt.subplots()
    title_font_size = 10
    for array in y:
        axs.plot(x, array, marker='.', ms=10.0, mfc='red')
    axs.legend(legends)
    axs.set_title(title, fontsize=title_font_size)
    axs.set_xlabel(xlabel, fontsize=title_font_size)
    axs.set_ylabel(ylabel, fontsize=title_font_size)
    plt.savefig(savefig_name)
    plt.show()


multiple_plot(values_of_probabilities * 100, avg_trajectory_lengths, 'Length of Trajectory', 'Density (in %)',
              'Length of Trajectory',
              'problem_9_' + filename + "trajectory_length_" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
multiple_plot(values_of_probabilities * 100, avg_num_cells_processed_by_repeated_astar, 'Total Explored Nodes',
              'Density (in %)', 'total explored nodes',
              'problem_9_' + filename + "total_explored_nodes_" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
multiple_plot(values_of_probabilities * 100, avg_computation_time, 'Total Computation time', 'Density (in %)',
              'Computation time (in seconds)',
              'problem_9_' + filename + "computation_time_" + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')


def single_plot(x, y, title, xlabel, ylabel, savefig_name):
    fig, axs = plt.subplots()
    title_font_size = 10
    axs.plot(x, y, marker='.', ms=10.0, c='blue', mfc='red')
    axs.set_title(title, fontsize=title_font_size)
    axs.set_xlabel(xlabel, fontsize=title_font_size)
    axs.set_ylabel(ylabel, fontsize=title_font_size)
    plt.savefig(savefig_name)
    plt.show()


single_plot(values_of_probabilities * 100, avg_ratio_of_computation_time, 'Density vs Average ratio of computation time (admissible / inadmissible)',
            'Density (in %)', 'Ratio of computation time', filename + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')