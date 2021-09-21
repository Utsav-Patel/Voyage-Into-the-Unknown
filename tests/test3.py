"""
This is the test file to solve the problem 5.
"""
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward_astar_search, \
    manhattan_distance, euclidean_distance, chebyshev_distance, compute_heuristics
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT
import matplotlib.pyplot as plt

mazes = [Maze(NUM_COLS, NUM_ROWS), Maze(NUM_COLS, NUM_ROWS), Maze(NUM_COLS, NUM_ROWS)]
compute_heuristics(mazes[0], GOAL_POSITION_OF_AGENT, manhattan_distance)
compute_heuristics(mazes[1], GOAL_POSITION_OF_AGENT, euclidean_distance)
compute_heuristics(mazes[2], GOAL_POSITION_OF_AGENT, chebyshev_distance)
avg_path_lengths_for_different_heuristics = [[], [], []]
avg_total_explored_nodes_for_different_heuristics = [[], [], []]
avg_computational_time = [[], [], []]
maze_algorithms = ['manhattan', 'euclidean', 'chebychev']

start_value_of_probability = 0.0
end_value_of_probability = 1.0
num_uniform_samples = 101
num_times_run_for_each_probability = 10000


def avg(lst: list):
    if len(lst) == 0:
        return 0
    return sum(lst) / len(lst)


list_of_probability_values = np.linspace(start_value_of_probability, end_value_of_probability, num_uniform_samples)

for probability_of_having_block in list_of_probability_values:

    print('Running for ', probability_of_having_block)

    path_lengths_for_different_heuristics = [list(), list(), list()]
    total_explored_nodes_for_different_heuristics = [list(), list(), list()]
    computational_time = [list(), list(), list()]

    for run_num in range(num_times_run_for_each_probability):
        maze_array = generate_grid_with_probability_p(probability_of_having_block)
        # print(maze_array)
        for maze_index in range(len(mazes)):
            mazes[maze_index].reset_except_h()
            for x in range(NUM_ROWS):
                for y in range(NUM_COLS):
                    if maze_array[x][y] == 1:
                        mazes[maze_index].maze[x][y].is_blocked = True
            final_paths, total_explored_nodes, total_seconds = repeated_forward_astar_search(mazes[maze_index],
                                                                                             maze_array,
                                                                                             STARTING_POSITION_OF_AGENT,
                                                                                             GOAL_POSITION_OF_AGENT)

            if not len(final_paths) == 0:
                length_of_path = 0
                for path in final_paths:
                    length_of_path += len(path)
                length_of_path -= len(final_paths)
                path_lengths_for_different_heuristics[maze_index].append(length_of_path)
                total_explored_nodes_for_different_heuristics[maze_index].append(total_explored_nodes)
                computational_time[maze_index].append(total_seconds)

    for maze_index in range(len(mazes)):
        avg_path_lengths_for_different_heuristics[maze_index].append(avg(path_lengths_for_different_heuristics[maze_index]))
        avg_total_explored_nodes_for_different_heuristics[maze_index].append(avg(total_explored_nodes_for_different_heuristics[maze_index]))
        avg_computational_time[maze_index].append(avg(computational_time[maze_index]))

for maze_index in range(len(mazes)):
    print('Shortest path algorithm for the ', maze_algorithms[maze_index])
    print(avg_path_lengths_for_different_heuristics[maze_index])
    print('Total number of explored states for the ', maze_algorithms[maze_index])
    print(avg_total_explored_nodes_for_different_heuristics[maze_index])
    print('Computation time for the ', maze_algorithms[maze_index])
    print(avg_computational_time[maze_index])
    # for row in range(NUM_ROWS):
    #     for col in range(NUM_COLS):
    #         print(mazes[maze_index].maze[row][col].g, end=" ")
    #     print()


def single_plot(x, y, title, xlabel, ylabel, savefig_name):
    fig, axs = plt.subplots()
    title_font_size = 10
    axs.plot(x, y, marker='.', ms=10.0, c='blue', mfc='red')
    axs.set_title(title, fontsize=title_font_size)
    axs.set_xlabel(xlabel, fontsize=title_font_size)
    axs.set_ylabel(ylabel, fontsize=title_font_size)
    plt.savefig(savefig_name)
    plt.show()


for maze_index in range(len(avg_path_lengths_for_different_heuristics)):
    single_plot(list_of_probability_values, avg_path_lengths_for_different_heuristics[maze_index],
                'Shortest Path for ' + maze_algorithms[maze_index], 'Density', 'Length of Trajectory',
                'Shortest Path for ' + maze_algorithms[maze_index] + '.png')

for maze_index in range(len(avg_total_explored_nodes_for_different_heuristics)):
    single_plot(list_of_probability_values, avg_total_explored_nodes_for_different_heuristics[maze_index],
                'total explored nodes for ' + maze_algorithms[maze_index], 'Density', 'Total Explored Nodes',
                'total explored nodes for ' + maze_algorithms[maze_index] + '.png')

for maze_index in range(len(avg_computational_time)):
    single_plot(list_of_probability_values, avg_computational_time[maze_index],
                'computation time for ' + maze_algorithms[maze_index], 'Density', 'computation time (in seconds)',
                'computation time for ' + maze_algorithms[maze_index] + '.png')


def multiple_plot(x, y, title, xlabel, ylabel, savefig_name):
    fig, axs = plt.subplots()
    title_font_size = 10
    for array in y:
        axs.plot(x, array, marker='.', ms=10.0,  mfc='red')
    axs.legend(maze_algorithms)
    axs.set_title(title, fontsize=title_font_size)
    axs.set_xlabel(xlabel, fontsize=title_font_size)
    axs.set_ylabel(ylabel, fontsize=title_font_size)
    plt.savefig(savefig_name)
    plt.show()

multiple_plot(list_of_probability_values, avg_path_lengths_for_different_heuristics, 'Length of Trajectory', 'Density', 'Length of Trajectory', 'combine_length_of_trajectory.png')
multiple_plot(list_of_probability_values, avg_total_explored_nodes_for_different_heuristics, 'Total Explored Nodes', 'Density', 'total explored nodes', 'combine_total_explored_nodes.png')
multiple_plot(list_of_probability_values, avg_computational_time, 'Total Computation time', 'Density', 'Computation time', 'combine_computation_time.png')