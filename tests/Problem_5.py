"""
This is the test file to solve the problem 5.
"""

# Necessary Imports
from datetime import datetime
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, single_plot, avg, compute_heuristics, \
    manhattan_distance, euclidean_distance, chebyshev_distance, multiple_plot, compute_trajectory_length_from_path
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT

# Just to check how much time the code took
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Initialize attributes for this problem and compute heuristics
mazes = [Maze(NUM_COLS, NUM_ROWS), Maze(NUM_COLS, NUM_ROWS), Maze(NUM_COLS, NUM_ROWS)]
compute_heuristics(mazes[0], GOAL_POSITION_OF_AGENT, manhattan_distance)
compute_heuristics(mazes[1], GOAL_POSITION_OF_AGENT, euclidean_distance)
compute_heuristics(mazes[2], GOAL_POSITION_OF_AGENT, chebyshev_distance)

avg_path_lengths_for_different_heuristics = [list(), list(), list()]
avg_total_explored_nodes_for_different_heuristics = [list(), list(), list()]
avg_computational_time = [list(), list(), list()]
maze_algorithms = ['manhattan', 'euclidean', 'chebyshev']

start_value_of_probability = 0.0
end_value_of_probability = 1.0
num_uniform_samples = 101
num_times_run_for_each_probability = 3000

# List of probability values
list_of_probability_values = np.linspace(start_value_of_probability, end_value_of_probability, num_uniform_samples)

# Iterate through each probability
for probability_of_having_block in list_of_probability_values:

    # Just printing so we know where we are at execution
    print('Running for ', probability_of_having_block)

    # Initialize some attributes for each probability
    path_lengths_for_different_heuristics = [list(), list(), list()]
    total_explored_nodes_for_different_heuristics = [list(), list(), list()]
    computational_time = [list(), list(), list()]

    # Run the same code multiple times
    for run_num in range(num_times_run_for_each_probability):

        # Generate maze randomly with each cell is blocked with probability of `probability_of_having_block`
        maze_array = generate_grid_with_probability_p(probability_of_having_block)

        # Iterate through three mazes with different heuristic function
        for maze_index in range(len(mazes)):

            # Reset maze
            mazes[maze_index].reset_except_h()

            # Make agent's maze known
            for row in range(NUM_ROWS):
                for col in range(NUM_COLS):
                    if maze_array[row][col] == 1:
                        mazes[maze_index].maze[row][col].is_blocked = True
                    else:
                        mazes[maze_index].maze[row][col].is_blocked = False

            # Compute how much time does repeated A* take
            starting_time = datetime.now()

            # Call to repeated forward A*
            final_paths, total_explored_nodes = repeated_forward(mazes[maze_index], maze_array,
                                                                 STARTING_POSITION_OF_AGENT,
                                                                 GOAL_POSITION_OF_AGENT)[:2]

            end_time = datetime.now()

            # Compute total seconds to run the repeated forward A*
            total_seconds = (end_time - starting_time).total_seconds()

            # Compute total path length and append some attributes in their corresponding lists
            if not len(final_paths) == 0:
                length_of_path = compute_trajectory_length_from_path(final_paths)
                path_lengths_for_different_heuristics[maze_index].append(length_of_path)
                total_explored_nodes_for_different_heuristics[maze_index].append(total_explored_nodes)
                computational_time[maze_index].append(total_seconds)

    # Compute average for each one
    for maze_index in range(len(mazes)):
        avg_path_lengths_for_different_heuristics[maze_index].append(
            avg(path_lengths_for_different_heuristics[maze_index]))
        avg_total_explored_nodes_for_different_heuristics[maze_index].append(
            avg(total_explored_nodes_for_different_heuristics[maze_index]))
        avg_computational_time[maze_index].append(avg(computational_time[maze_index]))

# Print average of shortest path, total explored nodes, and computation time.
for maze_index in range(len(mazes)):
    print('Shortest path algorithm for the ', maze_algorithms[maze_index])
    print(avg_path_lengths_for_different_heuristics[maze_index])
    print('Total number of explored states for the ', maze_algorithms[maze_index])
    print(avg_total_explored_nodes_for_different_heuristics[maze_index])
    print('Computation time for the ', maze_algorithms[maze_index])
    print(avg_computational_time[maze_index])

# Ending execution for this file. Now only plots are remaininng
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Plot average shortest path for manhattan, euclidean and chebyshev distances
for maze_index in range(len(avg_path_lengths_for_different_heuristics)):
    single_plot(list_of_probability_values, avg_path_lengths_for_different_heuristics[maze_index],
                'Shortest Path for ' + maze_algorithms[maze_index], 'Density', 'Length of Trajectory',
                'Shortest Path for ' + maze_algorithms[maze_index] + str(
                    datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')

# Plot average total number of explored nodes for manhattan, euclidean and chebyshev
for maze_index in range(len(avg_total_explored_nodes_for_different_heuristics)):
    single_plot(list_of_probability_values, avg_total_explored_nodes_for_different_heuristics[maze_index],
                'total explored nodes for ' + maze_algorithms[maze_index], 'Density', 'Total Explored Nodes',
                'total explored nodes for ' + maze_algorithms[maze_index] + str(
                    datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')

# Plot average computation time for manhattan, euclidean and chebyshev
for maze_index in range(len(avg_computational_time)):
    single_plot(list_of_probability_values, avg_computational_time[maze_index],
                'computation time for ' + maze_algorithms[maze_index], 'Density', 'computation time (in seconds)',
                'computation time for ' + maze_algorithms[maze_index] + str(
                    datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')

# Plot average of shortest path for manhattan, euclidean, and chebyshev in a single plot
multiple_plot(list_of_probability_values, avg_path_lengths_for_different_heuristics, 'Length of Trajectory', 'Density',
              'Length of Trajectory',
              'combine_length_of_trajectory' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', maze_algorithms)

# Plot average of total explored nodes for manhattan, euclidean, and chebyshev in a single plot
multiple_plot(list_of_probability_values, avg_total_explored_nodes_for_different_heuristics, 'Total Explored Nodes',
              'Density', 'total explored nodes',
              'combine_total_explored_nodes' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', maze_algorithms)

# Plot average of computation time for manhattan, euclidean, and chebyshev in a single plot
multiple_plot(list_of_probability_values, avg_computational_time, 'Total Computation time', 'Density',
              'Computation time',
              'combine_computation_time' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', maze_algorithms)
