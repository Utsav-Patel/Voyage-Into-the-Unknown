"""
This file contains the strategy which we used for the 8th problem.
"""

# Necessary imports
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, manhattan_distance, single_plot, \
    compute_heuristics, length_of_path_from_source_to_goal, create_maze_array_from_paths, avg, multiple_plot, \
    compute_trajectory_length_from_path, generate_grid_manually
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF
from datetime import datetime

# Start execution of this file with printing its current time
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Initialize agent's maze and update its heuristic
final_discovered_maze = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(final_discovered_maze, GOAL_POSITION_OF_AGENT, manhattan_distance)

# Initialize necessary lists for this problem
avg_trajectory_lengths = [list(), list()]
avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = [list(), list()]
avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = [list(), list()]
avg_num_cells_processed_by_repeated_astar = [list(), list()]
avg_computation_time = [list(), list()]
avg_num_backtracks = list()
avg_ratio_trajectory_lengths = list()
avg_ratio_computation_time = list()
avg_ratio_num_cells_processed_by_repeated_astar = list()

# Initialize some attributes for this problem
start_value_of_probability = 0.0
end_value_of_probability = 0.4
num_uniform_samples = 101
num_times_run_for_each_probability = 300
problem_no = '8'

# Initialize probabilities
values_of_probabilities = np.linspace(start_value_of_probability, end_value_of_probability, num_uniform_samples)

# Iterate through each probability
for probability_of_having_block in values_of_probabilities:
    print('Running for ', probability_of_having_block)

    # Initialize the following lists for the problem
    trajectory_lengths = [list(), list()]
    length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = [list(), list()]
    shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = [list(), list()]
    num_cells_processed_by_repeated_astar = [list(), list()]
    computation_time = [list(), list()]
    num_backtracks = list()
    ratio_trajectory_lengths = list()
    ratio_computation_time = list()
    ratio_num_cells_processed_by_repeated_astar = list()

    run_num = 0

    # Run the below code multiple times
    while run_num < num_times_run_for_each_probability:

        # Generate random grid with probability of each cell being blocked is `probability_of_having_block`
        # maze_array = generate_grid_manually()
        maze_array = generate_grid_with_probability_p(probability_of_having_block)

        # Compute distance from starting position to goal position to check whether the maze is solvable or not
        distance_from_start_to_goal_on_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)

        # If the distance is greater than or equal to INF, then the given maze is not solvable
        if distance_from_start_to_goal_on_full_grid >= INF:
            continue

        # Now, Following code will run if the maze is solvable. First, we need to reset previously used maze and need to
        # call repeated forward
        run_num += 1
        for ind in range(2):
            if ind == 0:
                is_backtrack_strategy_on = False
            else:
                is_backtrack_strategy_on = True
            final_discovered_maze.reset_except_h()
            start_time = datetime.now()
            final_paths, total_explored_nodes, backtracks = repeated_forward(final_discovered_maze, maze_array,
                                                                             STARTING_POSITION_OF_AGENT,
                                                                             GOAL_POSITION_OF_AGENT,
                                                                             is_backtrack_strategy_on=
                                                                             is_backtrack_strategy_on)
            end_time = datetime.now()

            # Compute trajectory length
            len_of_paths_travelled_by_repeated_forward_astar = compute_trajectory_length_from_path(final_paths)

            # Compute distance from source to goal using our discovered maze
            distance_from_start_to_goal_on_discovered_grid = length_of_path_from_source_to_goal(
                create_maze_array_from_paths(final_paths, final_discovered_maze),
                STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

            # Append attributes in the respective lists
            trajectory_lengths[ind].append(len_of_paths_travelled_by_repeated_forward_astar)
            length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind].append(
                len_of_paths_travelled_by_repeated_forward_astar / distance_from_start_to_goal_on_discovered_grid)
            shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind].append(
                distance_from_start_to_goal_on_discovered_grid / distance_from_start_to_goal_on_full_grid)
            num_cells_processed_by_repeated_astar[ind].append(total_explored_nodes)
            computation_time[ind].append((end_time - start_time).total_seconds())

            if is_backtrack_strategy_on:
                num_backtracks.append(backtracks)

        ratio_trajectory_lengths.append(trajectory_lengths[0][-1] / trajectory_lengths[1][-1])
        ratio_num_cells_processed_by_repeated_astar.append(num_cells_processed_by_repeated_astar[0][-1] / \
                                                           num_cells_processed_by_repeated_astar[1][-1])
        ratio_computation_time.append(computation_time[0][-1] / computation_time[1][-1])

    assert run_num == num_times_run_for_each_probability

    # Append values in the final average lists
    for ind in range(2):
        avg_trajectory_lengths[ind].append(avg(trajectory_lengths[ind]))
        avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind].append(
            avg(length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind]))
        avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind].append(
            avg(shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind]))
        avg_num_cells_processed_by_repeated_astar[ind].append(avg(num_cells_processed_by_repeated_astar[ind]))
        avg_computation_time[ind].append(avg(computation_time[ind]))
    avg_num_backtracks.append(avg(num_backtracks))
    avg_ratio_trajectory_lengths.append(avg(ratio_trajectory_lengths))
    avg_ratio_num_cells_processed_by_repeated_astar.append(avg(ratio_num_cells_processed_by_repeated_astar))
    avg_ratio_computation_time.append(avg(ratio_computation_time))

# Ending execution of the file after generating all the plots
legends = ['without backtracking', 'with backtracking']
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))
filename = 'problem_8_'
multiple_plot(values_of_probabilities * 100, avg_trajectory_lengths, 'Length of Trajectory', 'Density (in %)',
              'Length of Trajectory',
              filename + "trajectory_length_" + str(
                  datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', legends)
multiple_plot(values_of_probabilities * 100, avg_num_cells_processed_by_repeated_astar, 'Total Explored Nodes',
              'Density (in %)', 'total explored nodes',
              filename + "total_explored_nodes_" + str(
                  datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', legends)
multiple_plot(values_of_probabilities * 100, avg_computation_time, 'Total Computation time', 'Density (in %)',
              'Computation time (in seconds)',
              filename + "computation_time_" + str(
                  datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', legends)

single_plot(values_of_probabilities * 100, avg_ratio_computation_time,
            'Density vs Average ratio of computation time (without / with) backtracking',
            'Density (in %)', 'Ratio of computation time',
            filename + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
single_plot(values_of_probabilities * 100, avg_ratio_num_cells_processed_by_repeated_astar,
            'Density vs Average ratio of number of cell processed (without / with) backtracking',
            'Density (in %)', 'Ratio of number of cells processed',
            filename + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
single_plot(values_of_probabilities * 100, avg_ratio_trajectory_lengths,
            'Density vs Average ratio of trajectory lengths (without / with) backtracking',
            'Density (in %)', 'Ratio of trajectory lengths',
            filename + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
single_plot(values_of_probabilities * 100, avg_num_backtracks,
            'Density vs Average number of backtracks',
            'Density (in %)', 'Number of backtracks',
            filename + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
