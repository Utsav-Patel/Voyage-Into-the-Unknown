"""
This file contains the second strategy which we used for the 8th problem.
"""

# Necessary imports
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, manhattan_distance, single_plot, \
    compute_heuristics, length_of_path_from_source_to_goal, create_maze_array_from_paths, avg, \
    compute_trajectory_length_from_path
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF
from datetime import datetime

# Start execution of this file with printing its current time
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Initialize agent's maze and update its heuristic
final_discovered_maze = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(final_discovered_maze, GOAL_POSITION_OF_AGENT, manhattan_distance)

# Initialize necessary lists for this problem
avg_trajectory_lengths = list()
avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = list()
avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = list()
avg_num_cells_processed_by_repeated_astar = list()
avg_computation_time = list()

# Initialize some attributes for this problem
start_value_of_probability = 0.0
end_value_of_probability = 0.4
num_uniform_samples = 11
num_times_run_for_each_probability = 10
problem_no = '8'

# Initialize probabilities
values_of_probabilities = np.linspace(start_value_of_probability, min(0.33, end_value_of_probability),
                                      num_uniform_samples)

# Iterate through each probability
for probability_of_having_block in values_of_probabilities:
    print('Running for ', probability_of_having_block)

    # Initialize the following lists for the problem
    trajectory_lengths = list()
    length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = list()
    shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = list()
    num_cells_processed_by_repeated_astar = list()
    computation_time = list()

    run_num = 0

    # Run the below code multiple times
    while run_num < num_times_run_for_each_probability:

        # Generate random grid with probability of each cell being blocked is `probability_of_having_block`
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
        final_discovered_maze.reset_except_h()
        start_time = datetime.now()
        final_paths, total_explored_nodes = repeated_forward(final_discovered_maze, maze_array,
                                                             STARTING_POSITION_OF_AGENT,
                                                             GOAL_POSITION_OF_AGENT,
                                                             is_backtrack_strategy2_on=True)
        end_time = datetime.now()

        # Compute trajectory length
        len_of_paths_travelled_by_repeated_forward_astar = compute_trajectory_length_from_path(final_paths)

        # Compute distance from source to goal using our discovered maze
        distance_from_start_to_goal_on_discovered_grid = length_of_path_from_source_to_goal(
            create_maze_array_from_paths(final_paths, final_discovered_maze),
            STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

        # Append attributes in the respective lists
        trajectory_lengths.append(len_of_paths_travelled_by_repeated_forward_astar)
        length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(
            len_of_paths_travelled_by_repeated_forward_astar / distance_from_start_to_goal_on_discovered_grid)
        shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(
            distance_from_start_to_goal_on_discovered_grid / distance_from_start_to_goal_on_full_grid)
        num_cells_processed_by_repeated_astar.append(total_explored_nodes)
        computation_time.append((end_time - start_time).total_seconds())

    assert run_num == num_times_run_for_each_probability

    # Append values in the final average lists
    avg_trajectory_lengths.append(avg(trajectory_lengths))
    avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(
        avg(length_of_trajectory_by_shortest_path_in_final_discovered_gridworld))
    avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(
        avg(shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld))
    avg_num_cells_processed_by_repeated_astar.append(avg(num_cells_processed_by_repeated_astar))
    avg_computation_time.append(avg(computation_time))

# Ending execution of the file after adding plots
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

single_plot(values_of_probabilities * 100, avg_trajectory_lengths, 'Density vs Average Trajectory Length',
            'Density (in %)', 'Average Trajectory Length',
            problem_no + '_Average Trajectory Length' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')

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

single_plot(values_of_probabilities * 100, avg_computation_time,
            'Density vs Average computation time',
            'Density (in %)', 'Average Computation time (in seconds)',
            problem_no + '_avg_computation_time_' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')