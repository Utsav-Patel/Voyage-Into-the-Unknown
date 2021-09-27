"""
This file contains the experiment mentioned in the problem 6, 7 and extra credit question with same experiment using
repeated BFS.
"""

# Necessary imports
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, manhattan_distance, avg, \
    compute_heuristics, length_of_path_from_source_to_goal, create_maze_array_from_paths, single_plot, \
    compute_trajectory_length_from_path
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF
from datetime import datetime

# Just printing this to know when the program execution is started
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Initialize a maze for agent
final_discovered_maze = Maze(NUM_COLS, NUM_ROWS)

# Compute heuristic using manhattan
compute_heuristics(final_discovered_maze, GOAL_POSITION_OF_AGENT, manhattan_distance)

# Initialize necessary attributes
avg_trajectory_lengths = list()
avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = list()
avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = list()
avg_num_cells_processed_by_repeated_astar = list()

# You only need to play with the following variables. Rest will be taken care.
start_value_of_probability = 0.0
end_value_of_probability = 0.2

# How many uniform samples you want to take between start value and end value
num_uniform_samples = 101
num_times_run_for_each_probability = 100
is_field_of_view_explored = False
is_extra_credit_question = True
algorithm = 'astar'

# Change problem number according to exploration of field of view
if is_field_of_view_explored:
    problem_no = '6'
else:
    problem_no = '7'

# Change problem number and algorithm according to whether it is extra credit question or not
if is_extra_credit_question:
    problem_no = 'extra_credit_' + problem_no
    algorithm = 'bfs'

# Initialize the values of probabilities
values_of_probabilities = np.linspace(start_value_of_probability, min(0.33, end_value_of_probability),
                                      num_uniform_samples)

# Iterate through each probability
for probability_of_having_block in values_of_probabilities:
    print('Running for ', probability_of_having_block)

    # Initialize the following attributes
    trajectory_lengths = list()
    length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = list()
    shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = list()
    num_cells_processed_by_repeated_astar = list()

    run_num = 0

    # Run the following code for only solvable mazes
    while run_num < num_times_run_for_each_probability:

        # Generate random maze with each cell is blocked with probability of `probability_of_having_block`
        maze_array = generate_grid_with_probability_p(probability_of_having_block)

        # To check whether the maze is solvable or not, we used BFS from starting position to goal position
        distance_from_start_to_goal_on_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)

        # If the distance is greater than or equal to Infinity, then this maze is not solvable
        if distance_from_start_to_goal_on_full_grid >= INF:
            continue

        # Run the following code for the solvable mazes. First, Increment the count and reset the previously used maze.
        run_num += 1
        final_discovered_maze.reset_except_h()

        # Call repeated A* or BFS according to the problem.
        final_paths, total_explored_nodes = repeated_forward(final_discovered_maze, maze_array,
                                                             STARTING_POSITION_OF_AGENT,
                                                             GOAL_POSITION_OF_AGENT,
                                                             is_field_of_view_explored=is_field_of_view_explored,
                                                             algorithm=algorithm)[:2]

        # Compute trajectory length
        len_of_paths_travelled_by_repeated_forward_astar = compute_trajectory_length_from_path(final_paths)

        # Compute the shortest path in the final discovered maze
        distance_from_start_to_goal_on_discovered_grid = length_of_path_from_source_to_goal(
            create_maze_array_from_paths(final_paths, final_discovered_maze,
                                         is_field_of_view_explored=is_field_of_view_explored),
            STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

        # Append necessary things in its corresponding lists
        trajectory_lengths.append(len_of_paths_travelled_by_repeated_forward_astar)
        length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(
            len_of_paths_travelled_by_repeated_forward_astar / distance_from_start_to_goal_on_discovered_grid)
        shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(
            distance_from_start_to_goal_on_discovered_grid / distance_from_start_to_goal_on_full_grid)
        num_cells_processed_by_repeated_astar.append(total_explored_nodes)

    # Check whether the above loop ran perfectly or not
    assert run_num == num_times_run_for_each_probability

    # Append values in the average lists
    avg_trajectory_lengths.append(avg(trajectory_lengths))
    avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld.append(
        avg(length_of_trajectory_by_shortest_path_in_final_discovered_gridworld))
    avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld.append(
        avg(shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld))
    avg_num_cells_processed_by_repeated_astar.append(avg(num_cells_processed_by_repeated_astar))

# Ending executing this file after adding necessary plots
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

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
