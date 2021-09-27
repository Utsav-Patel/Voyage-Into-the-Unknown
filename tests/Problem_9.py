"""
This file contains the experiment of problem 9.
"""

# Necessary imports
import numpy as np
from datetime import datetime
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, repeated_forward, manhattan_distance, avg, multiple_plot, \
    compute_heuristics, euclidean_distance, chebyshev_distance, length_of_path_from_source_to_goal, single_plot, \
    create_maze_array_from_paths, compute_trajectory_length_from_path
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT, INF

# Starting execution of this file printing start time
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Create three mazes with different heuristics
maze_for_manhattan = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze_for_manhattan, GOAL_POSITION_OF_AGENT, manhattan_distance)

maze_for_euclidean = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze_for_euclidean, GOAL_POSITION_OF_AGENT, euclidean_distance)

maze_for_chebyshev = Maze(NUM_COLS, NUM_ROWS)
compute_heuristics(maze_for_chebyshev, GOAL_POSITION_OF_AGENT, chebyshev_distance)

# Create final maze with chosen heuristic and a particular factor
final_maze = Maze(NUM_COLS, NUM_ROWS)
heuristic = 'chebyshev'
factor = 2
filename = '_known_grid_' + heuristic + "_" + str(factor) + "_"

# Change heuristic of the final maze
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

# Initialize some attributes and lists
start_value_of_probability = 0.0
end_value_of_probability = 0.2
num_uniform_samples = 101
num_times_run_for_each_probability = 500
is_field_of_view_explored = True

avg_trajectory_lengths = [list(), list()]
avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = [list(), list()]
avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = [list(), list()]
avg_num_cells_processed_by_repeated_astar = [list(), list()]
avg_computation_time = [list(), list()]
avg_ratio_of_computation_time = list()
avg_ratio_num_cells_processed_by_repeated_astar = list()
values_of_probabilities = np.linspace(start_value_of_probability, min(0.33, end_value_of_probability),
                                      num_uniform_samples)


def compute_required_fields():
    """
    this function is used to compute some fields mentioned in the return statement
    :return: (trajectory length, (trajectory length / shortest distance in final discovered grid),
    (shortest path in final discovered grid / shortest path in full grid), total processed nodes,
    total seconds to execute repeated A*)
    """

    start_time = datetime.now()
    final_paths, total_explored_nodes = repeated_forward(maze, maze_array,
                                                         STARTING_POSITION_OF_AGENT,
                                                         GOAL_POSITION_OF_AGENT)[:2]
    end_time = datetime.now()
    total_seconds = (end_time - start_time).total_seconds()

    # Computing trajectory length
    len_of_paths_travelled_by_repeated_forward_astar = compute_trajectory_length_from_path(final_paths)

    # compute distance from starting position to goal position on the final discovered grid
    distance_from_start_to_goal_on_discovered_grid = length_of_path_from_source_to_goal(
        create_maze_array_from_paths(final_paths, maze,
                                     is_field_of_view_explored=is_field_of_view_explored),
        STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)

    return len_of_paths_travelled_by_repeated_forward_astar, \
           len_of_paths_travelled_by_repeated_forward_astar / distance_from_start_to_goal_on_discovered_grid, \
           distance_from_start_to_goal_on_discovered_grid / distance_from_start_to_goal_on_full_grid, \
           total_explored_nodes, total_seconds


# Iterate over each probability
for probability_of_having_block in values_of_probabilities:
    print('Running for ', probability_of_having_block)

    # Initialize some attributes
    trajectory_lengths = [list(), list()]
    length_of_trajectory_by_shortest_path_in_final_discovered_gridworld = [list(), list()]
    shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld = [list(), list()]
    num_cells_processed_by_repeated_astar = [list(), list()]
    computation_time = [list(), list()]
    ratio_computation_time = list()
    ratio_num_cells_processed_by_repeated_astar = list()

    run_num = 0

    # Run the below code multiple times for all solvable mazes
    while run_num < num_times_run_for_each_probability:

        # Generate maze with probability of cell being blocked is `probability_of_having_block`
        maze_array = generate_grid_with_probability_p(probability_of_having_block)

        # Compute distance from source to goal to check the solvability of the given maze
        distance_from_start_to_goal_on_full_grid = length_of_path_from_source_to_goal(maze_array,
                                                                                      STARTING_POSITION_OF_AGENT,
                                                                                      GOAL_POSITION_OF_AGENT)

        # Checking whether maze is solvable or not
        if distance_from_start_to_goal_on_full_grid >= INF:
            continue

        # Increment the counter for number of times the given function has been run
        run_num += 1

        # Run alternatively for admissible and inadmissible heuristic
        for ind in range(2):
            if ind == 0:
                if heuristic == 'manhattan':
                    maze = maze_for_manhattan
                elif heuristic == 'euclidean':
                    maze = maze_for_euclidean
                else:
                    maze = maze_for_chebyshev
            else:
                maze = final_maze

            # reset agent's maze and then make it known
            maze.reset_except_h()

            for row in range(NUM_ROWS):
                for col in range(NUM_COLS):
                    if maze_array[row][col] == 1:
                        maze.maze[row][col].is_blocked = True
                    else:
                        maze.maze[row][col].is_blocked = False

            # Run the above function and fetch the corresponding attributes
            len_of_paths_travelled_by_repeated_forward_astar, \
            length_of_trajectory_by_shortest_path_in_final_discovered_gridworld_val, \
            shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld_val, \
            num_cells_processed_by_repeated_astar_val, \
            total_seconds = compute_required_fields()

            # Append attributes to its corresponding lists
            trajectory_lengths[ind].append(len_of_paths_travelled_by_repeated_forward_astar)
            length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind].append(
                length_of_trajectory_by_shortest_path_in_final_discovered_gridworld_val)
            shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind].append(
                shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld_val)
            num_cells_processed_by_repeated_astar[ind].append(num_cells_processed_by_repeated_astar_val)
            computation_time[ind].append(total_seconds)

        ratio_computation_time.append(computation_time[0][-1] / computation_time[1][-1])
        ratio_num_cells_processed_by_repeated_astar.append(num_cells_processed_by_repeated_astar[0][-1] /
                                                           num_cells_processed_by_repeated_astar[1][-1])

    for ind in range(2):
        avg_trajectory_lengths[ind].append(avg(trajectory_lengths[ind]))
        avg_length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind].append(
            avg(length_of_trajectory_by_shortest_path_in_final_discovered_gridworld[ind]))
        avg_shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind].append(
            avg(shortest_path_in_final_discovered_gridworld_by_shortest_path_in_full_gridworld[ind]))
        avg_num_cells_processed_by_repeated_astar[ind].append(avg(num_cells_processed_by_repeated_astar[ind]))
        avg_computation_time[ind].append(avg(computation_time[ind]))
    avg_ratio_of_computation_time.append(avg(ratio_computation_time))
    avg_ratio_num_cells_processed_by_repeated_astar.append(avg(ratio_num_cells_processed_by_repeated_astar))

# Ending execution of the file after generating all the plots
legends = ['admissible' + '_' + heuristic, 'inadmissible' + '_' + heuristic]
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

multiple_plot(values_of_probabilities * 100, avg_trajectory_lengths, 'Length of Trajectory', 'Density (in %)',
              'Length of Trajectory',
              'problem_9_' + filename + "trajectory_length_" + str(
                  datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', legends)
multiple_plot(values_of_probabilities * 100, avg_num_cells_processed_by_repeated_astar, 'Total Explored Nodes',
              'Density (in %)', 'total explored nodes',
              'problem_9_' + filename + "total_explored_nodes_" + str(
                  datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', legends)
multiple_plot(values_of_probabilities * 100, avg_computation_time, 'Total Computation time', 'Density (in %)',
              'Computation time (in seconds)',
              'problem_9_' + filename + "computation_time_" + str(
                  datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png', legends)

single_plot(values_of_probabilities * 100, avg_ratio_of_computation_time,
            'Density vs Average ratio of computation time (admissible / inadmissible)',
            'Density (in %)', 'Ratio of computation time',
            'problem_9_' + filename + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
single_plot(values_of_probabilities * 100, avg_ratio_num_cells_processed_by_repeated_astar,
            'Density vs Average ratio of number of cell processed (admissible / inadmissible)',
            'Density (in %)', 'Ratio of number of cells processed',
            'problem_9_' + filename + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
