"""
This is the test file to solve the problem 4.
"""

# Necessary Imports
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, astar_search, single_plot
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT
from datetime import datetime

# Just to print how much time code took to run this file
print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Initialize some attributes
start_value_of_probability = 0.0
end_value_of_probability = 1.0
num_uniform_samples = 101
num_times_run_for_each_probability = 3000
maze = Maze(NUM_COLS, NUM_ROWS)
probability_list = np.linspace(start_value_of_probability, end_value_of_probability, num_uniform_samples)
final_list = list()
final_print_matrix = list()

# Iterate through each probability
for probability_of_each_cell_getting_block in probability_list:
    num_times_will_reach_goal = 0.0
    print('Running for ', probability_of_each_cell_getting_block)

    # Run code multiple times so that we can take average out
    for run_num in range(num_times_run_for_each_probability):

        # Generate initial maze randomly with probability of each cell block is `probability_of_each_cell_getting_block`
        maze_array = generate_grid_with_probability_p(probability_of_each_cell_getting_block)

        # Make Agent's maze known
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                if maze_array[row][col] == 1:
                    maze.maze[row][col].is_blocked = True
                else:
                    maze.maze[row][col].is_blocked = False

        # Check if the goal position is in the final paths or not
        if GOAL_POSITION_OF_AGENT in astar_search(maze, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)[0]:
            num_times_will_reach_goal += 1

    # Append corresponding results
    final_list.append(num_times_will_reach_goal * 100 / num_times_run_for_each_probability)
    final_print_matrix.append((probability_of_each_cell_getting_block, num_times_will_reach_goal))

print(final_print_matrix)
print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

# Final Plot for problem 4
single_plot(probability_list * 100, final_list, title='Density vs Solvability', xlabel='Density (in %)',
            ylabel='Solvability (in %)',
            savefig_name='problem_4_' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
