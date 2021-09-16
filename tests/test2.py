"""
This is the test file to solve the problem 4.
"""
import numpy as np
from src.helper import generate_grid_with_probability_p, length_of_path_from_source_to_goal
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT

final_list = []
for ind in np.linspace(0.0, 0.4, 11):
    num_times_will_reach_goal = 0.0
    print('Running for ', ind)
    for run_num in range(50):
        maze_array = generate_grid_with_probability_p(ind)
        if length_of_path_from_source_to_goal(maze_array, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT) <= NUM_COLS + NUM_ROWS:
            num_times_will_reach_goal += 1
    final_list.append((ind, num_times_will_reach_goal))

print(final_list)