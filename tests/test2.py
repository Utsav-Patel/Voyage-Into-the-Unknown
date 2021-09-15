import numpy as np
from src.helper import generate_grid_with_probability_p, is_path_exist_from_source_to_goal
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT

final_list = []
for ind in np.linspace(0.0, 1.0, 11):
    num_times_will_reach_goal = 0.0
    print('Running for ', ind)
    for run_num in range(10):
        maze_array = generate_grid_with_probability_p(ind)
        print('Completed generating grid')
        if is_path_exist_from_source_to_goal(maze_array, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT):
            num_times_will_reach_goal += 1
        print('Completed iteration')
    final_list.append((ind, num_times_will_reach_goal))

print(final_list)