"""
This is the test file to solve the problem 4.
"""
import numpy as np
from src.Maze import Maze
from src.helper import generate_grid_with_probability_p, astar_search
from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT
import matplotlib.pyplot as plt
from datetime import datetime

print('Start running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

start_value_of_probability = 0.0
end_value_of_probability = 1.0
num_uniform_samples = 101
num_times_run_for_each_probability = 3000
maze = Maze(NUM_COLS, NUM_ROWS)

probability_list = np.linspace(start_value_of_probability, end_value_of_probability, num_uniform_samples)
final_list = list()
final_print_matrix = list()

for probability_of_each_cell_getting_block in probability_list:
    num_times_will_reach_goal = 0.0
    print('Running for ', probability_of_each_cell_getting_block)
    for run_num in range(num_times_run_for_each_probability):
        maze_array = generate_grid_with_probability_p(probability_of_each_cell_getting_block)
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                if maze_array[row][col] == 1:
                    maze.maze[row][col].is_blocked = True
                else:
                    maze.maze[row][col].is_blocked = False
        if GOAL_POSITION_OF_AGENT in astar_search(maze, STARTING_POSITION_OF_AGENT, GOAL_POSITION_OF_AGENT)[0]:
            num_times_will_reach_goal += 1
    final_list.append(num_times_will_reach_goal * 100 / num_times_run_for_each_probability)
    final_print_matrix.append((probability_of_each_cell_getting_block, num_times_will_reach_goal))

print(final_print_matrix)

print('Ending running this file at', datetime.now().strftime("%m-%d-%Y %H-%M-%S"))

fig, axs = plt.subplots()
title_font_size=10
axs.plot(probability_list * 100, final_list, marker='.', ms=10.0, c='blue', mfc='red')
axs.set_title('Density vs Solvability', fontsize=title_font_size)
axs.set_xlabel('Density (in %)', fontsize=title_font_size)
axs.set_ylabel('Solvability (in %)', fontsize=title_font_size)
plt.savefig('problem_4_' + str(datetime.now().strftime("%m-%d-%Y %H-%M-%S")) + '.png')
plt.show()
