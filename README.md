# Voyage-Into-the-Unknown
This is repository for the first project of Introduction to Artificial Intelligence course at Rutgers University.

[The details of the project are given here ](https://github.com/Utsav-Patel/Voyage-Into-the-Unknown/blob/main/Assignment_1.pdf)


### Goal -

The main goal of our project is to create 100x100 grids where each cell has some random chance to be blocked. We then place an AI agent at position (0,0) and have to make
the agent reach position (100,100). The agent does not know which cells in the grid are blocked or unblocked. But the agent can 'see' the status (blocked or unblocked) of
its four cardinal neighbors. A sample grid with the agents path will look like this - here the black cells are the blocked cells, S is the starting position of agent, G is
is the goal position of the agent and the grey cells is the path taken by the agent.

<p align="center">
	<img src="/images/First_Path.PNG" width="200" height="200">
</p>

The agent will plan a path to the goal using its current knowledge of the grid and then follow the planned path until it encounters a block. After encountering a block it will
re-plan a path using all the knowledge it has gained while traversing the maze. The agent planned this path by using the A star algorithm.


##### Solvability vs Probability of blocks -

We first tested how many grids were solvable (agent found a valid path from start to goal) versus the probability of each cell being blocked. We got the following results - 

<p align="center">
	<img src="/images/Density_vs_Solvability.png">
</p>

As we can see, most grid are solvable when probability of being blocked lesser than 0.33 and unsolvable when probability is higher than that.


##### Comparing different Heuristics with A* algorithm - 

We wanted to see which heuristic; when used with the A star algorithm gave us the best results. We tested Euclidean, Manhattan and Chebychev heuristics and got the following
results -

<p align="center">
	<img src="/images/combine_computation_time.png">
</p>


As we can see manhattan distance performed extremely well in terms of computation time.

##### Comapring 'blindfolded agent' with normal agent

Here we created another agent that was blindfolded (it can only view blocks that are directly in its path unlike the normal agent that can view blocks in four cardinal directions)
when comparing their trajectory lengths and average number of cells they have to process, we get the following results -  The normal agent is on the left and the blindfolded agent
is on the right.

<div>
<img src="/images/6_Average_Trajectory_Length.png" width="390" height="250" align = "left">

<img src="/images/7_Average_Trajectory_Length.png" width="390" height="250" align = "right">

<img src="/images/6_avg_number_of_cells_processed.png" width="390" height="250" align = "left">

<img src="/images/7_avg_number_of_cells_processed.png" width="390" height="250" align = "right">
</div>


As we can see, the normal agent performed significantly better than the blindfolded agent which was expected. 

##### Using Inadmissible Heuristics to speed up A* -

We tried to heavily speed up our A star algorithm by using inadmissible heuristics, we got the following results -

<p align="center">
	<img src="/images/q9_manhattan_ratio.png">
</p>

As we can see the inadmissible heuristic sped up the computation time by a significant margin.

##### Using BFS instead of A* algorithm -

Here we tested using BFS instead of the normal A star algorithm and got the following results -

<img src="/images/6_avg_number_of_cells_processed.png" width="350" height="250" align = "left">

<img src="/images/extra_credit_6_avg_number_of_cells_processed.png" width="350" height="250" align = "right">


A star and BFS gave us the same results in terms of final path length, but BFS had to explore exponentially more cells than A star and thus takes exponentially more time to run.

# Final Thoughts -

This has been the first project of our Introduction to Artificial Intelligence class. For more in depth details please refer to [our report here](https://github.com/Utsav-Patel/Voyage-Into-the-Unknown/blob/main/Report.pdf)