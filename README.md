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
	<img src="/images/Density_vs_Solvability.png" width="400" height="200">
</p>