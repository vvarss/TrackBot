# 🤖TrackBot
A multi-robot path planning demo built with Q-Learning in MATLAB. Multiple robots navigate an N × N grid to reach a common goal while avoiding static obstacles. 
Grid size, number of robots and obstacles are user-specified at runtime. The project demonstrates reinforcement learning basics, collision handling, and a simple visualization / animation of robot trajectories.

## 🛣️Project Summary

This project implements a baseline multi-robot planner where each robot learns a local policy using Q-learning on the discrete grid. Robots receive positive reward on reaching the goal and negative reward for hitting obstacles. After training, the learned Q-table is used to generate robot trajectories which are animated on a 2D grid.

## 🛤️Key characteristics

* Language: MATLAB (script)
* Paradigm: Reinforcement learning (Q-Learning) on discrete grid
* Input: Grid size, number of robots, number of obstacles (entered by user)
* Output: Animated simulation, summary metrics (avg path length, computation time, collisions, Q-table memory size)

## 🛤️Features
Random placement of robots, goal, and obstacles.
Q-Learning training loop with tunable hyperparameters:

* gamma (discount factor)

* alpha (learning rate)

* epsilon (exploration probability)

* episodes (training episodes)

* Action set: Right, Left, Up, Down (4-neighbour moves).

* Visualization of grid, obstacles, goal, robot positions and animated trajectories.

* Basic metrics printed at the end: average path length, computation time, number of collisions, model size.

## 🛤️Workflow

* Initialize grid and randomly place goal, obstacles and robots.

* Initialize Q-table: Q_table(rows, cols, 4) (one value per action per cell).

* Train via episodes:

For each robot per episode, choose action by ε-greedy.

Update Q-values using the Q-learning update:
```
Q(s,a) ← (1 − α)Q(s,a) + α (reward + γ max_a' Q(s',a'))
```

* Rewards: large positive for reaching goal, negative for obstacle, small negative per step.

* After training, generate trajectories by greedily following the learned policy (argmax Q).

* Animate robot movement and report metrics.

```
Enter Grid Size as [rows cols]: [10 10]
Enter Number of Robots: 5
Enter Number of Obstacles: 10
Generating grid...
Plotting...
Path Length: 7.00
Computation Time: 0.2573 seconds
Collisions: 0
Max Robots: 5
Model Size: 3200 bytes
```
The following is the visualisation for the given input.
## 🛤️Visualizer
<centre> <img width="482" height="443" alt="image" src="https://github.com/user-attachments/assets/f3041aac-a818-4347-aef5-8f35c7ddd8b4" /> <centre>

The blue lines higlight the trackers for each robot which has started from random positions, taking in the most appropriate path based on Q-Learning, to reach their common goal at (9,2).
The square blocks are the obstacles.

