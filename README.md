# Final project

## Instructions
Code A*

Download world.csvPreview the document
Write a program to create a 4-connected graph and run an A* search from vertex (0,0) to vertex (19,19) across the obstacle map provided in world.csv.
The world is a 20×20 grid of cells
The world.csv file is an occupancy grid map: 1 means the grid cell is occupied and you can’t move through it
Edge costs are 1
Your code should output the final path (either plot it or print out the vertex coordinates) and associated path cost.
Comment your code to demonstrate that you understand the algorithm.
What to turn in:

A zip file of your commented A* code including world.csv.
A cover sheet (PDF) listing:
Web sites you used
People you worked with
The final path
Your heuristic function (in English)
How you implemented the graph and priority queue
Any known bugs/issues
A few notes:

4-connected means that you can travel from a cell to any of the cardinal neighbors (north, south, east, west).
Broadly speaking, there are two ways you can represent the graph
As an adjacency matrix with a function that returns valid neighbors for a given vertex when queried, or
As a list of vertices and a list of edges.
You need to demonstrate that you understand how the algorithm works and the best way to do this is to comment relevant lines of code. Marks will be awarded accordingly.
There are plenty of resources are available to you online, you may take inspiration from existing implementations that you find, but see Note 3 above.

## Rubric and Grade
| Criteria                                                                                                             | Ratings                  |                      | Pts  |
|----------------------------------------------------------------------------------------------------------------------|--------------------------|----------------------|------|
| Sends waypoints to the robot                                                                                         | 3.0   pts   Full Marks   | 0.0  pts  No Marks   | 3.0  |
| Uses SLAM to create a map                                                                                            | 3.0  pts  Full Marks     | 0.0   pts   No Marks | 3.0  |
| Has a documented exploration strategy                                                                                | 7.0    pts    Full Marks | 0.0   pts   No Marks | 7.0  |
| Strategy works in all world files (entire space visited)                                                             | 7.0    pts    Full Marks | 0.0   pts   No Marks | 7.0  |
| Mechanism for detecting unexplored area                                                                              | 5.0  pts  Full Marks     | 0.0   pts   No Marks | 5.0  |
| Mechanism for detecting when exploration strategy fails  Failure case: didn't get to way point, what do you do next? | 7.0   pts   Full Marks   | 0.0   pts   No Marks | 7.0  |
| Algorithm/strategy for getting to unexplored area                                                                    | 3.0   pts   Full Marks   | 0.0   pts   No Marks | 3.0  |
| Behaves "reasonably" on other test worlds  Doesn't crash, makes some attempt to navigate to unexplored areas         | 5.0  pts  Full Marks     | 0.0   pts   No Marks | 5.0  |
| Total Points:                                                                                                        |                          |                      | 40.0 |

## Setup
* Download the file
* Unzip the file into the \catkin_ws\src directory
* In \catkin_ws, run this command to build

```{bash}
catkin_make
```

## Run
* Open a new terminal
* In \catkin_ws, run this command to source the bash file

```{bash}
source devel/setup.bash
```

* Run this command to launch the ros package

```{bash}
roslaunch src/rob456_project/launch/rob456_project.launch
```
* Open a new terminal
* In \catkin_ws, run this command to source the bash file

```{bash}
source devel/setup.bash
```

* Run this command to launch the ros package

```{bash}
roslaunch src/nav_bundle/launch/nav_bundle.launch
```

* When the world starts, in rviz, select '2D Nav Goal' and point to a closer cell that has been explored