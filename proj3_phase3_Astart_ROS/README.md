# ENPM661P3
A* for turtlebot\
python 3.6
# library 
To import required library, first import numpy to store the digits in a 2d-array.  
then from robotPlanning.robot_map import map2DWithObstacle, map2DWithObstacleAndClearance.  
then robotPlanning.planning import bfs, Dijkstra, Astart.
then robotPlanning.robot import robot, point_robot, rigid_robot
`
from robotPlanning.robot_map import map2DWithObstacleAndClearance
from robotPlanning.planning import Astart
from robotPlanning.robot import turtlebot
`  

# Instructions to run the code
First run main.py in project3_phase3_part1
type in the clearance of turtlebot
chose random start and goal state
or input 
``
`x coordinate of start = `   ``
`y coordinate of start = `   ``
`x coordinate of goal = `  ``
`y coordinate of goal = `  ``

``


Next, create A* object, and start search  
`
planning = Astart(retrieve_goal_node=True)`   ``

`
planning.search(start, goal, robot_, map_, filepath=r'results/output.txt')
`   ``

`planning.search()`  ``

Finally, retrieve path to txtFile  
`planning.retrivePathToTxtFile(file)`  