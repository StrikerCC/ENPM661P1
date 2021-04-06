# ENPM661P3 phase 1
Dijkstra for rigid robot

# library 
To import required library, first import numpy to store the digits in a 2d-array.  
import opencv-python

`from robotPlanning.robot_map import map2DWithObstacle, map2DWithObstacleAndClearance
from robotPlanning.planning import bfs, Dijkstra, Astart
from robotPlanning.robot import robot, point_robot, rigid_robot
`  

# Instructions to run the code
First create initial state and goal state as a length 2 tuple.  
then build search tree class with those states.  
Input ``
`x coordinate of start = `   ``
`y coordinate of start = `   ``
`x coordinate of goal = `  ``
`y coordinate of goal = `  ``

``


Next, create Dijkstra object, and start search  
`searcher = Dijkstra()`   ``

`searcher.search(start, goal, robot_, map_, filepat=)`  ``

Finally, retrieve path to txtFile  
`searcher.retrivePathToTxtFile(file)`  