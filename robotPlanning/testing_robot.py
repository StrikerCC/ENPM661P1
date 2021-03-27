from robot import robot, point_robot, rigid_robot
from planning import bfs, Dijkstra
from robot_map import map_2D_with_obstacle
import numpy as np


def test():
    ### testing obstacle
    map_ = map_2D_with_obstacle()
    map_.add_circular_obstacle((90, 70), 70 / 2)


    map_.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    # map_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
    #                            (210, 270), (230, 270), (230, 280), (200, 280)])
    # map_.add_ellipsoid_obstacle((246, 145), 60 / 2, 120 / 2)
    map_.show()

    start = [0, 0]
    goal = [150, 108]
    robot_ = point_robot(start)

    robot_.move_up()
    print(robot_)

    # slover = bfs()
    slover = Dijkstra()
    goal_node = slover.search(start, goal, robot_, map_)
    print(goal_node)


test()