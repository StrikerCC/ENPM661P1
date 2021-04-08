import sys
import math
import random
import numpy as np

from robotPlanning.robot_map import map2DWithObstacle, map2DWithObstacleAndClearance
from robotPlanning.planning import bfs, Dijkstra, Astart
from robotPlanning.robot import robot, point_robot, rigid_robot


flag_ui = True


def main():
    print("Game start")

    """initialize map"""
    map_ = map2DWithObstacle()
    print('map size is', map_.size)

    """ask user type of robt"""
    radius_robot = 10
    robot_ = rigid_robot(state=None, radius=radius_robot)
    while True and flag_ui:
        choice = input('type of robot? 1: point_robot, 2: rigid_robot with clearance?')
        if choice == '1':
            break
        elif choice == '2':
            clearance = -1
            while clearance < 1 or clearance > 25:
                clearance = int(input('please enter clearance of the rigid robot, between 1 and 25'))
            map_ = map2DWithObstacleAndClearance(clearance=clearance+radius_robot)
            break
        else:
            print('invalid input, please re-enter')

    """add obstacle into map"""
    map_.add_circular_obstacle((90, 70), 70 / 2)
    map_.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    map_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
                               (210, 270), (230, 270), (230, 280), (200, 280)])
    map_.add_ellipsoid_obstacle((246, 145), 60 / 2, 120 / 2)
    # map_.show()

    """ask user coordinates of start and goal"""
    start_x, start_y, goal_x, goal_y = 0, 0, 0, 0
    while True and flag_ui:
        choice = input('random? y/n?')
        if choice.lower() == 'y':
            start_x, start_y, goal_x, goal_y = random.randint(1, 500), random.randint(1, 300), random.randint(1, 500), random.randint(1, 300)
            # break
        elif choice.lower() == 'n':
            start_x, start_y, goal_x, goal_y = int(input("x coordinate of start")), \
                                               int(input("y coordinate of start")), \
                                               int(input("x coordinate of goal")), \
                                               int(input("y coordinate of goal"))
            # break
        else:
            print('invalid input, please re-enter')
        if map_.invalidArea(robot_.teleport([start_x, start_y])) or map_.invalidArea(robot_.teleport([goal_x, goal_y])):
            print("start location and goal location is ok")
            break
        print("start location or goal location", start_x, start_y, goal_x, goal_y, type(start_x), " in obstacle, please re-enter")

    """test a simply case"""
    # start = (25, 35)
    # goal = (65, 35)

    """show map"""
    # map_.show()
    if map_.invalidArea(robot_.teleport([start_x, start_y])) or map_.invalidArea(robot_.teleport([goal_x, goal_y])):
        print("start location and goal location is ok")

    """planing"""
    robot_.teleport(start)
    planning = Dijkstra(retrieve_goal_node=True)
    planning.search(start, goal, robot_, map_, filepath=r'resuts/output.txt')


if __name__ == '__main__':
    main()
