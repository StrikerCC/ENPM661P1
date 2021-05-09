import sys
import math

from utils.robot import robot, point_robot, rigid_robot, turtlebot
from utils.planning import bfs, Dijkstra, Astart, RRT, DRRT
from utils.space import space2DWithObstacle, space2DWithObstacleAndClearance, space2DWithObstacle_dynamic
import numpy as np
import cv2
from shapely.geometry import Point, Polygon
# import geopandas
# from geopandas import GeoSeries

def plot(start, ends):
    space = np.zeros((800, 1000, 3))
    print(space.shape)
    for end in ends:
        start_line, end_line = start[0:-1][::-1].astype(int), end[0:-1][::-1].astype(int)
        cv2.line(space, tuple(start_line), tuple(end_line), color=(0, 150, 0), thickness=3)
        cv2.imshow('', space)
        cv2.waitKey(0)


def test1():

    start = [10.0, 10.0, math.pi/2]
    goal = [220, 290, 0]
    start_ = np.array(start)

    robot_ = rigid_robot(start, radius=5)


    ### testing obstacle
    map_ = space2DWithObstacleAndClearance(clearance=10)

    map_.add_circular_obstacle((50, 50), 20 / 2)

    map_.add_rotated_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    map_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
                               (210, 270), (230, 270), (230, 280), (200, 280)])
    map_.add_ellipsoid_obstacle((246, 145), 60 / 2, 120 / 2)
    map_.show()

    map_0 = space2DWithObstacleAndClearance()
    map_0.add_circular_obstacle((90, 70), 70 / 2)
    map_0.add_rotated_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    map_0.show()

    slover = Astart()

    goal_node = slover.search(start, goal, robot_, map_)
    # print(goal_node)


def test():
    # start = (2, 2)  # Starting node
    # goal = (250, 350)  # Goal node
    # robot_ = point_robot()

    start = [10.0, 10.0, math.pi / 2]
    goal = [220, 290, math.pi / 2]
    robot_ = rigid_robot(start, radius=5)
    #
    # start = (25.0, 25.0, 0.0)
    # goal = (205.0, 205.0, 0.0)
    # robot_ = turtlebot()
    # robot_.teleport(start)

    map_ = space2DWithObstacle_dynamic()
    map_.add_circular_obstacle((200, 200), 20, speed=np.array((0, 10)), period=5)
    map_.add_circular_obstacle((60, 150), 10, speed=np.array((0, 10)), period=5)
    map_.add_circular_obstacle((150, 50), 40, speed=np.array((0, 20)), period=20)
    map_.add_rectangle_obstacle((280, 120), (300, 140), speed=np.array((0, 10)), period=5)
    map_.add_rectangle_obstacle((220, 80), (230, 120), speed=np.array((0, 10)), period=10)
    map_.add_rectangle_obstacle((330, 100), (370, 140), speed=np.array((0, 10)), period=5)

    # map_.add_rotated_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    # map_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
    #                            (210, 270), (230, 270), (230, 280), (200, 280)])
    # map_.add_ellipsoid_obstacle((246, 145), 60 / 2, 120 / 2)
    # map_.show()

    # for i in range(1000):
    #     map_.update_obstacles()
    #     # map_.show()
    #     rrt = RRT()
    #     path = rrt.search(start, goal, robot_, map_)

    for i in range(1000):
        map_.update_obstacles()
        rrt = DRRT()
        path = rrt.planning(start, goal, robot_, map_)


if __name__ == '__main__':
    test()
    # a = np.array((np.random.uniform(0, 10), np.random.uniform(0, 5))).astype(int)
    # print(a)
