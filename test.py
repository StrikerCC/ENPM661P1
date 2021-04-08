import sys
import math

from robotPlanning.robot import robot, point_robot, rigid_robot
from robotPlanning.planning import bfs, Dijkstra, Astart, dis_to_point
from robotPlanning.robot_map import map2DWithObstacle, map2DWithObstacleAndClearance
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


def test():

    start = [50.0, 40.0, math.pi/2]
    goal = [90, 90, 0]
    start_ = np.array(start)

    robot_ = rigid_robot(start, radius=10)


    ### testing obstacle
    map_ = map2DWithObstacleAndClearance(clearance=20)
    map_.add_circular_obstacle((90, 70), 70 / 2)

    # map_.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    # map_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
    #                            (210, 270), (230, 270), (230, 280), (200, 280)])
    # map_.add_ellipsoid_obstacle((246, 145), 60 / 2, 120 / 2)
    # map_.show()

    # map_0 = map2DWithObstacleAndClearance()
    # map_0.add_circular_obstacle((90, 70), 70 / 2)
    # map_0.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    # map_0.show()



    # print(map_.invalidArea(robot_))
    #
    # # slover = bfs()
    slover = Astart()

    goal_node = slover.search(start, goal, robot_, map_)
    # print(goal_node)


if __name__ == '__main__':
    ass = np.arange(0, 10)
    print(ass.shape)
    # ass = ass.reshape((2, -1))
    print(ass.shape == (10,))

    k = (0.5, 2.1)
    print(k)
    # k = tuple(map(eval, k))
    print(k)


    test()
    # dis = np.linalg.norm(a-b)
    # print(dis)