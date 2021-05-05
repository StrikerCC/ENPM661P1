import sys
import math

from utils.robot import robot, point_robot, rigid_robot
from utils.planning import bfs, Dijkstra, Astart, dis_to_point
from utils.space import space2DWithObstacle, space2DWithObstacleAndClearance
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

    start = [10.0, 10.0, math.pi/2]
    goal = [220, 290, 0]
    start_ = np.array(start)

    robot_ = rigid_robot(start, radius=5)


    ### testing obstacle
    map_ = space2DWithObstacleAndClearance(clearance=10)

    map_.add_circular_obstacle((50, 50), 20 / 2)

    map_.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    map_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
                               (210, 270), (230, 270), (230, 280), (200, 280)])
    map_.add_ellipsoid_obstacle((246, 145), 60 / 2, 120 / 2)
    map_.show()

    map_0 = space2DWithObstacleAndClearance()
    map_0.add_circular_obstacle((90, 70), 70 / 2)
    map_0.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    map_0.show()

    slover = Astart()

    goal_node = slover.search(start, goal, robot_, map_)
    # print(goal_node)


if __name__ == '__main__':
    a_s = [i for i in range(10)]
    b_s = [i for i in range(10)]

    # for i, a, b in enumerate(a_s, b_s):
    #     print(i, a, b)
    for i, (a, b) in enumerate(zip(a_s, b_s)):
        print(i, a, b)

    # test()
    # dis = np.linalg.norm(a-b)
    # print(dis)