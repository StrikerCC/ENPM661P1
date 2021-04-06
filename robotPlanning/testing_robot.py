import sys

from robotPlanning.robot import robot, point_robot, rigid_robot
from robotPlanning.planning import bfs, Dijkstra, Astart
from robotPlanning.robot_map import map2DWithObstacle, map2DWithObstacleAndClearance
import numpy as np

from shapely.geometry import Point, Polygon
# import geopandas
# from geopandas import GeoSeries


def test():
    ### testing obstacle
    map_ = map2DWithObstacle()
    map_.add_circular_obstacle((90, 70), 70 / 2)

    map_.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    # map_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
    #                            (210, 270), (230, 270), (230, 280), (200, 280)])
    # map_.add_ellipsoid_obstacle((246, 145), 60 / 2, 120 / 2)

    map_0 = map2DWithObstacleAndClearance()
    map_0.add_circular_obstacle((90, 70), 70 / 2)
    map_0.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    map_.show()
    map_0.show()
    # start = [25, 70]
    # goal = [150, 108]
    # # robot_ = rigid_robot(start, clearance=20)
    # robot_ = point_robot(start)
    # map_.show(robot_)
    #
    # print(map_.invalidArea(robot_))
    #
    # # slover = bfs()
    # slover = Astart()
    # goal_node = slover.search(start, goal, robot_, map_)
    # print(goal_node)


if __name__ == '__main__':
    # test()
    a = np.array((0, 0))
    print(a)
    print(type(a))
    # b = np.array([1, 1])
    #
    # dis = np.linalg.norm(a-b)
    # print(dis)