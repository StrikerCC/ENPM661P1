import sys
import numpy as np
from proj2.BFS_point import node_point_robot, graph_point_robot, bfs
# import matplotlib.pyplot as plt
import math
import random

def main():
    # for path in sys.path:
    #     print(path)


    print("Game start")
    graph_ = graph_point_robot()
    print(graph_.size)

    ### testing obstacle
    graph_.add_circular_obstacle((90, 70), 70/2)
    graph_.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    graph_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
                                 (210, 270), (230, 270), (230, 280), (200, 280)])
    graph_.add_ellipsoid_obstacle((246, 145), 60/2, 120/2)
    # graph_.show()

    # testing robotPlanning object
    # robotPlanning = node_point_robot((0, 0))
    # children = robotPlanning.children(robotPlanning, graph_)
    # for child in children:
    #     print(child)
    planning = bfs(graph_)


if __name__ == '__main__':
    main()