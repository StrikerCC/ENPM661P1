import numpy as np
from BFS_point import node_point_robot, graph_point_robot, bfs
# import matplotlib.pyplot as plt
import math
import random


def main():
    print("Game start")
    graph_ = graph_point_robot(height=300, width=400)
    print(graph_.size)

    ### testing obstacle
    graph_.add_circular_obstacle((90, 70), 70/2)
    graph_.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    graph_.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
                                 (210, 270), (230, 270), (230, 280), (200, 280)])
    graph_.add_polygon_obstacle([(328, 63), (328+75*math.cos(math.pi/4), 63+75*math.sin(math.pi/4)),
                                 (328+75*math.cos(math.pi/4), 63+75*math.sin(math.pi/4)+55),
                                 (354, 138),
                                 (328 + 60 * math.cos(math.pi * 3 / 4) + 56 * math.cos(math.pi/4),
                                  63 + 60 * math.sin(math.pi * 3 / 4) + 56 * math.cos(math.pi/4)),
                                 (328 + 60 * math.cos(math.pi * 3 / 4), 63 + 60 * math.sin(math.pi * 3 / 4))])
    graph_.add_ellipsoid_obstacle((246, 145), 60/2, 120/2)
    # graph_.show()

    # testing robotPlanning object
    # robotPlanning = node_point_robot((0, 0))
    # children = robotPlanning.children(robotPlanning, graph_)
    # for child in children:
    #     print(child)
    planning = bfs(graph_)

    # testing planning

    while True:
        choice = input('random? y/n?')
        if choice == 'y':
            start_x, start_y, goal_x, goal_y = random.randint(1, 500), random.randint(1, 300), random.randint(1, 500), random.randint(1, 300)
        else:
            start_x, start_y, goal_x, goal_y = int(input("x coordinate of start")), \
                                               int(input("y coordinate of start")), \
                                               int(input("x coordinate of goal")), \
                                               int(input("y coordinate of goal"))

        if not planning.start_or_goal_in_obstacle((start_x, start_y), (goal_x, goal_y)):
            break
        print("start location or goal location in obstacle, please reenter")
    planning.search((start_x, start_y), (goal_x, goal_y))

    # planning.retrivePathToTxtFile('Assignment2Sol.txt')
    ### plot the optimal path from start to goal state
    # plt.plot()


if __name__ == '__main__':
    main()