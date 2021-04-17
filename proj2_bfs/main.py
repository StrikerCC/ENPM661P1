import numpy as np
import math
import random

from utils.robot import point_robot
from utils.space import space2DWithObstacle
from utils.nodes import node
from utils.planning import bfs


def main():
    print("Game start")
    space = space2DWithObstacle(height=300, width=400)
    print('searching space size： ', space.size)

    """ initialize a robot"""
    robot_ = point_robot()

    """testing obstacle"""
    space.add_circular_obstacle((90, 70), 70/2)
    space.add_rectangle_obstacle((48, 108), width=150, height=20, angle=35)
    space.add_polygon_obstacle([(200, 230), (230, 230), (230, 240), (210, 240),
                                 (210, 270), (230, 270), (230, 280), (200, 280)])
    space.add_polygon_obstacle([(328, 63), (328+75*math.cos(math.pi/4), 63+75*math.sin(math.pi/4)),
                                 (328+75*math.cos(math.pi/4), 63+75*math.sin(math.pi/4)+55),
                                 (354, 138),
                                 (328 + 60 * math.cos(math.pi * 3 / 4) + 56 * math.cos(math.pi/4),
                                  63 + 60 * math.sin(math.pi * 3 / 4) + 56 * math.cos(math.pi/4)),
                                 (328 + 60 * math.cos(math.pi * 3 / 4), 63 + 60 * math.sin(math.pi * 3 / 4))])
    space.add_ellipsoid_obstacle((246, 145), 60/2, 120/2)

    """start searching"""
    start, goal = None, None
    planning = bfs(retrieve_goal_node=True)
    while True:
        # break
        choice = input('random? y/n?')
        if choice == 'y':
            start_x, start_y, goal_x, goal_y = random.randint(1, 500), random.randint(1, 300), random.randint(1, 500), random.randint(1, 300)
        else:
            start_x, start_y, goal_x, goal_y = int(input("x coordinate of start")), \
                                               int(input("y coordinate of start")), \
                                               int(input("x coordinate of goal")), \
                                               int(input("y coordinate of goal"))

        if space.invalidArea(robot_.teleport((start_x, start_y))) and space.invalidArea(robot_.teleport((goal_x, goal_y))):
            start, goal = (start_x, start_y), (goal_x, goal_y)
            break
        print("start location or goal location in obstacle, please reenter")
    # start, goal = (25, 25), (75, 25)
    planning.search(start, goal, robot_=robot_, map_=space, filepath='results/optimalPath.txt')

    # planning.retrivePathToTxtFile('Assignment2Sol.txt')
    ### plot the optimal path from start to goal state
    # plt.plot()


if __name__ == '__main__':
    main()