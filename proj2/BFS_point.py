import numpy as np
import math
import queue
# from collections import deque
from shapely.geometry import Point, Polygon
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from descartes import PolygonPatch
# import pygame
import time
import cv2

debug = False
show = True
save = True
retrieve_goal_node = True


class geometry():
    def __init__(self, shape, size):
        self.shape_name = shape
        if shape == 'polygon':
            self.size = Polygon(size)
        else:
            self.size = size

    # check if the input point in any obstacle
    def inside(self, point):
        if self.shape_name == 'polygon':
            p_geometry = Point(point[1], point[0])
            return p_geometry.within(self.size)
        elif self.shape_name == 'circular':
            return np.linalg.norm(self.size["center"] - point) <= self.size["radius"]
        elif self.shape_name == 'ellipsoid':
            return ((point[0]-self.size["center"][0])**2)/(self.size['semi_major_axis']**2) + \
                   ((point[1]-self.size["center"][1])**2)/(self.size["semi_minor_axis"]**2) <= 1
        # elif self.shape == 'rectangle':
        # return self.size['dl'][0] <= point[0] <= self.size['ur'][0] and \
        #        self.size['dl'][1] <= point[1] <= self.size['ur'][1]

    def __str__(self):
        return self.shape_name + ' ' + str(self.size)


class graph_point_robot():
    # size = tuple()
    def __init__(self, height=300, width=500):
        self.size = (height, width)
        self.map_obstacle = np.zeros(self.size, dtype=np.bool)

    def get_map(self):
        return self.map_obstacle

    def add_rectangle_obstacle(self, corner_ll, width, height, angle):
        # self.obstacles.append(
        #     geometry('rectangle', {"corner_ll": np.array(corner_ll), "width": width, "height": height, "angle": angle}))
        angle *= math.pi/180
        arctan_h_w = math.atan(height/width)
        diag = math.sqrt(width**2+height**2)
        corner_lr = (corner_ll[0] + width * math.cos(angle), corner_ll[1] + width * math.sin(angle))
        corner_ul = (
        corner_ll[0] + height * math.cos(angle + math.pi / 2), corner_ll[1] + height * math.sin(angle + math.pi / 2))
        corner_ur = (
        corner_ll[0] + diag * math.cos(arctan_h_w + angle), corner_ll[1] + diag * math.sin(arctan_h_w + angle))
        rectangle = Polygon([corner_ll, corner_lr, corner_ur, corner_ul])

        obstacle = geometry('polygon', rectangle)
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if obstacle.inside((i, j)):
                    self.map_obstacle[i, j] = True


    def add_polygon_obstacle(self, points):
        obstacle = geometry('polygon', points)
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if obstacle.inside((i, j)):
                    self.map_obstacle[i, j] = True

    def add_circular_obstacle(self, center, radius):
        obstacle = geometry('circular', {"center": np.array(center)[::-1], "radius": radius})
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if obstacle.inside((i, j)):
                    self.map_obstacle[i, j] = True

    def add_ellipsoid_obstacle(self, center, semi_major_axis, semi_minor_axis):
        obstacle = geometry('ellipsoid', {"center": np.array(center)[::-1], "semi_major_axis": semi_minor_axis,
                                                     "semi_minor_axis": semi_major_axis})
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if obstacle.inside((i, j)):
                    self.map_obstacle[i, j] = True

    def __getitem__(self, indexes):
        assert len(indexes) == 2
        return self.map_obstacle[indexes]

    def freeToGo(self, point):
        return not self.map_obstacle[tuple(point)]

    def show(self):
        img = np.ones(self.size) * 200
        img[self.map_obstacle] = 0
        # draw each obstacle on display window
        img = cv2.flip(img, 0)
        cv2.imshow('obstacle', img)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            return


class graph_last():
    size = tuple()
    def __init__(self, width=500, height=300):
        self.size = (width, height)
        self.obstacles = []

    def add_rectangle_obstacle(self, corner_ll, width, height, angle):
        arctan_h_w = math.atan(height/width)
        diag = math.sqrt(width**2+height**2)
        corner_lr = (corner_ll[0] + width*math.cos(angle), corner_ll[1] + width*math.sin(angle))
        corner_ul = (corner_ll[0] + height*math.cos(angle+math.pi/2), corner_ll[1] + height*math.sin(angle+math.pi/2))
        corner_ur = (corner_ll[0] + diag*math.cos(arctan_h_w+angle), corner_ll[1] + diag*math.sin(arctan_h_w+angle))
        rectangle = Polygon([corner_ll, corner_lr, corner_ur, corner_ul])
        # if debug:
        #     print('rectangle corners', rectangle.exterior)
        #     x, y = rectangle.exterior.xy
        #     plt.plot(x, y)
        #     plt.show()

        self.obstacles.append(
            geometry('polygon', rectangle))

    def add_polygon_obstacle(self, points):
        self.obstacles.append(geometry('polygon', points))

    def add_circular_obstacle(self, center, radius):
        self.obstacles.append(geometry('circular', {"center": np.array(center), "radius": radius}))

    def add_ellipsoid_obstacle(self, center, semi_major_axis, semi_minor_axis):
        self.obstacles.append(geometry('ellipsoid', {"center": np.array(center), "semi_major_axis": semi_major_axis,
                                                     "semi_minor_axis": semi_minor_axis}))
    def freeToGo(self, point):
        if not self.obstacles: return True
        for obstacle in self.obstacles:
            if obstacle.inside(point):
                if debug: print(point, ' is in obstacle ', obstacle)
                return False
        return True

    def graphshow(self):
        fig = plt.figure(0)
        ax = fig.add_subplot(111, aspect='equal')

        # fig, ax = plt.subplots()
        # draw each obstacle on display window
        for obstacle in self.obstacles:
            if obstacle.shape_name == 'polygon':
                x, y = obstacle.exterior.xy
                ax = fig.add_subplot(111)
                ax.plot(x, y)
            elif obstacle.shape_name == 'circular':
                circle = plt.Circle(obstacle.size['center'], obstacle.size['radius'])
                ax.add_patch(circle)
            elif obstacle.shape_name == 'ellipsoid':
                ellipse = Ellipse(obstacle.size['center'], width=obstacle.size['semi_major_axis'], height=obstacle.size['semi_minor_axis'], angle=0)
                ax.add_artist(ellipse)

        fig.show()


class node_point_robot:
    def __init__(self, state, start=None, parent=None):
        self.heuristic = (state[0] - start.state[0])**2 + (state[1] - start.state[1])**2 if start else math.inf
        self.state = np.copy(state)
        self.parent = parent if retrieve_goal_node else None


    # return a list of child
    def children(self, node_start, graph_):
        nodes = []
        for move in self.moves():
            node_next = node_point_robot(self.state, start=node_start, parent=self)
            if move(node_next, start=node_start, graph_=graph_):
                nodes.append(node_next)
        if nodes:
            assert type(nodes[0]) == node_point_robot
        return nodes

    def moves(self):
        return [node_point_robot.moveUpRight,
                node_point_robot.moveLowRight,
                node_point_robot.moveUpLeft,
                node_point_robot.moveLowLeft,
                node_point_robot.moveUp, node_point_robot.moveRight,
                node_point_robot.moveDown, node_point_robot.moveLeft]
    def moveLeft(self, start, graph_):
        self.state[1] -= 1
        self.heuristic = (self.state[0] - start.state[0])**2 + (self.state[1] - start.state[1])**2
        return self.state[1] > 0 and graph_.freeToGo(self.state)
    def moveRight(self, start, graph_):
        self.state[1] += 1
        self.heuristic = (self.state[0] - start.state[0])**2 + (self.state[1] - start.state[1])**2
        return self.state[1] < graph_.size[1] and graph_.freeToGo(self.state)
    def moveUp(self, start, graph_):
        self.state[0] += 1
        self.heuristic = (self.state[0] - start.state[0])**2 + (self.state[1] - start.state[1])**2
        if debug:
            print("move up, state: ", self.state, " heuristic: ", self.heuristic, " ")
        return self.state[0] < graph_.size[0] and graph_.freeToGo(self.state)
    def moveDown(self, start, graph_):
        self.state[0] -= 1
        self.heuristic = (self.state[0] - start.state[0])**2 + (self.state[1] - start.state[1])**2
        return self.state[0] > 0 and graph_.freeToGo(self.state)
    def moveLowLeft(self, start, graph_):
        return self.moveLeft(start, graph_) and self.moveDown(start, graph_)
    def moveLowRight(self, start, graph_):
        return self.moveRight(start, graph_) and self.moveDown(start, graph_)
    def moveUpRight(self, start, graph_):
        return self.moveUp(start, graph_) and self.moveRight(start, graph_)
    def moveUpLeft(self, start, graph_):
        return self.moveUp(start, graph_) and self.moveLeft(start, graph_)

    def reach(self, other):
        return np.array_equal(self.state, other.state)

    # to string
    def __str__(self):
        return str(self.state)[1:-1]

    def __lt__(self, other):
        return self.heuristic < other.heuristic

    def __eq__(self, other):
        return self.heuristic == other.heuristic

    # to int for hashing
    # def __hash__(self):
    #     return hash(tuple(map(tuple, self.state)))


# searching algorithm object
class bfs:
    def __init__(self, graph):
        self.graph = graph  # graph of robot field

        print('searching ready')

    def search(self, state_init, state_goal):
        # change coordinate sys
        state_init, state_goal = state_init[::-1], state_goal[::-1]
        # make sure initial and goal state is not in obstacle
        if self.graph[state_init] or self.graph[state_goal]:
            print("start location or goal location in obstacle")
            return

        robot_start = node_point_robot(state_init)  # start node
        robot_goal = node_point_robot(state_goal, start=robot_start)  # goal node
        visited = np.zeros(self.graph.size, dtype=np.uint8)  # hash map of visited node
        visited[self.graph.get_map()] = 75
        queue_node = queue.PriorityQueue()
        queue_node.put(robot_start)  # min priority queue of node, priority is heuristics
        sol = None

        # as long as stack is not empty, keep dfs
        i = 0
        while queue_node:
            i += 1
        # for i in range(1000 * 20):
            if not queue_node:
                print("run out of nodes")
                break
            # while self.queue_node:
            if i % 500 == 0:
                print('loop', i)

            node_cur = queue_node.get()  # take a node from stack
            children = node_cur.children(robot_start, self.graph)  # children expanded from this node

            if debug:
                print(i, " step, reach ", node_cur, ', heuristic ', node_cur.heuristic, ", parent: ", node_cur.parent, ", children: ", children)
            if show:
                cv2.imshow('highlight explored', cv2.flip(visited, 0))
                cv2.waitKey(1)
            for child in children:  # for each child
                if child.reach(robot_goal):  # reach a goal
                    print('find a solution')
                    if retrieve_goal_node: # show optimal path
                        self.retrievePath(child, img=visited, filename="results/optimalPath.txt")
                    return child
                else:  # this child is not a goal
                    if debug: print('find new node')
                    if visited[tuple(child.state)] == 0:  # this child represent a new state never seen before
                        visited[tuple(child.state)] = 155
                        queue_node.put(child)
                    else:  # this child is repetitive
                        del child
                    # if show:
                    #     cv2.imshow('highlight explored', cv2.flip(visited, 0))
                    #     cv2.waitKey(1)
        return None

    # return parents of this node as a list
    def retrievePath(self, node, img, filename=None):
        file = open(filename, 'w')
        if not node:
            file.write('no solution with this map\n')
        else:
            path, node_cur = [], node
            while node_cur is not None:  # keep going until root
                path.append(node_cur)  # follow the convention to output matrix column wise
                parent = node_cur.parent
                node_cur = parent
            path.reverse()

            # save path to text file
            first_line = 'index, node loc, heuristic\n'
            file.write(first_line)
            for i, node_ in enumerate(path):  # keep going until root
                node_info = str(i) + ', (' + str(node_) + '), ' + str(node_.heuristic)
                file.write(node_info + '\n')  # follow the convention to output matrix column wise

            # show found path
            if show:
                for i, node_ in enumerate(path):
                    img[tuple(node_.state)] = 255
                    cv2.imshow('highlight optimal path', cv2.flip(img, 0))
                    if i == len(path)-1:
                        cv2.waitKey(0)
                    else:   # hold on for user to input
                        cv2.waitKey(10)
        return

    def start_or_goal_in_obstacle(self, state_init, state_goal):
        return self.graph[state_init] or self.graph[state_goal]
