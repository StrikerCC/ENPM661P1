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
        self.obstacles = []
        self.map_obstacle = np.zeros(self.size, dtype=np.bool)
        self.map_obstacle_expand = np.zeros(self.size, dtype=np.bool)

    def get_map(self):
        return self.map_obstacle

    def get_expanded_map(self):
        return self.map_obstacle_expand

    def add_rectangle_obstacle(self, corner_ll, width, height, angle):
        self.obstacles.append({'type': 'rectangle', 'corner_ll': corner_ll, 'width': width, 'height': height, 'angle': angle})

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
        self.obstacles.append({'type': 'polygon', 'points':points})

        obstacle = geometry('polygon', points)
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if obstacle.inside((i, j)):
                    self.map_obstacle[i, j] = True

    def add_circular_obstacle(self, center, radius):
        self.obstacles.append({'type': 'circular', 'center': center, 'radius': radius})

        obstacle = geometry('circular', {"center": np.array(center)[::-1], "radius": radius})
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if obstacle.inside((i, j)):
                    self.map_obstacle[i, j] = True

    def add_ellipsoid_obstacle(self, center, semi_major_axis, semi_minor_axis):
        self.obstacles.append({'type': 'ellipsoid', 'center': center, 'semi_major_axis': semi_major_axis, 'semi_minor_axis': semi_minor_axis})

        obstacle = geometry('ellipsoid', {"center": np.array(center)[::-1], "semi_major_axis": semi_minor_axis,
                                                     "semi_minor_axis": semi_major_axis})
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if obstacle.inside((i, j)):
                    self.map_obstacle[i, j] = True

    def __getitem__(self, indexes, expand):
        assert len(indexes) == 2
        return self.map_obstacle[indexes] if not expand else self.map_obstacle_expand[indexes]

    def freeToGo(self, point):
        return not self.map_obstacle[tuple(point)]

    def freeToGo_expand(self, point):
        return not self.map_obstacle_expand[tuple(point)]

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




