import math
from shapely.geometry import Point, Polygon
import numpy as np
import cv2


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


class map_2D:
    def __init__(self, height=300, width=400):
        self.size = (height, width)

    def invalidArea(self, robot_):
        return 0 < robot_.loc()[0] < self.size[0] and 0 < robot_.loc()[1] < self.size[1]


class map_2D_with_obstacle(map_2D):
    def __init__(self, height=300, width=400):
        super().__init__(height, width)
        self.map_obstacle = np.zeros(self.size, dtype=np.bool)  # mask obstacle in map. 1 is free space, 0 is obstacle

    def invalidArea(self, robot_):
        return super().invalidArea(robot_) and not self.map_obstacle[robot_.loc()]

    def isfree(self, index):
        index = index if isinstance(index, tuple) else tuple(index)
        assert len(index) == len(self.size) and isinstance(index[0], int)
        return not self.map_obstacle[index]

    def get_map_obstacle(self):
        return np.copy(self.map_obstacle)

    def add_rectangle_obstacle(self, corner_ll, width, height, angle):
        angle *= math.pi / 180
        arctan_h_w = math.atan(height / width)
        diag = math.sqrt(width ** 2 + height ** 2)
        corner_lr = (corner_ll[0] + width * math.cos(angle), corner_ll[1] + width * math.sin(angle))
        corner_ul = (
            corner_ll[0] + height * math.cos(angle + math.pi / 2),
            corner_ll[1] + height * math.sin(angle + math.pi / 2))
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

    def numpy_array_representation(self):
        img = np.ones(self.size) * 200
        img[self.map_obstacle] = 0
        # draw each obstacle on display window
        img = cv2.flip(img, 0)
        return img

    def show(self):
        cv2.imshow('obstacle', self.numpy_array_representation())
        if cv2.waitKey(0) & 0xFF == ord('q'):
            return
