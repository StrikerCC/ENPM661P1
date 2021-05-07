import heapq
import queue

import math
import numpy as np
# from utils.robot import point_robot, rigid_robot
# import queue
import cv2
import copy
import datetime

from utils.nodes import node, nodeHeuristic

debug_showmap = True
debug_nodeinfo = False


class bfs:  # searching algorithm object
    def __init__(self, retrieve_goal_node=False, filepath=None):
        self.retrieve_goal_node = retrieve_goal_node
        print('searching ready')


    def search(self, state_init, state_goal, robot_, map_, filepath=None):
        """
        breath first searching
        :param state_init: initial state
        :type state_init:
        :param state_goal: goal state
        :type state_goal:
        :param robot_: robot type
        :type robot_:
        :param map_: graph of utils field
        :type map_:
        :return:
        :rtype:
        """

        # make sure initial and goal state is not in obstacle
        # robot_.teleport(state_init)
        # if not map_.invalidArea(robot_):
        #     print("start location or goal location in or too close to obstacle", state_init, state_goal)
        #     return

        # initialize control variables for searching
        node_start = node(state_init)  # start node
        node_goal = node(state_goal)  # goal node
        visited = np.zeros(map_.size, dtype=np.uint8)  # map mask of visited node
        visited[map_.get_map_obstacle()] = 75   # mark obstacle as 75
        visited[state_goal] = 255   # mark goal as 255
        breath = [node_start]  # min priority queue of node, priority is heuristics

        i = 0
        while breath:   # as long as stack is not empty, keep dfs
            i += 1
            if i % 500 == 0:
                print('loop', i)

            node_cur = breath.pop(0)  # take a node from stack
            if node_cur == node_goal:  # reach a goal
                optimal_path = self.retrievePath(node_cur, img=visited)
                return optimal_path
            else:
                if debug_nodeinfo:
                    print(i, " step, reach ", node_cur)
                if debug_showmap:
                    cv2.imshow('highlight explored', cv2.flip(visited, 0))
                    cv2.waitKey(1)

                children, _ = node_cur.expand(robot_, map_)  # children expanded from this node
                for child in children:  # for each child
                    if visited[child.get_loc()] != 155 and visited[child.get_loc()] != 75:  # this child represent a new state never seen before
                        visited[child.get_loc()] = 155    # mark visited as 155
                        breath.append(child)

        # couldn't find a valid solution
        print("run out of nodes")
        return None

    # return parents of this node as a list
    def retrievePath(self, node_, img, filename=None):
        if not self.retrieve_goal_node:
            print('cannot retrieve path because no recording')
            return None
        file = open(filename, 'w')

        """recursive go to parent"""
        if not node_:
            file.write('no solution with this map\n')
            return None
        else:
            path, node_cur = [], node_
            i = 0
            while node_cur is not None:  # keep going until root
                print(i)
                i += 1
                path.append(node_cur)  # follow the convention to output matrix column wise
                parent = node_cur.get_parent()
                node_cur = parent
            path.reverse()

            # save path to text file
            first_line = 'time: ' + str(datetime.datetime) + 'index, node loc, heuristic\n'
            file.write(first_line)
            for i, node_ in enumerate(path):  # keep going until root
                node_info = ''
                if isinstance(node_, nodeHeuristic):
                    node_info = str(i) + ', (' + str(node_) + '), ' + str(node_.heuristic)
                else:
                    node_info = str(i) + ', (' + str(node_)
                file.write(node_info + '\n')  # follow the convention to output matrix column wise

            # show found path
            # if show:
            for i, node_ in enumerate(path):
                loc = node_.get_loc()
                img[loc] = 255
                cv2.imshow('highlight optimal path', cv2.flip(img, 0))
                if i == len(path)-1:
                    cv2.waitKey(100)
                else:   # hold on for user to input
                    cv2.namedWindow('highlight optimal path', )
                    cv2.waitKey(10)
            return path


class Dijkstra(bfs):  # searching algorithm object
    def __init__(self, retrieve_goal_node=False):
        super().__init__(retrieve_goal_node)

    def search(self, state_init, state_goal, robot_, map_, filepath=None):
        """
        Dijkstra searching
        :param state_init: initial state
        :type state_init:
        :param state_goal: goal state
        :type state_goal:
        :param robot_: robot type
        :type robot_:
        :param map_: graph of utils field
        :type map_:
        :return:
        :rtype:
        """
        # make sure initial and goal state is not in obstacle
        robot_.teleport(state_init)
        if not map_.invalidArea(robot_) or not map_.invalidArea(robot_):
            print("start location or goal location in or too close to obstacle", state_init, state_goal)
            return

        # initialize control variables for searching
        node_start = nodeHeuristic(state_init, cost_from_start=0)    # start node
        node_goal = nodeHeuristic(state_goal, cost_from_start=np.inf)     # goal node
        visited = np.zeros(map_.size, dtype=np.uint8)           # map mask of visited node
        visited[map_.get_map_obstacle()] = 75                   # mark obstacle as 75

        nodes_ = [node_start]  # min priority queue of node, priority is heuristics

        i = 0
        while nodes_:  # as long as stack is not empty, keep dfs
            i += 1
            if i % 500 == 0:
                print('loop', i)

            node_cur = pop_min(nodes_)      # take the node with smallest heuristic from list
            if node_cur == node_goal:           # reach a goal
                print('find the solution')
                if self.retrieve_goal_node:     # show optimal path
                    self.retrievePath(node_cur, img=visited, filename=filepath)
                return node_cur
            else:
                if debug_nodeinfo:
                    print(i, " step, reach ", node_cur)
                if debug_showmap:
                    cv2.imshow('highlight explored', cv2.flip(visited, 0))
                    cv2.waitKey(1)
                children = node_cur.expand(robot_, map_)  # children expanded from this node
                for child in children:  # for each possible next state, filter again using visited node
                    if visited[child.get_loc()] == 0:  # this child represent a new state
                        visited[child.get_loc()] = 155  # mark visited as 155
                        nodes_.append(child)
                    else:   # this is a visited not expanded node
                        for node_ in nodes_:  # find the node in list
                            if child == node_:  # find a repeated node , update its heuristic
                                node_.update_heuristic(cost_from_start=min(node_.get_heuristic(), child.get_heuristic()),
                                                       cost_to_goal=0)

        # couldn't find a valid solution
        print("run out of nodes")
        return None




class Astart(bfs):
    def __init__(self, retrieve_goal_node=False):
        super().__init__(retrieve_goal_node)
        self.color_obstacle = (0, 0, 255)
        self.color_visited = (150, 150, 0)
        self.color_edge = (50, 50, 50)

    def search(self, state_init, state_goal, robot_, map_, tolerance=5, filepath=None):
        """
        A* searching
        :param state_init: initial state
        :type state_init: tuple or list
        :param state_goal: goal state
        :type state_goal: tuple or list
        :param robot_: robot type
        :type robot_:
        :param map_: graph of utils field
        :type map_: map or mapWithObstacle from utils moduel
        :return:
        :rtype:
        """

        # make sure initial and goal state is not in obstacle
        robot_.teleport(state_init)
        if not map_.invalidArea(robot_) or not map_.invalidArea(robot_):
            print("start location or goal location in or too close to obstacle", state_init, state_goal)
            return

        map_cost_to_goal = self.__initialzie_map_cost_to_goal__(state_goal[0:2], map_)      # initialize a cost function from point on map to goal
        # map_cost_to_goal = np.zeros(map_.shape)      # initialize a cost function from point on map to goal
        costs_from_start_to_here = self.__init_map_cost_from_start(map_)  # map mask of visited node

        # initialize control variables for searching
        node_start = nodeHeuristic(state_init, cost_from_start=0, cost_to_goal=map_cost_to_goal[robot_.get_loc()])  # start node
        node_goal = nodeHeuristic(state_goal, cost_from_start=np.inf, cost_to_goal=0)  # goal node

        visited_loc = np.zeros(map_.size, dtype=bool)           # map mask of visited node

        expanded = np.zeros(map_.size, dtype=bool)

        map_goal = self.__initialzie_map_reach_goal(state_goal, tolerance=tolerance, size=map_.shape)     # mask of goal
        state_map_search = self.__initialzie_search_map(map_)     # numpy representation of searching state

        # open_set = [node_start]
        open_set = []
        heapq.heappush(open_set, (node_start.get_heuristic(), node_start))

        i = 0
        while open_set:  # as long as stack is not empty, keep dfs
            i += 1
            if i % 1000 == 0: print('loop', i, 'current keeping', len(open_set))

            """take the node having smallest cost"""
            _, node_cur = heapq.heappop(open_set)        # take the node with smallest heuristic from list
            loc_cur = node_cur.get_loc()
            expanded[loc_cur] = True
            # visited[loc_cur] = True

            if map_goal[loc_cur].any():               # reach a goal
                break
            else:
                for child in node_cur.expand(robot_, map_, map_cost_to_goal):             # children expanded from this node
                    if map_.get_map_obstacle()[child.get_loc()]: # or expanded[child.get_loc()]:
                        continue

                    if self.__update_map_cost_from_start(node_=child, map_cost_from_start=costs_from_start_to_here): # this child have a new heuristic
                        # node_.update_heuristic(cost_from_start=min(node_.cost_from_start, child.cost_from_start),
                        #                        cost_to_goal=map_cost_to_goal[child.get_loc()])
                        heapq.heappush(open_set, (self.__get_cost_from_start(node_=child, map_cost_from_start=costs_from_start_to_here), child))
                        visited_loc[child.get_loc()] = True

            self.__update_search_map__(state_map_search, visited_loc, map_, expanded)
            cv2.imshow('highlight explored', cv2.flip(state_map_search, 0))
            cv2.waitKey(1)

        """search space should have some node left if answer has been found, right?"""
        if len(open_set) > 0:
            print('path found')
            if self.retrieve_goal_node:  # show optimal path
                self.retrievePath(node_cur, img=state_map_search, filename=filepath)
            return node_cur
        else:
            # couldn't find a valid solution
            print("run out of nodes, couldn't find a solution")
            return None

    def __initialzie_map_cost_to_goal__(self, state_goal, map_):
        """
        make a map of cost to goal
        :param state_goal: where you wanna go
        :type state_goal: tuple or list
        :param map_: map that contain map info
        :type map_: map or mapWithObstacle from utils moduel
        :return: a map of cost to goal location
        :rtype: np.array
        """
        heuristic_init = dis_to_point(state_goal, map_)
        print(heuristic_init)
        return heuristic_init

    def __initialzie_map_reach_goal(self, state_goal, tolerance, size):
        map_ = np.zeros(size, dtype=bool)
        map_[int(state_goal[0]-tolerance):int(state_goal[0]+tolerance), int(state_goal[1]-tolerance):int(state_goal[1]+tolerance)] = True
        return map_

    def __initialzie_search_map(self, map_):
        search_map = np.zeros((map_.size[0], map_.size[1], 3), dtype=np.uint8)
        search_map[map_.get_map_obstacle()] = self.color_obstacle
        return search_map

    def __update_search_map__(self, search_map, visited, map_, expanded):
        search_map[visited] = self.color_visited
        search_map[map_.get_map_obstacle()] = self.color_obstacle
        search_map[expanded] = self.color_edge

    def __init_map_cost_from_start(self, map_):
        a = np.zeros((map_.size[0], map_.size[1], 4), dtype=float)
        a[::] = np.inf
        return a

    def __get_cost_from_start(self, node_, map_cost_from_start):
        """convert radius to degree, linearly interpolate - pi is 0, pi is upper limit"""
        loc = node_.get_loc()
        theta = node_.get_state()[-1]

        theta_resolution = map_cost_from_start.shape[-1]
        if theta > np.pi: theta = theta - 2 * np.pi
        if theta < -np.pi: theta = theta + 2 * np.pi
        theta = int((theta + np.pi) / np.pi * theta_resolution) % map_cost_from_start.shape[-1]

        assert 0 <= theta < theta_resolution, str(node_.get_state()) + ' ' + str(theta)
        return map_cost_from_start[loc[0], loc[1], theta]

    def __update_map_cost_from_start(self, node_, map_cost_from_start):
        """convert radius to degree, linearly interpolate - pi is 0, pi is upper limit"""
        loc = node_.get_loc()
        theta = node_.get_state()[-1]

        theta_resolution = map_cost_from_start.shape[-1]
        if theta > np.pi: theta = theta - 2 * np.pi
        if theta < -np.pi: theta = theta + 2 * np.pi
        theta = int((theta + np.pi) / np.pi * theta_resolution) % map_cost_from_start.shape[-1]

        assert 0 <= theta < theta_resolution, str(node_.get_state()) + ' ' + str(theta)
        if node_.get_heuristic() < map_cost_from_start[loc[0], loc[1], theta]:
            map_cost_from_start[loc[0], loc[1], theta] = node_.get_heuristic()
            return True
        return False


class RRT():
    def __init__(self):
        self.robot = None
        self.space = None
        self.node_start, self.node_goal = None, None
        self.goal_sample_rate = 0.5
        self.step_len = 10

        self.color_obstacle = (0, 0, 100)
        self.color_visited = (250, 250, 0)
        self.color_edge = (250, 0, 250)
        self.color_goal = (255, 255, 255)

    def search(self, state_init, state_goal, robot_, map_, tolerance=5, filepath=None):
        # make sure initial and goal state is not in obstacle
        robot_.teleport(state_init)
        if not map_.invalidArea(robot_) or not map_.invalidArea(robot_):
            print("start location or goal location in or too close to obstacle", state_init, state_goal)
            return

        self.node_start = node(state_init)  # start node
        self.node_goal = node(state_goal)  # goal node
        self.space = map_
        visited_loc = np.zeros(map_.size, dtype=bool)  # map mask of visited node
        # expanded = np.zeros(map_.size, dtype=bool)

        map_goal = self._initialzie_map_reach_goal(state_goal, tolerance=tolerance, size=map_.shape)  # mask of goal
        state_map_search = self._initialzie_search_map(map_, map_goal)  # numpy representation of searching state

        # open_set = [node_start]
        vertexes = [self.node_start]

        for i in range(10000):  # as long as stack is not empty, keep dfs
            i += 1
            if i % 1000 == 0:
                print('loop', i, 'current keeping', len(vertexes))
            node_random = self._generate_random_node()                      # get a random node
            node_closest = self._nearest_neighbor(vertexes, node_random)     # take the node having closest to the random node
            node_ = self._new_node(node_closest, node_random, robot_, space=map_)               # make new node based on closest node and robot action set
            loc_cur = node_.get_loc()
            if not visited_loc[loc_cur]:                                    # new node found
                if map_goal[loc_cur].any():                                 # reach a goal
                    break
                else:
                    if not map_.get_map_obstacle()[node_.get_loc()]:        # check collision
                        visited_loc[loc_cur] = True
                        vertexes.append(node_)

                        """visualization"""
                        self._update_search_map(state_map_search, node_)
                        cv2.namedWindow("highlight explored", cv2.WINDOW_NORMAL)
                        cv2.imshow('highlight explored', cv2.flip(state_map_search, 0))
                        cv2.waitKey(1)

        """search space should have some node left if answer has been found, right?"""
        cv2.imshow('highlight explored', cv2.flip(state_map_search, 0))
        cv2.waitKey(0)

    def _initialzie_map_reach_goal(self, state_goal, tolerance, size):
        map_ = np.zeros(size, dtype=bool)
        map_[int(state_goal[0] - tolerance):int(state_goal[0] + tolerance),
        int(state_goal[1] - tolerance):int(state_goal[1] + tolerance)] = True
        return map_

    def _initialzie_search_map(self, map_, map_goal):
        search_map = np.zeros((map_.size[0], map_.size[1], 3), dtype=np.uint8)
        search_map[map_.get_map_obstacle()] = self.color_obstacle
        search_map[map_goal] = self.color_goal
        return search_map

    def _update_search_map(self, search_map, node_):
        loc = node_.get_loc()
        loc_parent = (0, 0)
        if node_.get_parent() is not None:
            loc_parent = node_.get_parent().get_loc()
        search_map = cv2.line(search_map, loc[::-1], loc_parent[::-1], self.color_edge, thickness=1)
        search_map = cv2.circle(search_map, center=loc[::-1], color=self.color_visited, radius=1)

    def _generate_random_node(self):
        self.delta = 10
        if np.random.random() > self.goal_sample_rate:
            x_range, y_range = self.space.size
            return node(
                (np.random.uniform(0 + self.delta, y_range - self.delta),
                 np.random.uniform(0 + self.delta, y_range - self.delta))
            )
        else:
            return self.node_goal

    @staticmethod
    def _nearest_neighbor(node_list, node_):
        """

        :param node_list: current expanded node
        :type node_list:
        :param node_: random generated node
        :type node_:
        :return:
        :rtype:
        """
        return node_list[int(np.argmin([math.hypot(nd.get_loc()[0] - node_.get_loc()[0], nd.get_loc()[1] - node_.get_loc()[1])
                                        for nd in node_list]))]

    def _new_node(self, node_start, node_end, robot_, space):
        """

        :param node_start: node to expand from
        :type node_start: node
        :param node_end: node to expand toward to
        :type node_end:
        :return:
        :rtype:
        """
        # dist, theta = self._get_distance_and_angle(node_start, node_end)
        # dist = min(self.step_len, dist)
        # node_new = node((node_start.get_loc()[0] + dist * math.cos(theta),
        #                  node_start.get_loc()[1] + dist * math.sin(theta)),
        #                 parent=node_start)
        # return node_new

        state_next, cost_next = robot_.move_toward(node_start.get_state(), node_end.get_state(), space)
        node_new = node(state_next, parent=node_start)
        return node_new


    @staticmethod
    def _get_distance_and_angle(node_start, node_end):
        dx = node_end.get_loc()[0] - node_start.get_loc()[0]
        dy = node_end.get_loc()[1] - node_start.get_loc()[1]
        return math.hypot(dx, dy), math.atan2(dy, dx)


# return a numpy array representation of distance from points on map to input point
def dis_to_point(point, map_):
    # assert isinstance(map_, map_with)
    assert isinstance(point, tuple) or isinstance(point, list) and len(point) == len(map_.size)

    point_np = np.array(point)
    dis_to_goal = np.zeros(map_.size, dtype=np.float)  # distances map
    dis_to_goal[:, :] = np.inf                         # initialize all distance to inf

    obstacle = map_.get_map_obstacle()
    for i in range(map_.shape[0]):
        for j in range(map_.shape[1]):
            if not obstacle[i, j]:  # point not in obstacle
                point_map = np.array((i, j))
                dis_to_goal[i, j] = np.linalg.norm(point_map-point_np)
    print(np.round(dis_to_goal, decimals=2))
    return dis_to_goal

#
def pop_min(list_):
    if not list_: return None
    i_min, ele_min = 0, list_[0]
    for i in range(len(list_)):
        if list_[i] < ele_min:
            ele_min = list_[i]
            i_min = i
    return list_.pop(i_min)
