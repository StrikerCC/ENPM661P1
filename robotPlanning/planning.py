import numpy as np
# from robotPlanning.robot import robot, point_robot, rigid_robot
# import queue
import cv2
import copy

from robotPlanning.nodes import node, node_heuristic

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
        :param map_: graph of robotPlanning field
        :type map_:
        :return:
        :rtype:
        """

        # make sure initial and goal state is not in obstacle
        robot_.teleport(state_init)
        if not map_.invalidArea(robot_):
            print("start location or goal location in or too close to obstacle", state_init, state_goal)
            return

        # initialize control variables for searching
        node_start = node(state_init)  # start node
        node_goal = node(state_goal)  # goal node
        visited = np.zeros(map_.size, dtype=np.uint8)  # map mask of visited node
        visited[map_.get_map_obstacle()] = 75   # mark obstacle as 75
        breath = [node_start]  # min priority queue of node, priority is heuristics

        i = 0
        while breath:   # as long as stack is not empty, keep dfs
            i += 1
            if i % 500 == 0:
                print('loop', i)

            node_cur = breath.pop(0)  # take a node from stack
            if node_cur == node_goal:  # reach a goal
                if self.retrieve_goal_node:  # show optimal path
                    self.retrievePath(node_cur, img=visited, filename=filepath)
                return node_cur
            else:
                if debug_nodeinfo:
                    print(i, " step, reach ", node_cur)
                if debug_showmap:
                    cv2.imshow('highlight explored', cv2.flip(visited, 0))
                    cv2.waitKey(1)

                children = node_cur.expand(robot_, map_)  # children expanded from this node
                for child in children:  # for each child
                    if visited[child.get_state()] == 0:  # this child represent a new state never seen before
                        visited[child.get_state()] = 155    # mark visited as 155
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
        if not node_:
            file.write('no solution with this map\n')
        else:
            path, node_cur = [], node_
            i = 0
            while node_cur is not None:  # keep going until root
                print(i)
                i += 1
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
            # if show:
            for i, node_ in enumerate(path):
                img[tuple(node_.state)] = 100
                cv2.imshow('highlight optimal path', cv2.flip(img, 0))
                if i == len(path)-1:
                    cv2.waitKey(0)
                else:   # hold on for user to input
                    cv2.waitKey(10)
        return


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
        :param map_: graph of robotPlanning field
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
        node_start = node_heuristic(state_init, heuristic=0)    # start node
        node_goal = node_heuristic(state_goal, heuristic=0)     # goal node
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
                    if visited[child.get_state()] == 0:  # this child represent a new state
                        visited[child.get_state()] = 155  # mark visited as 155
                        nodes_.append(child)
                    else:   # this is a visited not expanded node
                        for node_ in nodes_:  # find the node in list
                            if child == node_:  # find a repeated node , update its heuristic
                                node_.update_heuristic(min(node_.get_heuristic(), child.get_heuristic()))

        # couldn't find a valid solution
        print("run out of nodes")
        return None


class Astart(bfs):
    def __init__(self, retrieve_goal_node=False):
        super().__init__(retrieve_goal_node)
        self.color_obstacle = (255, 255, 255)
        self.color_visited = (50, 50, 50)
        self.color_edge = (0, 0, 255)

    def search(self, state_init, state_goal, robot_, map_, filepath=None):
        """
        A* searching
        :param state_init: initial state
        :type state_init: tuple or list
        :param state_goal: goal state
        :type state_goal: tuple or list
        :param robot_: robot type
        :type robot_:
        :param map_: graph of robotPlanning field
        :type map_: map or mapWithObstacle from robotPlanning moduel
        :return:
        :rtype:
        """

        # make sure initial and goal state is not in obstacle
        robot_.teleport(state_init)
        if not map_.invalidArea(robot_) or not map_.invalidArea(robot_):
            print("start location or goal location in or too close to obstacle", state_init, state_goal)
            return

        # initialize control variables for searching
        node_start = node_heuristic(state_init, heuristic=0)    # start node
        node_goal = node_heuristic(state_goal, heuristic=0)     # goal node
        cost_to_goal = self.initialzie_map_cost_to_goal(state_goal[0:2], map_)      # initialize a cost function from point on map to goal
        visited = np.zeros(map_.size, dtype=bool)           # map mask of visited node
        visited[map_.get_map_obstacle()] = True             # mark obstacle as 75
        map_goal = self.__initialzie_map_reach_goal__(state_goal, tolerance=5, size=map_.shape)
        state_map_search = self.__initialzie_search_map__(map_)     # numpy representation of searching state
        nodes_ = [node_start]  # min priority queue of node, priority is heuristics

        i = 0
        while nodes_:  # as long as stack is not empty, keep dfs
            i += 1
            if i % 500 == 0:
                print('loop', i)

            node_cur = pop_min(nodes_)      # take the node with smallest heuristic from list
            loc_cur = tuple(node_cur.get_state().astype(int)[0:2])
            if map_goal[loc_cur]:           # reach a goal
                if self.retrieve_goal_node:     # show optimal path
                    self.retrievePath(node_cur, img=visited, filename=filepath)
                return node_cur
            else:
                if debug_nodeinfo:
                    print(i, " step, reach ", node_cur)
                if debug_showmap:
                    self.__update_search_map__(state_map_search, visited)
                    cv2.imshow('highlight explored', cv2.flip(state_map_search, 0))
                    cv2.waitKey(1)

                children = node_cur.expand(robot_, map_, cost_to_goal)  # children expanded from this node
                for child in children:                       # filter each child again using visited node
                    state_child = child.get_state()
                    loc_child = tuple(state_child.astype(int)[0:2])
                    if visited[loc_child] == False:      # this child represent a new state
                        visited[loc_child] = True     # mark visited as 155
                        nodes_.append(child)
                        state_map_search = cv2.line(state_map_search, loc_cur[::-1], loc_child[::-1], color=self.color_edge, thickness=2)     # update state map
                    else:   # this is a visited not expanded node
                        for node_ in nodes_:  # find the node in list
                            if child == node_:  # find a repeated node , update its heuristic
                                node_.update_heuristic(min(node_.get_heuristic(), child.get_heuristic()))

        # couldn't find a valid solution
        print("run out of nodes")
        return None

    def initialzie_map_cost_to_goal(self, state_goal, map_):
        """
        make a map of cost to goal
        :param state_goal: where you wanna go
        :type state_goal: tuple or list
        :param map_: map that contain map info
        :type map_: map or mapWithObstacle from robotPlanning moduel
        :return: a map of cost to goal location
        :rtype: np.array
        """
        heuristic_init = dis_to_point(state_goal, map_)
        print(heuristic_init)
        return heuristic_init

    def __initialzie_map_reach_goal__(self, state_goal, tolerance, size):
        map_ = np.zeros(size, dtype=bool)
        map_[state_goal[0]-tolerance, state_goal[1]-tolerance] = True
        return map_

    def __initialzie_search_map__(self, map_):
        search_map = np.zeros((map_.size[0], map_.size[1], 3), dtype=np.uint8)
        search_map[map_.get_map_obstacle()] = self.color_obstacle
        return search_map

    def __update_search_map__(self, search_map, visited):
        search_map[visited] = self.color_visited

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
    # print(np.round(dis_to_goal, decimals=2))
    return dis_to_goal


def pop_min(list_):
    if not list_: return None
    i_min, ele_min = 0, list_[0]
    for i in range(len(list_)):
        if list_[i] < ele_min:
            ele_min = list_[i]
            i_min = i
    return list_.pop(i_min)



# def insert_2_sorted_list(element, sorted_list):
#     # binary insert
#     if element < sorted_list[0]:
#         sorted_list.insert(0, element)
#         return
#     if element > sorted_list[-1]:
#         sorted_list.append(element)
#         return
#     left, right = 0, len(sorted_list)-1
#     while True:
#         mid = int((left + right) / 2)
#         if element > sorted_list[mid]:  # go right
#             left = mid
#         elif element < sorted_list[mid]:  # go left
#             right = mid
#         else:  # same priority element
#             sorted_list.insert(mid, element)
#             break
#         if right-left <= 1:
#             sorted_list.insert(right, element)
#             break



