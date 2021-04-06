import numpy as np
# from robotPlanning.robot import robot, point_robot, rigid_robot
# import queue
import cv2
import copy

debug_showmap = True
debug_nodeinfo = True


class node:
    def __init__(self, state, parent_state=None):
        self.state = state if type(state) == np.ndarray else np.array(state)
        self.parent_state = parent_state

    def get_state(self):
        return tuple(self.state)

    def expand(self, robot_, map_):
        childern = []
        robot_.teleport(self.state)     # make a robot at this state, for movement at this node state
        for move in robot_.actionset():
            robot_copy = robot_.copy()
            move(robot_copy)
            if map_.invalidArea(robot_copy):
                childern.append(node(robot_copy.loc()))
        return childern

    # to string
    def __str__(self):
        return str(self.state)[1:-1]

    def __eq__(self, another):
        assert isinstance(another, node)
        assert self.state.dtype == another.state.dtype, 'class state type ' + str(self.state.dtype) + ' not comparable with input state type' + str(another.state.dtype)
        return np.alltrue(self.state == another.state)


class node_heuristic(node):
    def __init__(self, state, heuristic, parent=None):
        super().__init__(state, parent)
        self.heuristic = heuristic

    def get_heuristic(self):
        return self.heuristic

    def update_heuristic(self, heuristic):
        self.heuristic = heuristic

    def expand(self, robot_, map_):
        children = []
        robot_.teleport(self.state)  # make a robot at this state, for movement at this node state
        for move in robot_.actionset():
            robot_copy = robot_.copy()
            dist = move(robot_copy)
            if map_.invalidArea(robot_copy):
                children.append(node_heuristic(robot_copy.loc(), self.heuristic+dist))
        return children

    def __lt__(self, other):
        return self.heuristic < other.heuristic

    def __gt__(self, another):
        assert isinstance(another, node_heuristic)
        assert self.state.dtype == another.state.dtype, 'class state type ' + str(
            self.state.dtype) + ' not comparable with input state type' + str(another.state.dtype)
        return self.heuristic > another.heuristic


class bfs:  # searching algorithm object
    def __init__(self, retrieve_goal_node=False):
        self.retrieve_goal_node = retrieve_goal_node
        print('searching ready')


    def search(self, state_init, state_goal, robot_, map_):
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
            print("start location or goal location in or too close to obstacle")
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
                    self.retrievePath(node_cur, img=visited, filename="results/optimalPath.txt")
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
            # if show:
            #     for i, node_ in enumerate(path):
            #         img[tuple(node_.state)] = 255
            #         cv2.imshow('highlight optimal path', cv2.flip(img, 0))
            #         if i == len(path)-1:
            #             cv2.waitKey(0)
            #         else:   # hold on for user to input
            #             cv2.waitKey(10)
        return

    def start_or_goal_in_obstacle(self, state_init, state_goal):
        return self.map_[state_init] or self.map_[state_goal]


class Dijkstra(bfs):  # searching algorithm object
    def __init__(self, retrieve_goal_node=False):
        super().__init__(retrieve_goal_node)


    def search(self, state_init, state_goal, robot_, map_):
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
            print("start location or goal location in or too close to obstacle")
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
                    self.retrievePath(node_cur, img=visited, filename="results/optimalPath.txt")
                return node_cur
            else:
                if debug_nodeinfo:
                    print(i, " step, reach ", node_cur)
                if debug_showmap:
                    cv2.imshow('highlight explored', cv2.flip(visited, 0))
                    cv2.waitKey(1)
                children = node_cur.expand(robot_, map_)  # children expanded from this node
                for child in children:  # for each child
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


    def search(self, state_init, state_goal, robot_, map_):
        """
        A* searching
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
            print("start location or goal location in or too close to obstacle")
            return

        # initialize control variables for searching
        node_start = node_heuristic(state_init, heuristic=0)    # start node
        node_goal = node_heuristic(state_goal, heuristic=0)     # goal node
        visited = np.zeros(map_.size, dtype=np.uint8)           # map mask of visited node
        visited[map_.get_map_obstacle()] = 75                   # mark obstacle as 75
        nodes_ = [node_start]  # min priority queue of node, priority is heuristics

        # initialize a cost function from point on map to goal
        cost_to_goal = self.dis_to_point(state_goal, map_)
        # heuristic = np.zeros(map_.size, dtype=np.int)
        # heuristic[:, :] = np.inf

        i = 0
        while nodes_:  # as long as stack is not empty, keep dfs
            i += 1
            if i % 500 == 0:
                print('loop', i)

            node_cur = pop_min(nodes_)      # take the node with smallest heuristic from list
            if node_cur == node_goal:           # reach a goal
                if self.retrieve_goal_node:     # show optimal path
                    self.retrievePath(node_cur, img=visited, filename="results/optimalPath.txt")
                return node_cur
            else:
                if debug_nodeinfo:
                    print(i, " step, reach ", node_cur)
                if debug_showmap:
                    cv2.imshow('highlight explored', cv2.flip(visited, 0))
                    cv2.waitKey(1)
                children = node_cur.expand(robot_, map_)  # children expanded from this node
                for child in children:  # for each child
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

    # return a numpy array representation of distance from points on map to input point
    def dis_to_point(self, point, map_):
        # assert isinstance(map_, map_with)
        assert isinstance(point, tuple) and len(point) == len(map_.size)

        point_np = np.array(point)
        dis_to_goal = np.zeros(map_.size, dtype=np.float)  # distances map
        dis_to_goal[:, :] = np.inf                         # initialize all distance to inf

        obstacle = map_.get_map_obstacle()
        for i in range(map_.shape[0]):
            for j in range(map_.shape[1]):
                if obstacle[i, j]:  # point not in obstacle
                    point_map = np.array((i, j))
                    dis_to_goal[i, j] = np.linalg.norm(point_map-point_np)
        print(dis_to_goal)
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



