import numpy as np
# from robotPlanning.robot import robot, point_robot, rigid_robot
# import queue
import cv2

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
    def __init__(self, state, heuristic, parent):
        super().__init__(state, parent)
        self.heuristic = heuristic

    def get_heuristic(self):
        return self.heuristic

    def update_heuristic(self, heuristic):
        self.heuristic = heuristic

    def __lt__(self, other):
        return self.heuristic < other.heuristic

    # def __eq__(self, other):
    #     return self.heuristic == other.heuristic


class bfs:  # searching algorithm object
    def __init__(self, retrieve_goal_node=False):
        self.retrieve_goal_node = retrieve_goal_node
        print('searching ready')

    # initial state
    # goal state
    # robot type
    # graph of robotPlanning field
    def search(self, state_init, state_goal, robot_, map_):
        # make sure initial and goal state is not in obstacle
        if not map_.isfree(state_init) or not map_.isfree(state_goal):
            print("start location or goal location in obstacle")
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

    # initial state
    # goal state
    # robot type
    # graph of robotPlanning field
    def search(self, state_init, state_goal, robot_, map_):
        # make sure initial and goal state is not in obstacle
        if not map_.isfree(state_init) or not map_.isfree(state_goal):
            print("start location or goal location in obstacle")
            return

        # initialize control variables for searching
        node_start = node(state_init)  # start node
        node_goal = node(state_goal)  # goal node
        visited = np.zeros(map_.size, dtype=np.uint8)  # map mask of visited node
        visited[map_.get_map()] = 75
        nodes_sorted = [node_start]  # min priority queue of node, priority is heuristics

        i = 0
        while nodes_sorted:  # as long as stack is not empty, keep dfs
            i += 1
            if i % 500 == 0:
                print('loop', i)

            node_cur = nodes_sorted.pop()  # take a node from stack
            if node_cur == node_goal:  # reach a goal
                print('find a solution')
                if self.retrieve_goal_node:  # show optimal path
                    self.retrievePath(node_cur, img=visited, filename="results/optimalPath.txt")
                return node_cur
            else:
                if debug_nodeinfo:
                    print(i, " step, reach ", node_cur, ', heuristic ', node_cur.heuristic)
                if debug_showmap:
                    # cv2.imshow('highlight explored', cv2.flip(visited, 0))
                    # cv2.waitKey(1)
                    pass
                children = node_cur.expand(robot_, map_)  # children expanded from this node
                for child in children:  # for each child
                    if debug_nodeinfo: print('find new node')
                    if visited[tuple(child.state)] == 0:  # this child represent a new state never seen before
                        visited[tuple(child.state)] = 155
                        nodes_sorted.append(child)
                    else:  # this child is repetitive
                        for node_ in nodes_sorted:  # find the node in list
                            if child == node_:
                                node_.update_heuristic(min(node.get_heuristic()), node_cur.get_heuristic() + node.dist(child))   # update the heuristic
                        nodes_sorted.sort()   # resort the list

        # couldn't find a valid solution
        print("run out of nodes")
        return None
