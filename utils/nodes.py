import numpy as np


class node:
    def __init__(self, state, parent=None):
        """

        :param state: state of the node
        :type state: 1d tuple, list, or np.array
        :param parent: parent of this node
        :type parent: node
        """
        assert isinstance(state, tuple) or isinstance(state, list) or isinstance(state, np.ndarray)
        # assert isinstance(state[0], int) or isinstance(state[0], np.int), str(state) + str(type(state[0]))
        self.state = tuple(state)
        self._parent = parent

    def get_state(self):
        return self.state

    def get_parent(self):
        return self._parent

    def expand(self, robot_, space):
        """
        produce a list of child node form this node, filtered by search space, such as obstacle, boundary
        :param robot_: the robot that planning for
        :type robot_: robot
        :param space: the space that planning for, should contain obstacle
        :type space:
        :return: a list of nodes that current node can really expand
        :rtype: list
        """
        children, cost_2_move = [], []   # children list to return
        state = self.state  # current node state
        loc = state[0:2]    # current node location

        # assert isinstance(loc[0], int)
        states_next, dises_move = robot_.next_moves_virtual(self.state)
        for state_next, dis_move in zip(states_next, dises_move):  # get a list of possible next move
            state_next = tuple(state_next)
            loc_next = state_next[0:2]
            if space.invalidArea(robot_.teleport(state_next)):  # filter the next possible move by space and obstacle
                children.append(node(state_next, parent=self))
                cost_2_move.append(dis_move)
        return children, cost_2_move

    # to string
    def __str__(self):
        return str(self.state)[1:-1]

    def __eq__(self, another):
        assert isinstance(another, node)
        # assert type(self.state[0]) == type(another.state[0]), 'class state type ' + str(type(self.state[0])) + ' not comparable with input state type' + str(type(another.state[0]))
        return self.state == another.state



class nodeHeuristic(node):
    def __init__(self, state, heuristic, parent=None):
        super().__init__(state, parent)
        self.heuristic = heuristic

    def get_heuristic(self):
        return self.heuristic

    def update_heuristic(self, heuristic):
        self.heuristic = heuristic

    def expand(self, robot_, space, map_visited=None, map_heuristic_candidate=None, map_cost_to_here=None, map_cost_to_goal=None):
        """
        produce a list of child node form this node, filtered by search space, such as obstacle, boundary, then filtered
        by settled or not
        :param robot_: the robot that planning for
        :type robot_: robot
        :param space: the space that planning for, should contain obstacle
        :type space:
        :param map_visited: the numpy array that represent the visited node
        :type map_visited: np.array
        :param map_heuristic_candidate:
        :type map_heuristic_candidate:
        :param map_cost_to_here: min cost form start point to each point in search space, if not find yet
        :type map_cost_to_here: np.array
        :param map_cost_to_goal: cost form each point in search space to goal location
        :type map_cost_to_goal: np.array
        :return: a list of nodes that current node can really expand
        :rtype: list
        """
        assert isinstance(map_visited, np.ndarray) and len(map_visited.shape) == 3, map_visited
        assert isinstance(map_cost_to_here, np.ndarray) and len(map_cost_to_here.shape) == 3
        assert isinstance(map_cost_to_goal, np.ndarray) and len(map_cost_to_goal.shape) == 3

        """get a list of valid child, possible settled"""
        children, distances = super().expand(robot_=robot_, space=space)
        children_heuristic = []
        """if not settled, assign each child with heuristic"""
        loc_current = self.state[0:2]
        for child, dis_from_current_2_child in zip(children, distances):  # get a list of possible next move
            loc_child = child.get_state()[0:2]
            if not map_visited[loc_child]:   # if shortest distance from this node is not determined
                children_heuristic.append(__node_2_node_heuristic__(child, heuristic=min(map_heuristic_candidate[loc_child], map_cost_to_here[loc_current] + distances + map_cost_to_goal[loc_child])))
            else:   # this node is visited before
                if map_heuristic_candidate[loc_child] > map_cost_to_here[loc_current] + distances + map_cost_to_goal[loc_child]:
                    children_heuristic.append(__node_2_node_heuristic__(child, heuristic=map_cost_to_here[loc_current] + distances + map_cost_to_goal[loc_child]))
        """sort the candidates list"""

        return children

    def __lt__(self, other):
        assert isinstance(self.heuristic, int) or isinstance(self.heuristic, float)
        assert isinstance(other.heuristic, int) or isinstance(other.heuristic, float)
        return self.heuristic < other.heuristic

    def __gt__(self, another):
        assert isinstance(another, nodeHeuristic)
        assert self.state.dtype == another.state.dtype, 'class state type ' + str(
            self.state.dtype) + ' not comparable with input state type' + str(another.state.dtype)
        return self.heuristic > another.heuristic


def __node_2_node_heuristic__(node_, heuristic):
    return nodeHeuristic(node_.get_state(), heuristic=heuristic, parent=node_.parent)
