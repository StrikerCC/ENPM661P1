import numpy as np


class node:
    def __init__(self, state, parent=None):
        """

        :param state: state of the node
        :type state: 1d tuple, list, or np.array
        :param parent: parent of this node
        :type parent: node
        """
        assert isinstance(state, tuple) or isinstance(state, list) or isinstance(state, np.ndarray), type(state)
        # assert isinstance(state[0], int) or isinstance(state[0], np.int), str(state) + str(type(state[0]))
        self.state = state if isinstance(state, np.ndarray) else np.array(state)
        self._parent = parent
        self.valid = True

    def get_valid(self):
        return self.valid

    def set_valid(self, flag):
        assert isinstance(flag, bool)
        self.valid = flag

    def get_state(self): return np.copy(self.state)

    def get_loc(self): return tuple(self.state[0:2].astype(int))

    def get_parent(self):
        return self._parent

    def expand(self, robot_, space):
        """
        produce a list of child node form this node, filtered by search space, such as obstacle, boundary
        :param robot_: the robot that planning for
        :type robot_: robot
        :param space:
        :return: a list of nodes that current node can really expand
        :rtype: list
        """
        children, cost_2_move = [], []   # children list to return
        state = self.state  # current node state

        # assert isinstance(loc[0], int)
        states_next, dises_move = robot_.next_moves(self.state, space=space)     # get a list of possible next move
        children = [node(state_next, parent=self) for state_next in states_next]

        return children, dises_move

    # to string
    def __str__(self):
        return 'state: ' + str(self.state) + ' valid: ' + str(self.valid)

    def __eq__(self, another):
        assert isinstance(another, node)
        # assert type(self.state[0]) == type(another.state[0]), 'class state type ' + str(type(self.state[0])) + ' not comparable with input state type' + str(type(another.state[0]))
        return self.get_loc() == another.get_loc()


class nodeHeuristic(node):
    def __init__(self, state, cost_from_start, cost_to_goal=0, parent=None):
        super().__init__(state, parent)
        self.cost_from_start = cost_from_start
        self.cost_to_goal = cost_to_goal
        self.heuristic = self.cost_to_goal + self.cost_from_start

    def get_heuristic(self):
        return self.heuristic

    def update_heuristic(self, cost_from_start, cost_to_goal):
        self.cost_from_start = cost_from_start
        self.cost_to_goal = cost_to_goal
        self.heuristic = self.cost_to_goal + self.cost_from_start

    def update_parent(self, node_):
        self._parent = node_

    def expand(self, robot_, space, costs_to_goal=None):
        children, distances = super().expand(robot_=robot_, space=space)

        """change child from node to node with heuristic"""
        for i, (child, dis) in enumerate(zip(children, distances)):
            if costs_to_goal is not None:
                child = nodeHeuristic(child.get_state(),
                                      cost_from_start=self.cost_from_start + dis,
                                      cost_to_goal=costs_to_goal[self.get_loc()],
                                      parent=self)
            else:
                child = nodeHeuristic(child.get_state(),
                                      cost_from_start=self.cost_from_start + dis,
                                      cost_to_goal=0,
                                      parent=self)
            children[i] = child

        if children: assert isinstance(children[0], self.__class__), children[0].__class__
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


class node_dynamic(node):
    def __init__(self, state, parent=None):
        super(node_dynamic, self).__init__(state, parent=parent)
        self.valid = True

    def get_valid(self):
        return self.valid

    def set_valid(self, flag):
        assert isinstance(flag, bool)
        self.valid = flag


class edge():
    def __init__(self, start, end):
        assert isinstance(start, node) and isinstance(end, node), str(start) + '    ' + str(end)
        self.start = start
        self.end = end


def __node_2_node_heuristic__(node_, cost_from_start, cost_to_goal):
    """
    make a deep copy of nodeHeuristic
    :param node_:
    :param heuristic:
    :return:
    """
    return nodeHeuristic(node_.get_state(), cost_from_start=cost_from_start, cost_to_goal=cost_to_goal, parent=node_.get_parent)

