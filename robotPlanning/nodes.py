import numpy as np


class node:
    def __init__(self, state, parent=None):
        self.state = state if type(state) == np.ndarray else np.array(state)
        self.parent = parent

    def get_state(self):
        return self.state

    def expand(self, robot_, map_):
        childern = []
        robot_.teleport(self.state)  # make a robot at this state, for movement at this node state
        for move in robot_.actionset():
            robot_copy = robot_.copy()
            move(robot_copy)
            if map_.invalidArea(robot_copy):
                child = node(robot_copy.loc(), parent=self)
                childern.append(child)
        return childern

    # to string
    def __str__(self):
        return str(self.state)[1:-1]

    def __eq__(self, another):
        assert isinstance(another, node)
        assert self.state.dtype == another.state.dtype, 'class state type ' + str(
            self.state.dtype) + ' not comparable with input state type' + str(another.state.dtype)
        return np.alltrue(self.state == another.state)


class node_heuristic(node):
    def __init__(self, state, heuristic, parent=None):
        super().__init__(state, parent)
        self.heuristic = heuristic

    def get_heuristic(self):
        return self.heuristic

    def update_heuristic(self, heuristic):
        self.heuristic = heuristic

    def expand(self, robot_, map_, cost_togoal=None):
        """
        produce a list of child node form this node, filtered by search space, such as obstacle, boundary
        :param robot_: robot to move
        :type robot_: robot or it's subclass
        :param map_: search space
        :type map_: map
        :param cost_togoal: cost form each point in search space to goal location
        :type cost_togoal: np.array
        :return: a list of child node form this node
        :rtype: list
        """
        state = self.state
        loc = tuple(state.astype(int)[0:2])
        costtogo = cost_togoal[loc]
        assert isinstance(costtogo, int) or isinstance(costtogo, float)

        cost_to_come_current = self.heuristic - costtogo if cost_togoal is not None else self.heuristic
        children = []
        states_next, dises_move = robot_.next_moves_virtual(self.state)
        for state_next, dis_move in zip(states_next, dises_move):  # get a list of possible next move
            loc_next = tuple(state_next.astype(int)[0:2])
            if map_.invalidArea(robot_.teleport(state_next)):  # filter the next possible move by space and obstacle
                if cost_togoal is not None:  # if cost to goal is considering
                    children.append(
                        node_heuristic(state_next, cost_to_come_current + dis_move + cost_togoal[loc_next], parent=self))
                else:
                    children.append(node_heuristic(state_next, self.heuristic + dis_move, parent=self))
                assert isinstance(children[-1].heuristic, int) or isinstance(children[-1].heuristic, float)
        return children

    def __lt__(self, other):
        assert isinstance(self.heuristic, int) or isinstance(self.heuristic, float)
        assert isinstance(other.heuristic, int) or isinstance(other.heuristic, float)
        return self.heuristic < other.heuristic

    def __gt__(self, another):
        assert isinstance(another, node_heuristic)
        assert self.state.dtype == another.state.dtype, 'class state type ' + str(
            self.state.dtype) + ' not comparable with input state type' + str(another.state.dtype)
        return self.heuristic > another.heuristic
