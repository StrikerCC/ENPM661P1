import numpy as np
import copy
import math

debug = True


class robot:
    def __init__(self, state=None):
        self.state = None
        if isinstance(state, np.ndarray):
            self.state = state
        elif isinstance(state, list):
            self.state = np.array(state)

    def loc(self):
        return tuple(self.state)

    def teleport(self, state):
        if isinstance(state, np.ndarray):
            self.state = state
        elif isinstance(state, list):
            self.state = np.array(state)
        elif isinstance(state, tuple):
            self.state = np.array(state)
        else:
            AssertionError('cannot recognize input state', state)
        return self

    def actionset(self):
        return [point_robot.moveUpRight,
                point_robot.move_low_right,
                point_robot.move_up_left,
                point_robot.move_low_left,
                point_robot.move_up, point_robot.move_right,
                point_robot.move_down, point_robot.move_left]

    def move_left(self):
        self.state[1] -= 1
        return 1

    def move_right(self):
        self.state[1] += 1
        return 1

    def move_up(self):
        self.state[0] += 1
        return 1

    def move_down(self):
        self.state[0] -= 1
        return 1

    def move_low_left(self):
        self.move_left()
        self.move_down()
        return math.sqrt(2)

    def move_low_right(self):
        self.move_right()
        self.move_down()
        return math.sqrt(2)

    def moveUpRight(self):
        self.move_up()
        self.move_right()
        return math.sqrt(2)

    def move_up_left(self):
        self.move_up()
        self.move_left()
        return math.sqrt(2)

    def copy(self):
        return robot(copy.deepcopy(self.state))

    def __str__(self):
        return str(self.loc())[1:-1]


class point_robot(robot):
    def __init__(self, state=None):
        super().__init__(state)

    def copy(self): return point_robot(copy.deepcopy(self.state))


class rigid_robot(robot):
    def __init__(self, state=None, clearance=0):
        """

        :param state:
        :type state:
        :param clearance: closet distance from robot center to obstacle to avoid collision
        :type clearance: int or float
        """
        super().__init__(state)
        self.clearanc = clearance

    def clearance(self): return self.clearanc

    def copy(self): return rigid_robot(copy.deepcopy(self.state), self.clearance())
