import numpy as np
import copy

debug = True


class robot:
    def __init__(self, state):
        self.state = state if type(state) == np.ndarray else np.array(state)

    def loc(self):
        return tuple(self.state)

    def teleport(self, state):
        self.state = state if isinstance(state, np.ndarray) else np.array(state)

    def actionset(self):
        return [point_robot.moveUpRight,
                point_robot.move_low_right,
                point_robot.move_up_left,
                point_robot.move_low_left,
                point_robot.move_up, point_robot.move_right,
                point_robot.move_down, point_robot.move_left]

    def move_left(self):
        self.state[1] -= 1

    def move_right(self):
        self.state[1] += 1

    def move_up(self):
        self.state[0] += 1

    def move_down(self):
        self.state[0] -= 1

    def move_low_left(self):
        self.move_left()
        self.move_down()

    def move_low_right(self):
        self.move_right()
        self.move_down()

    def moveUpRight(self):
        self.move_up()
        self.move_right()

    def move_up_left(self):
        self.move_up()
        self.move_left()

    def copy(self):
        return robot(copy.deepcopy(self.state))

    def __str__(self):
        return str(self.loc())[1:-1]


class point_robot(robot):
    def __init__(self, state):
        super().__init__(state)


class rigid_robot(robot):
    def __init__(self, state, clearance):
        super().__init__(state)
        self.clearanc = clearance

    def clearance(self): return self.clearanc
