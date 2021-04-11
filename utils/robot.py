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
        return tuple(self.state.astype(int))

    def teleport(self, state):
        if type(state) == np.ndarray:
            assert state.shape == (2,) or state.shape == (3,), state
        if type(state) == list or type(state) == tuple:
            assert len(state) == 2 or len(state) == 3, state

        if isinstance(state, np.ndarray):
            self.state = state
        elif isinstance(state, list):
            self.state = np.array(state)
        elif isinstance(state, tuple):
            self.state = np.array(state)
        else:
            raise AssertionError('cannot recognize input state', state)
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

    def next_moves_virtual(self, state):
        """
        all possible location next move can go
        :param state: current state of robot
        :type state: numpy.ndarray
        :return: a list of possible location of next move
        :rtype: list
        """
        states_next = []
        dises_next = []
        for x in np.arange(-1, 2):      # can only move 1 at a time
            for y in np.arange(-1, 2):  # can only move 1 at a time
                dis = math.sqrt(x**2 + y**2)
                move = np.array([x, y])
                state_copy = np.copy(state) + move
                states_next.append(state_copy)
                dises_next.append(dis)
        return states_next, dises_next


class point_robot(robot):
    def __init__(self, state=None):
        super().__init__(state)

    def copy(self): return point_robot(copy.deepcopy(self.state))


class rigid_robot(robot):
    def __init__(self, state=None, radius=0, step_resolution=1, step_min=1, step_max=10, angle_turn_min=0, angle_turn_max=math.pi/3, angel_turn_resolution=math.pi/6):
        """
        a rigid robot with radius
        :param state: robot state [x, y, angle]
        :type state:
        :param radius:
        :type radius:
        :param step_min:
        :type step_min:
        :param step_max:
        :type step_max:
        :param angel_turn_resolution: int or float
        :type angel_turn_resolution:
        """
        super().__init__(state)
        self.radius = radius
        self.step_resolution = step_resolution
        self.step_min, self.step_max = step_min, step_max
        self.angle_turn_min, self.angle_turn_max = angle_turn_min, angle_turn_max
        self.angel_turn_resolution = angel_turn_resolution  # change angle resolution from degree to radians

    def get_radius(self): return self.radius

    def copy(self): return rigid_robot(state=copy.deepcopy(self.state), radius=self.radius, step_resolution=self.step_resolution, step_min=self.step_min, step_max=self.step_max, angle_turn_min=self.angle_turn_min, angle_turn_max=self.angle_turn_max, angel_turn_resolution=self.angel_turn_resolution/math.pi*180)

    def next_moves_virtual(self, state):
        """
        all possible location next move can go
        :param state: current state of robot
        :type state: numpy.ndarray
        :return: a list of possible location of next move
        :rtype: list
        """
        state_next = []
        dises_next = []
        for turn in np.arange(-self.angle_turn_max, self.angle_turn_max+self.angel_turn_resolution, self.angel_turn_resolution):
            for step in np.arange(self.step_max, self.step_min, -self.step_resolution):
                state_copy = np.copy(state)
                state_next.append(rigid_robot.move_virtual(self, state=state_copy, step=step, theta=turn))
                dises_next.append(step)
        return state_next, dises_next

    def move(self, step, theta):
        """
        move rigid robot
        :param step: step length
        :type step: int or float
        :param theta: angle to turn
        :type theta: int or float
        :return:
        :rtype:
        """
        self.state[3] += theta  # turn robot angle first
        self.state[0:-1] = self.state[0:-1] + step * np.array([np.cos(self.state[-1]), np.sin(self.state[-1])])

    def move_virtual(self, state, step, theta):
        """
        move virtual state of a robot
        :param state: virtual state
        :type state: numpy array
        :param step: step length
        :type step: int or float
        :param theta: angle to turn
        :type theta: int or float
        :return:
        :rtype:
        """
        state[2] += theta  # turn robot angle first
        state[0:-1] = state[0:-1] + step * np.array([np.cos(state[-1]), np.sin(state[-1])])
        return state
