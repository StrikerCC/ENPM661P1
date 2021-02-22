import numpy as np
from collections import deque

class node:

    def __init__(self, state, parent=None):
        self.state = np.copy(state)
        self.parent = parent


    # return a list of child
    def children(self):
        nodes = []
        for move in [up, left, down, right]:
            node_next = node(self.state, parent=self)
            if move(node_next): nodes.append(node_next)

        assert type(nodes[0]) == node
        return nodes

    def blankTileLocation(self):
        return np.where(self.state == 0)

    # to string col-wise
    def toColWiseString(self):
        state = np.transpose(self.state).flatten()
        return str(state)[1:-1]

    # to integer
    def __int__(self):
        state = np.transpose(self.state).flatten()
        return int(sum(d * 10 ** i for i, d in enumerate(state[::-1])))

    # to string
    def __str__(self):
        return str(self.state)[1:-1]

    # to int for hashing
    def __hash__(self):
        return hash(tuple(map(tuple, self.state)))


# searching algorithm object
class bfs:
    def __init__(self, state_init, goal):
        self.queue_node = deque([node(state_init)])
        self.visited = set()
        self.visited.add(int(self.queue_node[0]))
        self.goal = node(goal)
        self.goal_hash = int(self.goal)
        self.solutions = []
        print('searching ready')

    def search(self):

        i = 0
        # as long as stack is not empty, keep dfs
        # for _ in range(5):
        while self.queue_node:
            if i%100000 == 0:
                print('loop', i)
            i += 1

            # for node in self.stack:
            #     print(int(node))

            node_cur = self.queue_node.popleft()     # take a node from stack
            children = node_cur.children()  # children expanded from this node

            child_hash = 0
            for child in children:
                child_hash = int(child)
                if child_hash == self.goal_hash:  # reach a goal
                    print('find a solution')
                    self.solutions.append(child)   # retrieve ancestors from goal node
                    # return
                else:  # this child is not a goal
                    if child_hash not in self.visited:  # this child represent a new state never seen before
                        self.visited.add(child_hash)
                        self.queue_node.append(child)
                    else:   # this child is repeative
                        del child

    # return parents of this node as a list
    def retrivePathToTxtFile(self, file):
        if not self.solutions: file.write('no solution in this case\n')
        for i in range(len(self.solutions)):
            path, node_cur = [], self.solutions[i]
            while node_cur is not None:  # keep going until root
                path.append(node_cur)  # follow the convention to output matrix column wise
                parent = node_cur.parent
                node_cur = parent
            path.reverse()

            file.write(str(i+1) + " th solution is\n")
            for node_ in path:  # keep going until root
                nodeColWiseString = node_.toColWiseString()
                file.write(nodeColWiseString+'\n')  # follow the convention to output matrix column wise
        return


# puzzle class, represents the state of puzzle, could perform quzzle action
class puzzle15:

    # default 4x4 puzzle goal
    goal_defalut = np.array([[1, 2, 3, 4],
                              [5, 6, 7, 8],
                              [9, 10, 11, 12],
                              [13, 14, 15, 0]])

    def __init__(self, nums, goal=goal_defalut):
        if type(nums) == list: self.state = np.array(nums)
        elif type(nums) == np.ndarray: self.state = nums
        else: AssertionError('puzzle takes ', nums)

        if type(goal) == list: self.goal = np.array(goal)
        elif type(goal) == np.ndarray: self.goal = goal
        else: AssertionError('puzzle takes ', goal)

        # self.actions = [self.moveUp, self.moveDown, self.moveLeft, self.moveRight]

    # query of blank tile location
    def blankTileLocation(self):
        return np.where(self.state == 0)

    def movesAround(self):
        return [[self.moveUp, self.moveDown],
                [self.moveDown, self.moveUp],
                [self.moveLeft, self.moveRight],
                [self.moveRight, self.moveLeft]]

    # move blank tile one step up
    def moveUp(self):
        i, j = self.blankTileLocation()
        if i <= 0:
            return
        else:
            self.state[i, j] += self.state[i - 1, j]
            self.state[i - 1, j] = 0

    # move blank tile one step down
    def moveDown(self):
        i, j = self.blankTileLocation()
        if i >= len(self.state) - 1:
            return
        else:
            self.state[i, j] += self.state[i + 1, j]
            self.state[i + 1, j] = 0

    # move blank tile one step left
    def moveLeft(self):
        i, j = self.blankTileLocation()
        if j <= 0:
            return
        else:
            self.state[i, j] += self.state[i, j - 1]
            self.state[i, j - 1] = 0

    # move blank tile one step right
    def moveRight(self):
        i, j = self.blankTileLocation()
        if j >= len(self.state) - 1:
            return
        else:
            self.state[i, j] += self.state[i, j + 1]
            self.state[i, j + 1] = 0

    # if current state reach the goal
    def reachGoal(self):
        return np.alltrue(self.state == self.goal)

    # override tostring method
    def __str__(self):
        return str(self.state)

# move blank tile one step up
def up(self):
    i, j = self.blankTileLocation()
    if i <= 0:
        return False
    else:
        self.state[i, j] += self.state[i - 1, j]
        self.state[i - 1, j] = 0
        return True

# move blank tile one step down
def down(self):
    i, j = self.blankTileLocation()
    if i >= len(self.state) - 1:
        return False
    else:
        self.state[i, j] += self.state[i + 1, j]
        self.state[i + 1, j] = 0
        return True

# move blank tile one step left
def left(self):
    i, j = self.blankTileLocation()
    if j <= 0:
        return False
    else:
        self.state[i, j] += self.state[i, j - 1]
        self.state[i, j - 1] = 0
        return True

# move blank tile one step right
def right(self):
    i, j = self.blankTileLocation()
    if j >= len(self.state) - 1:
        return False
    else:
        self.state[i, j] += self.state[i, j + 1]
        self.state[i, j + 1] = 0
        return True


