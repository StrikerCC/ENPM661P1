class Searcher:

    def __int__(self, name="dfs"):
        self.name = name
        print('searching ready')
        print("search strategy is " + self.name)

    def find(self, initial, goal, actionSet):
        print("search strategy is " + self.name)
        print('from ', initial)
        print('available moves are ', actionSet)
        print('looking for ', goal)



