from search import Searcher

def main():
    print('game start')
    dfs = Searcher()
    dfs.__int__('dfs')
    dfs.find([], [], [])


if __name__ == '__main__':
    main()