from search import node, dfs, puzzle15


def main():
    inti = [[[1, 2, 3, 4], [5, 6, 0, 8], [9, 10, 7, 12], [13, 14, 11, 15]],
            [[1, 0, 3, 4], [5, 2, 7, 8], [9, 6, 10, 11], [13, 14, 15, 12]],
            [[0, 2, 3, 4], [1, 5, 7, 8], [9, 6, 11, 12], [13, 10, 14, 15]],
            [[5, 1, 2, 3], [0, 6, 7, 4], [9, 10, 11, 8], [13, 14, 15, 12]],
            [[1, 6, 2, 3], [9, 5, 7, 4], [0, 10, 11, 8], [13, 14, 15, 12]]]

    sol = [[1, 2, 3, 4],
           [5, 6, 7, 8],
           [9, 10, 11, 12],
           [13, 14, 15, 0]]

    # testing with 2x2 puzzle
    inti = [[[1, 2, 3], [4, 0, 5], [6, 7, 8]]]
    inti = [[[1, 0, 2], [3, 4, 5], [6, 7, 8]]]
    sol = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]

    puzzle = puzzle15(inti[0], goal=sol)
    node1 = node(puzzle.state)

    children = node1.children()
    # for child in children:
    #     print(str(child))

    print('game start')
    searcher = dfs(puzzle.state, puzzle.goal)
    searcher.search()
    # print(searcher)
    for sol in searcher.solutions:
        print('one solution is ')
        for node_ in sol:
            print(str(node_))


if __name__ == '__main__':
    main()