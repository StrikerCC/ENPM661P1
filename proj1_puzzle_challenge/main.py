from search import bfs, puzzle15


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
    # inti = [[[1, 2], [0, 3]], [[3, 1], [2, 0]]]
    # sol = [[1, 2], [3, 0]]

    # testing with 3x3 puzzle
    # inti = [[[1, 2, 3], [4, 0, 5], [6, 7, 8]], [[1, 0, 2], [3, 4, 5], [6, 7, 8]]]
    # sol = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]


    for i in range(len(inti)-1, -1, -1):
        puzzle = puzzle15(inti[i], goal=sol)
        file = open('nodePath_'+ str(i+1) + '.txt', 'w')

        print('search start')
        searcher = bfs(puzzle.state, puzzle.goal)
        searcher.search()
        print('finished search')
        searcher.retrivePathToTxtFile(file)

        # for node_ in searcher.solutions:
        #     print('one solution is ')
        #
        #     print(str(node_))


if __name__ == '__main__':
    main()