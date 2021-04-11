# ENPM661P1
Project 1: puzzle challenge  

# library 
import the following package:

`import numpy as np`
numpy to store the digits in a 2d-array.  

`from search import node, bfs, puzzle15`
then from search import puzzle15, bfs.  
puzzle15 class give user UI to play with puzzle game.  
node class store state puzzle, and generate children node  
search class search goal node based on given initial node  

# Instructions to run the code
First create initial state and goal state as a 2d list.  
then build puzzle15 class with those states.  
`state_initial = [[1, 2, 3, 4], 
                [5, 6, 0, 8], 
                [9, 10, 7, 12], 
                [13, 14, 11, 15]]`  

`sol = [[1, 2, 3, 4],
       [5, 6, 7, 8],
       [9, 10, 11, 12],
       [13, 14, 15, 0]]`  

`puzzle = puzzle15(inti, goal=sol)`   

Next, create bfs object, and start search  
`searcher = bfs(puzzle.state, puzzle.goal)`  

`searcher.search()`  

Finally, retrieve path to txtFile  
`searcher.retrivePathToTxtFile(file)`  