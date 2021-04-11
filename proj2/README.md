# ENPM661P2
BFS for point robot

# library 
To import required library, first import numpy to store the digits in a 2d-array.  
then from search import puzzle15, bfs.  
puzzle15 class give user UI to play with puzzle game.  
node class store state puzzle, and generate children node  
search class search goal node based on given initial node  
`from search import bfs, node, graph`  

# Instructions to run the code
First create initial state and goal state as a length 2 tuple.  
then build search tree class with those states.  
Input ``
`x coordinate of start = `   ``
`y coordinate of start = `   ``
`x coordinate of goal = `  ``
`y coordinate of goal = `  ``

``


Next, create bfs object, and start search  
`searcher = bfs(puzzle.state, puzzle.goal)`   ``

`searcher.search()`  ``

Finally, retrieve path to txtFile  
`searcher.retrivePathToTxtFile(file)`  