Prof. Keough discussed what the 8-puzzle problem, during class.  Then he told us to implement code for the 8-puzzle problem, rather, a generalized version of the same using Uniform cost search and A * Algorithm (using Heuristics: Misplaced tile, Manhattan Distance). 

What is the 8-puzzle problem? According to Princeton (COS 226), The 8-puzzle is a sliding puzzle that is played on a 3-by-3 grid with 8 square tiles labeled 1 through 8, plus a blank square. The goal is to rearrange the tiles so that they are in row-major order, using as few moves as possible. You are permitted to slide tiles either horizontally or vertically into the blank square.

How can Uniform cost search help in solving this problem? According to CUNY (CIS 32) Uniform cost search guaranteed to find cheapest solution. It is a search algorithm, that expands the node with the cheapest cost. It uses priority queue to store the elements. It finds the lowest-cost solution, but it doesn't inherently reconstruct the path. Each move in the 8-puzzle has a cost of 1, making it a uniform cost search problem. 

How can A * be used with Heuristics to solve this problem? According to UCI (ICS 271), in A* we avoid expanding paths that are expensive and focus on paths that show promise. The misplaced tile heuristic helps us calculate the number of tiles that are not in the correct position and the Manhattan distance helps us estimate the distance of each tile from its correct position.  

Note - The output is of these algorithms is analyzed based on the max queue size, solution depth, number of nodes expanded and execution time. The same example is used for all 3 algorithms.
