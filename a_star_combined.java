import java.util.*;

public class a_star_combined {

    static class PuzzleNode implements Comparable<PuzzleNode> {
        public int[][] state;
        public double pathCost;
        public PuzzleNode parent;
        public int zeroRow, zeroCol;
        public double heuristic; // Added for A* search

        public PuzzleNode(int[][] state, double pathCost, PuzzleNode parent, int zeroRow, int zeroCol, double heuristic) { // Added heuristic
            int n = state.length;
            this.state = new int[n][n];
            for (int i = 0; i < n; i++) {
                this.state[i] = Arrays.copyOf(state[i], n);
            }
            this.pathCost = pathCost;
            this.parent = parent;
            this.zeroRow = zeroRow;
            this.zeroCol = zeroCol;
            this.heuristic = heuristic; // Initialize
        }

        @Override
        public int compareTo(PuzzleNode other) {
            // Compare based on the total estimated cost (f = g + h) for A*
            // For UCS, heuristic is 0, so it compares only based on pathCost
            return Double.compare(this.pathCost + this.heuristic, other.pathCost + other.heuristic);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            PuzzleNode that = (PuzzleNode) obj;
            return Arrays.deepEquals(state, that.state);
        }

        @Override
        public int hashCode() {
            return Arrays.deepHashCode(state);
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            for (int[] row : state) {
                for (int val : row) {
                    sb.append(val).append(" ");
                }
                sb.append("\n");
            }
            sb.append("Path Cost: ").append(pathCost).append("\n");
            sb.append("Heuristic: ").append(heuristic).append("\n");
            return sb.toString();
        }
    }

    static class PuzzleProblem {
        public int[][] initialState, goalState;
        public int n;

        public PuzzleProblem(int[][] initialState, int[][] goalState, int n) {
            this.initialState = new int[n][n];
            this.goalState = new int[n][n];
            this.n = n;
            for (int i = 0; i < n; i++) {
                this.initialState[i] = Arrays.copyOf(initialState[i], n);
                this.goalState[i] = Arrays.copyOf(goalState[i], n);
            }
        }

        // Generates possible actions (moves of the zero) from the current state
        public List<PuzzleNode> actions(PuzzleNode node, int searchType) { // Added searchType
            List<PuzzleNode> actions = new ArrayList<>();
            int[][] moves = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Possible moves: up, down, left, right

            for (int[] move : moves) {
                int newRow = node.zeroRow + move[0];
                int newCol = node.zeroCol + move[1];

                // Check if the new position is within the puzzle boundaries
                if (newRow >= 0 && newRow < n && newCol >= 0 && newCol < n) {
                    int[][] newState = new int[n][n];
                    for (int i = 0; i < n; i++) {
                        newState[i] = Arrays.copyOf(node.state[i], n);
                    }

                    // Swap the zero with the tile at the new position
                    newState[node.zeroRow][node.zeroCol] = newState[newRow][newCol];
                    newState[newRow][newCol] = 0;

                    // Calculate the heuristic for the new state
                    double heuristicValue = 0;
                    if (searchType == 1) { // A* search - Misplaced Tiles
                        heuristicValue = calculateMisplacedTiles(newState, goalState, n);
                    } else if (searchType == 2) { // A* search - Manhattan Distance
                        heuristicValue = calculateManhattanDistance(newState, goalState, n);
                    } // For UCS (searchType == 3), heuristicValue remains 0
                    // Create a new PuzzleNode for the resulting state
                    actions.add(new PuzzleNode(newState, node.pathCost + 1, node, newRow, newCol, heuristicValue)); // Pass heuristicValue
                }
            }
            return actions;
        }

        // Checks if the given state is the goal state
        public boolean goalTest(int[][] state) {
            return Arrays.deepEquals(state, goalState);
        }

        // Reconstructs the path from the initial state to the goal state
        public List<PuzzleNode> getSolutionPath(PuzzleNode node) {
            List<PuzzleNode> path = new ArrayList<>();
            while (node != null) {
                path.add(node);
                node = node.parent;
            }
            Collections.reverse(path);
            return path;
        }

        // Calculates the number of misplaced tiles in the current state compared to the goal state
        private double calculateMisplacedTiles(int[][] currentState, int[][] goalState, int n) {
            double count = 0;
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (currentState[i][j] != 0 && currentState[i][j] != goalState[i][j]) {
                        count++;
                    }
                }
            }
            return count;
        }

        // Calculates the Manhattan distance heuristic
        private double calculateManhattanDistance(int[][] currentState, int[][] goalState, int n) {
            double distance = 0;
            Map<Integer, int[]> goalPositions = new HashMap<>();
            // Store the coordinates of each tile in the goal state
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    goalPositions.put(goalState[i][j], new int[]{i, j});
                }
            }

            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if (currentState[i][j] != 0) {
                        int[] goalPos = goalPositions.get(currentState[i][j]);
                        distance += Math.abs(i - goalPos[0]) + Math.abs(j - goalPos[1]);
                    }
                }
            }
            return distance;
        }
    }

    public PuzzleNode aStarSearch(PuzzleProblem problem, int searchType) { // Changed from aStarSearch
        int n = problem.n;
        int initialZeroRow = -1, initialZeroCol = -1;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (problem.initialState[i][j] == 0) {
                    initialZeroRow = i;
                    initialZeroCol = j;
                    break;
                }
            }
        }

        // Calculate the initial heuristic value
        double initialHeuristic = 0;
        if (searchType == 1) {
            initialHeuristic = problem.calculateMisplacedTiles(problem.initialState, problem.goalState, n);
        } else if (searchType == 2) {
            initialHeuristic = problem.calculateManhattanDistance(problem.initialState, problem.goalState, n);
        } // For UCS, initialHeuristic is 0

        // Create the initial node
        PuzzleNode initialNode = new PuzzleNode(problem.initialState, 0, null, initialZeroRow, initialZeroCol, initialHeuristic); // Pass initialHeuristic
        // Priority queue to store nodes to be explored, prioritized by f(n) = g(n) + h(n)
        PriorityQueue<PuzzleNode> frontier = new PriorityQueue<>();
        // Set to keep track of explored states to avoid redundant computations
        Set<String> explored = new HashSet<>();

        frontier.add(initialNode);
        int steps = 0;
        int maxQueueSize = 0;

        while (!frontier.isEmpty()) {
            // Get the node with the lowest f(n) from the frontier
            PuzzleNode node = frontier.poll();
            steps++;

            // Check if the current node is the goal state
            if (problem.goalTest(node.state)) {
                System.out.println("Steps taken: " + steps);
                System.out.println("Max queue size: " + maxQueueSize);
                return node;
            }

            // Add the current state to the explored set
            String stateStr = Arrays.deepToString(node.state);
            explored.add(stateStr);

            // Generate successor nodes
            for (PuzzleNode child : problem.actions(node, searchType)) { // Pass searchType
                String childStr = Arrays.deepToString(child.state);
                // If the child state has not been explored and is not in the frontier
                if (!explored.contains(childStr) && !containsState(frontier, childStr)) {
                    frontier.add(child);
                }
                // If the child state is in the frontier, check if the new path is better
                else if (containsState(frontier, childStr)) {
                    PuzzleNode existing = getNode(frontier, childStr);
                    if (existing != null && (existing.pathCost + existing.heuristic) > (child.pathCost + child.heuristic)) {
                        frontier.remove(existing);
                        frontier.add(child);
                    }
                }
            }
            // Update the maximum size the queue reached
            maxQueueSize = Math.max(maxQueueSize, frontier.size());

            if (steps % 1000 == 0) {
                System.out.println("Checked " + steps + " nodes...");
            }
        }

        System.out.println("Max queue size: " + maxQueueSize);
        return null; // Goal not found
    }

    // Helper function to check if a state exists in the priority queue
    private boolean containsState(PriorityQueue<PuzzleNode> frontier, String stateStr) {
        for (PuzzleNode node : frontier) {
            if (Arrays.deepToString(node.state).equals(stateStr)) {
                return true;
            }
        }
        return false;
    }

    // Helper function to get a node with a specific state from the priority queue
    private PuzzleNode getNode(PriorityQueue<PuzzleNode> frontier, String stateStr) {
        for (PuzzleNode node : frontier) {
            if (Arrays.deepToString(node.state).equals(stateStr)) {
                return node;
            }
        }
        return null;
    }

    // Checks if a given n-puzzle state is solvable
    public static boolean isSolvable(int[][] state, int n) {
        int[] flat = new int[n * n];
        int idx = 0;
        for (int[] row : state) {
            for (int val : row) flat[idx++] = val;
        }

        int inversions = 0;
        for (int i = 0; i < flat.length; i++) {
            for (int j = i + 1; j < flat.length; j++) {
                if (flat[i] != 0 && flat[j] != 0 && flat[i] > flat[j]) inversions++;
            }
        }

        if (n % 2 == 1) {
            return inversions % 2 == 0;
        } else {
            int zeroRow = -1;
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    if (state[i][j] == 0) zeroRow = i;
            return (inversions + zeroRow) % 2 == 1;
        }
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        System.out.print("Enter puzzle size n (for n x n puzzle): ");
        int n = scanner.nextInt();

        int[][] initialState = new int[n][n];
        int[][] goalState = new int[n][n];

        System.out.println("Enter initial state (row-wise, use 0 for blank):");
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                initialState[i][j] = scanner.nextInt();

        System.out.println("Enter goal state (row-wise, use 0 for blank):");
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                goalState[i][j] = scanner.nextInt();

        // Check if the initial puzzle is solvable
        if (!isSolvable(initialState, n)) {
            System.out.println("This puzzle is unsolvable.");
            return;
        }

        // Create the puzzle problem and the A* solver
        PuzzleProblem problem = new PuzzleProblem(initialState, goalState, n);
        a_star_combined solver = new a_star_combined();

        System.out.println("Choose a search algorithm:");
        System.out.println("1. A* Search (Misplaced Tiles)");
        System.out.println("2. A* Search (Manhattan Distance)");
        System.out.println("3. Uniform Cost Search");
        System.out.print("Enter your choice (1, 2, or 3): ");
        int searchChoice = scanner.nextInt();

        if (searchChoice < 1 || searchChoice > 3) {
            System.out.println("Invalid search choice. Exiting.");
            scanner.close();
            return;
        }

        long start = System.currentTimeMillis();
        PuzzleNode goalNode = null;
        if (searchChoice == 3) {
            goalNode = solver.aStarSearch(problem, 0); // UCS
        } else {
            goalNode = solver.aStarSearch(problem, searchChoice); // A*
        }
        long end = System.currentTimeMillis();

        if (goalNode != null) {
            // Reconstruct and print the solution path
            List<PuzzleNode> path = problem.getSolutionPath(goalNode);
            System.out.println("\nSolution Path:");
            for (PuzzleNode node : path) {
                System.out.println(node);
            }
            int solutionDepth = path.size() - 1;
            System.out.println("Solution depth was: " + solutionDepth);
            System.out.println("Number of moves: " + (path.size() - 1));
            System.out.println("Execution Time: " + (end - start) + " ms");
        } else {
            System.out.println("Goal not found. Execution Time: " + (end - start) + " ms");
        }
        scanner.close();
    }
}

