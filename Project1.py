class PuzzleNode:
    def __init__(self, state, depth, f_value, move_cost=0):
        """ Initialize the node with the puzzle state, depth, f_value, and move cost."""
        self.state = state
        self.depth = depth
        self.f_value = f_value
        self.move_cost = move_cost

    def generate_children(self, tile_sequence):
        """ Generate child nodes by moving specified tiles."""
        """REQUIREMENT 2"""
        children = []
        for tile in tile_sequence:
            x, y = self.find_tile(self.state, tile)
            # Possible moves with corresponding costs
            potential_moves = [[x, y - 1, 2], [x, y + 1, 2], [x - 1, y, 1], [x + 1, y, 1]] #REQUIREMENT 4
            for move in potential_moves:
                new_state = self.swap_tiles(self.state, x, y, move[0], move[1])
                if new_state is not None:
                    move_cost = move[2]
                    child_node = PuzzleNode(new_state, self.depth + 1, 0, move_cost)
                    children.append(child_node)
        return children

    def swap_tiles(self, puzzle, x1, y1, x2, y2):
        """ Swap tiles and return the new puzzle state."""
        """REQUIREMENT 2"""
        if 0 <= x2 < len(self.state) and 0 <= y2 < len(self.state):
            new_puzzle = self.copy_state(puzzle)
            new_puzzle[x1][y1], new_puzzle[x2][y2] = new_puzzle[x2][y2], new_puzzle[x1][y1]
            return new_puzzle
        return None

    def copy_state(self, state):
        """ Create a copy of the given puzzle state."""
        return [row[:] for row in state]

    def find_tile(self, puzzle, tile):
        """ Find the position of a specific tile in the puzzle."""
        for i, row in enumerate(puzzle):
            for j, value in enumerate(row):
                if value == tile:
                    return i, j


class SlidingPuzzleSolver:
    def __init__(self, size):
        """ Initialize puzzle solver with size and tracking lists."""
        self.size = size
        self.open_list = []
        self.closed_set = set()

    def convert_to_tuple(self, matrix):
        """ Convert a 2D list into a tuple representation."""
        return tuple(tuple(row) for row in matrix)

    def validate_input(self, puzzle):
        """ Validate the puzzle input to ensure only allowed characters are present."""
        allowed_tiles = {'1', '2', '3', '_'}
        for row in puzzle:
            for tile in row:
                if tile not in allowed_tiles:
                    print(f"Invalid tile '{tile}' detected. Allowed tiles are: {', '.join(allowed_tiles)}")
                    return False
        return True

    def get_puzzle_input(self):
        """ Accept the puzzle state from the user."""
        """REQUIREMENT 1"""
        while True:
            puzzle = []
            print(f"Enter the {self.size}x{self.size} matrix row by row: (Write '_' for empty tiles)")
            for _ in range(self.size):
                row = input().split(" ")
                puzzle.append(row)
            if self.validate_input(puzzle):
                return puzzle

    def f_function(self, current_node, goal_state): #REQUIREMENT 5
        """ Heuristic function f(x) = h(x) + g(x)."""
        return self.manhattan_distance(current_node.state, goal_state) + current_node.depth

    def manhattan_distance(self, current_state, goal_state): #REQUIREMENT 5
        """ Calculate Manhattan Distance as the heuristic."""
        distance = 0
        for i in range(self.size):
            for j in range(self.size):
                tile = current_state[i][j]
                if tile != '_' and tile != goal_state[i][j]:
                    goal_x, goal_y = self.find_tile(goal_state, tile)
                    distance += abs(i - goal_x) + abs(j - goal_y)
        return distance

    def find_tile(self, puzzle, tile):
        """ Find the position of a tile in the puzzle."""
        for i, row in enumerate(puzzle):
            for j, value in enumerate(row):
                if value == tile:
                    return i, j

    def solve(self):
        """ Solve the puzzle using the A* algorithm."""
        print("Enter the start state matrix:")
        start_state = self.get_puzzle_input()
        print("Enter the goal state matrix:")
        goal_state = self.get_puzzle_input()
        tile_sequence = ['1', '2', '3'] #REQUIREMENT 3

        start_node = PuzzleNode(start_state, 0, 0)
        start_node.f_value = self.f_function(start_node, goal_state)
        self.open_list.append(start_node)

        print("\nProcessing...\n")
        expanded_nodes = 0
        goal_tuple = self.convert_to_tuple(goal_state)
        max_expansions = 10  # REQUIREMENT 6

        while self.open_list:
            current_node = self.open_list.pop(0)
            current_tuple = self.convert_to_tuple(current_node.state)

            if current_tuple in self.closed_set:
                continue

            expanded_nodes += 1
            print(f"Expanded Node {expanded_nodes}:")
            for row in current_node.state:
                print(" ".join(row))
            print()

            if current_tuple == goal_tuple:
                print("Goal state reached!")
                return

            if expanded_nodes >= max_expansions:
                print(f"Expansion limit of {max_expansions} reached. Ending search.")
                return

            self.closed_set.add(current_tuple)

            children = current_node.generate_children(tile_sequence) #REQUIREMENT 3
            for child in children:
                child.f_value = self.f_function(child, goal_state) + child.move_cost
                child_tuple = self.convert_to_tuple(child.state)

                if child_tuple not in self.closed_set and all(self.convert_to_tuple(open_node.state) != child_tuple for open_node in self.open_list):
                    self.open_list.append(child)

            self.open_list.sort(key=lambda x: x.f_value)

        print("No solution found!")


puzzle_solver = SlidingPuzzleSolver(3)
puzzle_solver.solve()
