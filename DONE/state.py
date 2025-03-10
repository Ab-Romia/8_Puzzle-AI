class State:
    nodes_expanded = 0

    def __init__(self, board, paths=None, cost=None, depth=0, heuristic=0):
        self.board = board
        self.paths = paths if paths is not None else []
        self.cost = cost if cost is not None else 0
        self.depth = depth
        self.heuristic = heuristic

    def find_empty_tile(self):
        for i in range(3):
            for j in range(3):
                if self.board[i][j] == 0:
                    return i, j

    def neighbors(self, explored, frontier):
        neighbors = []
        x, y = self.find_empty_tile()
        directions = [(-1, 0, 'Up'), (1, 0, 'Down'), (0, -1, 'Left'), (0, 1, 'Right')]

        for dx, dy, direction in directions:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < 3 and 0 <= new_y < 3:
                new_board = [row[:] for row in self.board]
                new_board[x][y], new_board[new_x][new_y] = new_board[new_x][new_y], new_board[x][y]
                new_state = State(new_board, self.paths + [direction], depth=self.depth + 1)
                new_state_tuple = tuple(map(tuple, new_board))
                if new_state_tuple not in explored and new_state_tuple not in frontier:
                    neighbors.append(new_state)

        return neighbors