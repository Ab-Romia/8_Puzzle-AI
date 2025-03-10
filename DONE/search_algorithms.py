from .data_structures import Queue, Stack, Heap
from .state import State

def goalTest(board):
    return board == [[0, 1, 2], [3, 4, 5], [6, 7, 8]]

def bfs(initialState):
    frontier = Queue()
    frontier.enqueue(initialState)
    frontier_set = {tuple(map(tuple, initialState.board))}
    explored = set()
    State.nodes_expanded = 0

    while not frontier.isEmpty():
        state = frontier.dequeue()
        frontier_set.remove(tuple(map(tuple, state.board)))
        explored.add(tuple(map(tuple, state.board)))
        State.nodes_expanded += 1

        if goalTest(state.board):
            return state

        for neighbor in state.neighbors(explored,frontier_set):
            neighbor_board_tuple = tuple(map(tuple, neighbor.board))
            frontier.enqueue(neighbor)
            frontier_set.add(neighbor_board_tuple)

    return None


def dfs(initialState, depth_limit=300000, nodes_expanded=0):
    frontier = Stack()
    frontier.push((initialState, 0))  # Track state and depth
    frontier_set = {tuple(map(tuple, initialState.board))}
    explored = set()
    State.nodes_expanded = nodes_expanded  # Reset node count
    max_depth = 0
    while not frontier.isEmpty():
        state, current_depth = frontier.pop()
        state_tuple = tuple(map(tuple, state.board))

        if state_tuple in explored:
            continue  # Avoid revisiting states
        max_depth = max(max_depth, current_depth)
        explored.add(state_tuple)  # Mark explored immediately
        frontier_set.discard(state_tuple)  # Remove from frontier set
        State.nodes_expanded += 1
        # print(state.board)
        # print(state.paths)
        if goalTest(state.board):
            state.depth = max_depth
            return state  # Found solution within depth

        if depth_limit is None or current_depth < depth_limit:
            neighbors = state.neighbors(explored, frontier_set)
            for neighbor in neighbors:
                neighbor_tuple = tuple(map(tuple, neighbor.board))
                frontier.push((neighbor, current_depth + 1))  # Push with updated depth
                frontier_set.add(neighbor_tuple)

    return None  # No solution found within depth limit


def iterative_dfs(initialState):
    depth = 0
    result = None
    State.nodes_expanded = 0  # Reset counter

    while result is None:
        result = dfs(initialState, depth,State.nodes_expanded)  # Run DFS with increasing depth
        if result is not None:
            return result  # Immediately return when we find the shortest solution
        depth += 1  # Increase depth limit only if no solution was found

    return result  # Return optimal path


def a_star(initialState,heuristic, depth=None, nodes_expanded=0):
    frontier = Heap()
    initialState.heuristic = heuristic(initialState.board)
    initialState.cost = initialState.depth + initialState.heuristic
    frontier.push(initialState)
    frontier_set = {tuple(map(tuple, initialState.board))}
    explored = set()
    State.nodes_expanded = 0
    max_depth = 0

    while not frontier.isEmpty():
        state = frontier.pop()
        frontier_set.remove(tuple(map(tuple, state.board)))
        explored.add(tuple(map(tuple, state.board)))
        State.nodes_expanded += 1
        max_depth = max(max_depth, state.depth)

        if goalTest(state.board):
            state.depth = max_depth
            return state

        for neighbor in state.neighbors(explored,frontier_set):
            neighbor.heuristic = heuristic(neighbor.board)
            neighbor.cost = neighbor.depth + neighbor.heuristic
            neighbor_board_tuple = tuple(map(tuple, neighbor.board))
            frontier.push(neighbor)
            frontier_set.add(neighbor_board_tuple)

    return None

