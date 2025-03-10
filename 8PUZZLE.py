import random
import time
from DONE import State, bfs, dfs, iterative_dfs, a_star, visualize_path, goalTest, euclidean_heuristic, \
    manhattan_heuristic


def measure_search_time(search_func, initial_state, search_name, heuristic=None):
    start_time = time.time()
    if heuristic is None:
        goal_state = search_func(initial_state)
    else:
        goal_state = search_func(initial_state, heuristic)
    end_time = time.time()
    if goal_state is None:
        print(f"{search_name}: The puzzle could not be solved due to data loss in the frontier")
    else:
        if goalTest(goal_state.board):
            stats = {
                'nodes_expanded': State.nodes_expanded,
                'max_depth': goal_state.depth,
                'time_taken': end_time - start_time
            }
            visualize_path(initial_state.board, goal_state.paths, search_name, stats)
        else:
            print(f"{search_name}: No path found")


def count_inversions(board):
    flat_board = [tile for row in board for tile in row if tile != 0]
    inversions = 0
    for i in range(len(flat_board)):
        for j in range(i + 1, len(flat_board)):
            if flat_board[i] > flat_board[j]:
                inversions += 1
    return inversions


def is_solvable(board):
    inversions = count_inversions(board)
    return inversions % 2 == 0


# Example usage in the main script
#initial_state = State([[1, 4, 2], [6, 5, 8], [7, 3, 0]], paths=[])
# random initial_state
# temp = [0, 1, 2, 3, 4, 5, 6, 7, 8]
# random.shuffle(temp)
# initial_state = State([temp[:3], temp[3:6], temp[6:]], paths=[])
initial_state = State([[1, 2, 3], [4, 5, 6], [7, 8, 0]], paths=[])

if is_solvable(initial_state.board):
    measure_search_time(bfs, initial_state, "BFS")
    measure_search_time(dfs, initial_state, "DFS")
    measure_search_time(iterative_dfs, initial_state, "Iterative DFS")
    measure_search_time(a_star, initial_state, "A* Euclidean Search", euclidean_heuristic)
    measure_search_time(a_star, initial_state, "A* Manhattan Search", manhattan_heuristic)

else:
    print("The puzzle is not solvable")
