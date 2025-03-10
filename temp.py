from collections import deque
import time
import random
import heapq
import sys

sys.setrecursionlimit(10000)


def get_neighbors(state):
    neighbors = []
    zero_index = state.index(0)
    x, y = zero_index // 3, zero_index % 3

    # Possible moves (dx, dy)
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right

    for dx, dy in moves:
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < 3 and 0 <= new_y < 3:
            # Swap blank with the tile at (new_x, new_y)
            new_index = new_x * 3 + new_y
            new_state = list(state)
            new_state[zero_index], new_state[new_index] = new_state[new_index], new_state[zero_index]
            neighbors.append(tuple(new_state))

    return neighbors


def reconstruct_path(parent, goal_state):
    path = []
    current = goal_state
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path


def bfs(start_state, goal_state):
    if start_state == goal_state:
        return [start_state], 0, 0

    queue = deque([(start_state, 0)])  # (state, depth)
    visited = {start_state}
    parent = {start_state: None}  # To reconstruct path
    nodes_expanded = 0

    while queue:
        current, depth = queue.popleft()
        nodes_expanded += 1

        if current == goal_state:
            return reconstruct_path(parent, current), nodes_expanded, depth

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                queue.append((neighbor, depth + 1))

    return None, nodes_expanded, 0  # No solution found


def dfs(start_state, goal_state, max_depth=50):
    if start_state == goal_state:
        return [start_state], 0, 0

    stack = [(start_state, 0)]  # (state, depth)
    visited = {start_state}
    parent = {start_state: None}  # To reconstruct path
    nodes_expanded = 0

    while stack:
        current, depth = stack.pop()
        nodes_expanded += 1

        if current == goal_state:
            return reconstruct_path(parent, current), nodes_expanded, depth

        if depth < max_depth:
            for neighbor in get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    parent[neighbor] = current
                    stack.append((neighbor, depth + 1))

    return None, nodes_expanded, 0  # No solution found


def iddfs(start_state, goal_state):
    # This counter will count node expansions across each depth-limited search.
    nodes_expanded = 0

    def dls(state, goal_state, depth, path):
        nonlocal nodes_expanded
        nodes_expanded += 1
        # Check if we've reached the goal.
        if state == goal_state:
            return path + [state]
        # If no depth remains, cut off the search.
        if depth == 0:
            return None
        # For each neighbor, do a recursive depth-limited search.
        for neighbor in get_neighbors(state):
            # Cycle check: don't revisit a state in the current path.
            if neighbor not in path:
                result = dls(neighbor, goal_state, depth - 1, path + [state])
                if result is not None:
                    return result
        return None

    depth_limit = 0
    while True:
        result = dls(start_state, goal_state, depth_limit, [])
        if result is not None:
            # The solution depth is the number of moves: path length minus one.
            return result, nodes_expanded, len(result) - 1
        depth_limit += 1


def a_star(start_state, goal_state, heuristic):
    def heuristic_cost(state):
        return heuristic(state, goal_state)

    open_set = []
    heapq.heappush(open_set, (0 + heuristic_cost(start_state), 0, start_state))
    parent = {start_state: None}
    g_cost = {start_state: 0}
    nodes_expanded = 0

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        nodes_expanded += 1

        if current == goal_state:
            return reconstruct_path(parent, current), nodes_expanded, cost

        for neighbor in get_neighbors(current):
            tentative_g_cost = g_cost[current] + 1
            if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                g_cost[neighbor] = tentative_g_cost
                f_cost = tentative_g_cost + heuristic_cost(neighbor)
                heapq.heappush(open_set, (f_cost, tentative_g_cost, neighbor))
                parent[neighbor] = current

    return None, nodes_expanded, 0  # No solution found


def manhattan_distance(state, goal_state):
    distance = 0
    for i in range(1, 9):
        x1, y1 = state.index(i) // 3, state.index(i) % 3
        x2, y2 = goal_state.index(i) // 3, goal_state.index(i) % 3
        distance += abs(x1 - x2) + abs(y1 - y2)
    return distance


def euclidean_distance(state, goal_state):
    distance = 0
    for i in range(1, 9):
        x1, y1 = state.index(i) // 3, state.index(i) % 3
        x2, y2 = goal_state.index(i) // 3, goal_state.index(i) % 3
        distance += ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    return distance


def generate_random_start_state(goal_state):
    start_state = list(goal_state)
    random.shuffle(start_state)
    return tuple(start_state)


def is_solvable(state):
    inversions = 0
    state = [tile for tile in state if tile != 0]  # Remove the blank tile
    for i in range(len(state)):
        for j in range(i + 1, len(state)):
            if state[i] > state[j]:
                inversions += 1
    return inversions % 2 == 0


def format_state(state):
    return "\n".join([" ".join(map(str, state[i:i + 3])) for i in range(0, 9, 3)])


goal_state = (0, 1, 2, 3, 4, 5, 6, 7, 8)


def main():
    start_state = (0, 8, 7, 4, 5, 6, 1, 2, 3)
    # generate_random_start_state(goal_state)
    # while not is_solvable(start_state):
    #    start_state = generate_random_start_state(goal_state)
    # print(f"Start State:\n{format_state(start_state)}\n")

    # BFS
    start_time = time.time()
    path_bfs, nodes_expanded_bfs, depth_bfs = bfs(start_state, goal_state)
    bfs_time = time.time() - start_time

    # DFS
    start_time = time.time()
    path_dfs, nodes_expanded_dfs, depth_dfs = dfs(start_state, goal_state)
    dfs_time = time.time() - start_time

    # IDDFS
    start_time = time.time()
    path_iddfs, nodes_expanded_iddfs, depth_iddfs = iddfs(start_state, goal_state)
    iddfs_time = time.time() - start_time

    # A* Manhattan
    start_time = time.time()
    path_astar_man, nodes_expanded_astar_man, depth_astar_man = a_star(start_state, goal_state, manhattan_distance)
    astar_man_time = time.time() - start_time

    # A* Euclidean
    start_time = time.time()
    path_astar_euc, nodes_expanded_astar_euc, depth_astar_euc = a_star(start_state, goal_state, euclidean_distance)
    astar_euc_time = time.time() - start_time

    # Print or log the results
    def print_results(name, path, nodes_expanded, depth, time_taken):
        if path is None:
            print(f"{name}: No solution found")
        else:
            print(f"{name}:")
            print(f"  Cost: {len(path) - 1}")
            print(f"  Nodes Expanded: {nodes_expanded}")
            print(f"  Search Depth: {depth}")
            print(f"  Running Time: {time_taken:.4f} seconds")
            print(f"  Path:")
            for state in path:
                print(format_state(state))
                print()

    print_results("BFS", path_bfs, nodes_expanded_bfs, depth_bfs, bfs_time)
    print_results("DFS", path_dfs, nodes_expanded_dfs, depth_dfs, dfs_time)
    print_results("IDDFS", path_iddfs, nodes_expanded_iddfs, depth_iddfs, iddfs_time)
    print_results("A* Manhattan", path_astar_man, nodes_expanded_astar_man, depth_astar_man, astar_man_time)
    print_results("A* Euclidean", path_astar_euc, nodes_expanded_astar_euc, depth_astar_euc, astar_euc_time)


if __name__ == "__main__":
    main()
