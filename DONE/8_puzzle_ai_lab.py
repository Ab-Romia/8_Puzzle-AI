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
    visited = set([start_state])
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
    visited = set([start_state])
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
    def dls(state, depth, parent, nodes_expanded):
        if state == goal_state:
            return [state], nodes_expanded, depth
        if depth == 0:
            return None, nodes_expanded, 0
        for neighbor in get_neighbors(state):
            if neighbor not in parent:
                parent[neighbor] = state
                path, nodes, d = dls(neighbor, depth - 1, parent, nodes_expanded + 1)
                if path:
                    return [state] + path, nodes, d
        return None, nodes_expanded, 0
    
    depth = 0
    while True:
        parent = {start_state: None}
        path, nodes_expanded, d = dls(start_state, depth, parent, 0)
        if path:
            return path, nodes_expanded, d
        depth += 1

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
    return "\n".join([" ".join(map(str, state[i:i+3])) for i in range(0, 9, 3)])

goal_state = (0, 8, 7, 4, 5, 6, 1, 2, 3)
def main():
    start_state = (1,0,2,7,5,4,8,6,4)
    #generate_random_start_state(goal_state)
    #while not is_solvable(start_state):
    #    start_state = generate_random_start_state(goal_state)
    #print(f"Start State:\n{format_state(start_state)}\n")
    
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