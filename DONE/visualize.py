from graphviz import Digraph

def visualize_path(initial_state, path, search_name, stats):
    if len(path) > 60:
        print(f"{search_name}:")
        print("Initial State:", initial_state)
        for p in path:
            print(p[0],end='')
        print()
        print("Cost of path:", len(path))
        print("Nodes expanded:", stats['nodes_expanded'])
        print("Max depth:", stats['max_depth'])
        print("Time taken:", stats['time_taken'], "seconds")
        print("Path is too long to visualize.")
        return

    dot = Digraph(comment=f'{search_name} Path')
    node_counter = 0

    def add_node(state, parent=None, direction=None):
        nonlocal node_counter
        node_id = f'node{node_counter}'
        node_counter += 1
        label = '\n'.join([' '.join(map(str, row)) for row in state])
        dot.node(node_id, label)
        if parent:
            dot.edge(parent, node_id, label=direction)
        return node_id

    def apply_move(state, direction):
        for i in range(3):
            for j in range(3):
                if state[i][j] == 0:
                    x, y = i, j
                    break

        dx, dy = {
            'Up': (-1, 0),
            'Down': (1, 0),
            'Left': (0, -1),
            'Right': (0, 1)
        }[direction]

        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < 3 and 0 <= new_y < 3:
            new_state = [row[:] for row in state]
            new_state[x][y], new_state[new_x][new_y] = new_state[new_x][new_y], new_state[x][y]
            return new_state
        else:
            return state

    current_state = initial_state
    parent_id = add_node(current_state)
    for move in path:
        current_state = apply_move(current_state, move)
        parent_id = add_node(current_state, parent_id, move)

    # Add stats to the graph
    stats_label = f"{search_name}:\nPath: {''.join([move[0] for move in path])}\nCost of path: {len(path)}\nNodes expanded: {stats['nodes_expanded']}\nMax depth: {stats['max_depth']}\nTime taken: {stats['time_taken']} seconds"
    dot.attr(label=stats_label, labelloc="t", fontsize="12")

    dot.render(f'{search_name}_path', view=True)

    print(f"{search_name}:")
    for move in path:
        print(move[0], end='')
    print()
    print("Cost of path:", len(path))
    print("Nodes expanded:", stats['nodes_expanded'])
    print("Max depth:", stats['max_depth'])
    print("Time taken:", stats['time_taken'], "seconds")