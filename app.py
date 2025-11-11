import gradio as gr
import random
import time
from DONE.state import State
from DONE.search_algorithms import bfs, dfs, iterative_dfs, a_star, goalTest
from DONE.heuristics import euclidean_heuristic, manhattan_heuristic
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def count_inversions(board):
    """Count the number of inversions in the puzzle."""
    flat_board = [tile for row in board for tile in row if tile != 0]
    inversions = 0
    for i in range(len(flat_board)):
        for j in range(i + 1, len(flat_board)):
            if flat_board[i] > flat_board[j]:
                inversions += 1
    return inversions


def is_solvable(board):
    """Check if the puzzle is solvable."""
    inversions = count_inversions(board)
    return inversions % 2 == 0


def create_puzzle_image(board, highlight_empty=True):
    """Create a visual representation of the puzzle board."""
    cell_size = 120
    padding = 10
    img_size = cell_size * 3 + padding * 4

    # Create image with gradient background
    img = Image.new('RGB', (img_size, img_size), '#1e293b')
    draw = ImageDraw.Draw(img)

    # Try to load a font, fall back to default if not available
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 48)
    except:
        font = ImageFont.load_default()

    for i in range(3):
        for j in range(3):
            x = j * (cell_size + padding) + padding
            y = i * (cell_size + padding) + padding

            value = board[i][j]

            if value == 0:
                # Empty tile
                if highlight_empty:
                    color = '#475569'
                else:
                    color = '#334155'
            else:
                # Number tile with gradient effect
                color = '#3b82f6'  # Blue

            # Draw rounded rectangle
            draw.rounded_rectangle(
                [x, y, x + cell_size, y + cell_size],
                radius=15,
                fill=color,
                outline='#64748b',
                width=2
            )

            # Draw number
            if value != 0:
                # Get text bounding box for centering
                bbox = draw.textbbox((0, 0), str(value), font=font)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]

                text_x = x + (cell_size - text_width) // 2
                text_y = y + (cell_size - text_height) // 2

                draw.text((text_x, text_y), str(value), fill='white', font=font)

    return img


def generate_random_puzzle():
    """Generate a random solvable puzzle."""
    while True:
        tiles = list(range(9))
        random.shuffle(tiles)
        board = [tiles[i:i+3] for i in range(0, 9, 3)]
        if is_solvable(board):
            return board


def solve_puzzle(board_str, algorithm):
    """Solve the puzzle using the selected algorithm."""
    try:
        # Parse the board
        board_str = board_str.strip()
        if not board_str:
            return None, "Please enter a valid puzzle configuration.", None, None, None, None

        # Convert string to board
        tiles = [int(x) for x in board_str.replace(',', ' ').split()]
        if len(tiles) != 9:
            return None, "Please enter exactly 9 numbers (0-8).", None, None, None, None

        if sorted(tiles) != list(range(9)):
            return None, "Please use numbers 0-8 exactly once.", None, None, None, None

        board = [tiles[i:i+3] for i in range(0, 9, 3)]

        # Check if solvable
        if not is_solvable(board):
            initial_img = create_puzzle_image(board)
            return initial_img, "❌ This puzzle is not solvable! The number of inversions is odd.", None, None, None, None

        # Create initial state image
        initial_img = create_puzzle_image(board)

        # Select algorithm
        initial_state = State(board, paths=[])

        start_time = time.time()

        if algorithm == "BFS":
            result = bfs(initial_state)
        elif algorithm == "DFS":
            result = dfs(initial_state)
        elif algorithm == "Iterative DFS":
            result = iterative_dfs(initial_state)
        elif algorithm == "A* (Manhattan)":
            result = a_star(initial_state, manhattan_heuristic)
        elif algorithm == "A* (Euclidean)":
            result = a_star(initial_state, euclidean_heuristic)
        else:
            return initial_img, "Invalid algorithm selected.", None, None, None, None

        end_time = time.time()

        if result is None:
            return initial_img, "❌ Could not find a solution (may have exceeded search limits).", None, None, None, None

        # Generate solution images
        solution_images = [initial_img]
        current_board = [row[:] for row in board]

        for move in result.paths:
            # Apply move
            x, y = None, None
            for i in range(3):
                for j in range(3):
                    if current_board[i][j] == 0:
                        x, y = i, j
                        break
                if x is not None:
                    break

            move_map = {'Up': (-1, 0), 'Down': (1, 0), 'Left': (0, -1), 'Right': (0, 1)}
            dx, dy = move_map[move]
            new_x, new_y = x + dx, y + dy

            if 0 <= new_x < 3 and 0 <= new_y < 3:
                current_board[x][y], current_board[new_x][new_y] = current_board[new_x][new_y], current_board[x][y]
                solution_images.append(create_puzzle_image(current_board))

        # Create statistics
        stats = f"""
### 🎯 Solution Found!

**Algorithm:** {algorithm}
**Path Length:** {len(result.paths)} moves
**Nodes Expanded:** {State.nodes_expanded}
**Max Depth Reached:** {result.depth}
**Time Taken:** {end_time - start_time:.4f} seconds
**Moves:** {' → '.join([m[0] for m in result.paths])}

### 📊 Algorithm Info:
"""

        if algorithm == "BFS":
            stats += "- Guarantees shortest path\n- Explores level by level\n- Complete and optimal"
        elif algorithm == "DFS":
            stats += "- May not find shortest path\n- Explores depth-first\n- Memory efficient"
        elif algorithm == "Iterative DFS":
            stats += "- Combines DFS and BFS benefits\n- Finds shortest path\n- More memory efficient than BFS"
        elif "A*" in algorithm:
            stats += "- Uses heuristic to guide search\n- Guarantees shortest path\n- Very efficient for this problem"

        # Create final image
        final_img = solution_images[-1] if solution_images else initial_img

        return initial_img, stats, final_img, solution_images, len(result.paths), ' → '.join([m[0] for m in result.paths])

    except Exception as e:
        return None, f"❌ Error: {str(e)}", None, None, None, None


def generate_and_display_random():
    """Generate a random puzzle and display it."""
    board = generate_random_puzzle()
    board_str = ' '.join([str(tile) for row in board for tile in row])
    img = create_puzzle_image(board)
    return board_str, img, None, None, None, None


def create_demo():
    """Create the Gradio interface."""

    with gr.Blocks(theme=gr.themes.Soft(), title="🧩 8-Puzzle AI Solver") as demo:
        gr.Markdown("""
        # 🧩 8-Puzzle AI Solver
        ### Solve the classic 8-puzzle using various AI search algorithms!

        The 8-puzzle is a sliding puzzle consisting of a 3×3 grid with 8 numbered tiles and one empty space (represented by 0).
        The goal is to arrange the tiles from the initial state to the goal state: `0 1 2 / 3 4 5 / 6 7 8`
        """)

        with gr.Row():
            with gr.Column(scale=1):
                gr.Markdown("### 🎮 Input Configuration")

                board_input = gr.Textbox(
                    label="Puzzle Configuration",
                    placeholder="0 1 2 3 4 5 6 7 8",
                    value="1 2 3 4 5 6 7 8 0",
                    info="Enter 9 numbers (0-8) separated by spaces. 0 represents the empty tile."
                )

                algorithm_choice = gr.Radio(
                    choices=["BFS", "DFS", "Iterative DFS", "A* (Manhattan)", "A* (Euclidean)"],
                    value="A* (Manhattan)",
                    label="Select Algorithm",
                    info="Choose which search algorithm to use"
                )

                with gr.Row():
                    solve_btn = gr.Button("🚀 Solve Puzzle", variant="primary", size="lg")
                    random_btn = gr.Button("🎲 Random Puzzle", variant="secondary", size="lg")

                gr.Markdown("""
                ### 📚 Algorithm Descriptions:
                - **BFS**: Breadth-First Search - Explores all nodes at current depth before going deeper
                - **DFS**: Depth-First Search - Explores as far as possible along each branch
                - **Iterative DFS**: Gradually increases depth limit, combines BFS and DFS benefits
                - **A* (Manhattan)**: Uses Manhattan distance heuristic for optimal pathfinding
                - **A* (Euclidean)**: Uses Euclidean distance heuristic for optimal pathfinding
                """)

            with gr.Column(scale=1):
                gr.Markdown("### 📊 Results")

                initial_image = gr.Image(label="Initial State", type="pil", height=400)
                solution_stats = gr.Markdown("Solve a puzzle to see statistics here!")

        with gr.Row():
            with gr.Column():
                final_image = gr.Image(label="Final State", type="pil", height=400)
            with gr.Column():
                solution_gallery = gr.Gallery(
                    label="Solution Steps (Click to view each step)",
                    columns=4,
                    rows=2,
                    height=400,
                    object_fit="contain"
                )

        with gr.Row():
            move_count = gr.Number(label="Total Moves", precision=0)
            move_sequence = gr.Textbox(label="Move Sequence", max_lines=3)

        gr.Markdown("""
        ### 💡 Tips:
        - Not all puzzles are solvable! Only puzzles with an even number of inversions can be solved.
        - Try different algorithms and compare their performance.
        - A* algorithms are typically the fastest for this problem.
        - The goal state is: `0 1 2 / 3 4 5 / 6 7 8`

        ---
        Made with ❤️ by **Ab-Romia** | [GitHub](https://github.com/Ab-Romia/8_Puzzle-AI)
        """)

        # Event handlers
        solve_btn.click(
            fn=solve_puzzle,
            inputs=[board_input, algorithm_choice],
            outputs=[initial_image, solution_stats, final_image, solution_gallery, move_count, move_sequence]
        )

        random_btn.click(
            fn=generate_and_display_random,
            inputs=[],
            outputs=[board_input, initial_image, solution_stats, final_image, solution_gallery, move_count]
        )

    return demo


if __name__ == "__main__":
    demo = create_demo()
    demo.launch()
