# 🧩 8-Puzzle AI Solver

[![Hugging Face Spaces](https://img.shields.io/badge/%F0%9F%A4%97%20Hugging%20Face-Spaces-blue)](https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An intelligent 8-puzzle solver featuring multiple AI search algorithms with an interactive web interface. Try it live on [Hugging Face Spaces](https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI)!

## 🌟 Features

- **🎯 Multiple AI Algorithms**: Compare performance across BFS, DFS, Iterative DFS, and A* search algorithms
- **🎨 Interactive Web Interface**: Beautiful, user-friendly UI built with Gradio
- **📊 Real-time Visualization**: Watch the solution unfold step-by-step
- **📈 Performance Metrics**: Track nodes expanded, depth, time taken, and more
- **🎲 Random Puzzle Generator**: Generate solvable puzzles instantly
- **✅ Solvability Checker**: Automatically detects unsolvable configurations
- **🚀 Cloud Deployment**: Accessible anywhere via Hugging Face Spaces

## 🎮 Demo

Try the live demo: **[8-Puzzle AI Solver on Hugging Face](https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI)**

## 🧠 Algorithms Implemented

### 1. Breadth-First Search (BFS)
- **Guarantee**: Finds the shortest path
- **Strategy**: Explores all nodes at the current depth before moving deeper
- **Pros**: Complete and optimal
- **Cons**: High memory usage for complex puzzles

### 2. Depth-First Search (DFS)
- **Strategy**: Explores as far as possible along each branch before backtracking
- **Pros**: Memory efficient
- **Cons**: May not find the shortest path

### 3. Iterative Deepening DFS
- **Strategy**: Combines BFS and DFS by gradually increasing depth limit
- **Pros**: Finds shortest path with better memory efficiency than BFS
- **Cons**: Revisits nodes multiple times

### 4. A* Search (Manhattan Distance)
- **Heuristic**: Sum of Manhattan distances of tiles from their goal positions
- **Guarantee**: Finds optimal solution when using admissible heuristic
- **Pros**: Very efficient, guides search toward goal
- **Best for**: Most 8-puzzle problems

### 5. A* Search (Euclidean Distance)
- **Heuristic**: Sum of Euclidean distances of tiles from their goal positions
- **Similar to**: Manhattan A* but uses straight-line distance
- **Pros**: Also optimal and efficient

## 📦 Project Structure

```
8_Puzzle-AI/
├── app.py                 # Gradio web interface
├── 8PUZZLE.py            # Command-line interface
├── DONE/                 # Core algorithm implementations
│   ├── __init__.py
│   ├── state.py          # State representation
│   ├── search_algorithms.py  # BFS, DFS, IDFS, A*
│   ├── heuristics.py     # Heuristic functions
│   ├── data_structures.py    # Queue, Stack, Heap
│   └── visualize.py      # Graphviz visualization
├── requirements.txt      # Python dependencies
├── README.md            # This file
└── report.pdf           # Detailed algorithm analysis
```

## 🚀 Quick Start

### Online (Recommended)
Visit [Hugging Face Spaces](https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI) to use the app instantly without any installation!

### Local Installation

1. **Clone the repository**
```bash
git clone https://github.com/Ab-Romia/8_Puzzle-AI.git
cd 8_Puzzle-AI
```

2. **Install dependencies**
```bash
pip install -r requirements.txt
```

3. **Run the web interface**
```bash
python app.py
```

4. **Or run the command-line version**
```bash
python 8PUZZLE.py
```

## 💻 Usage

### Web Interface

1. **Enter a puzzle**: Input 9 numbers (0-8) separated by spaces, where 0 represents the empty tile
   - Example: `1 2 3 4 5 6 7 8 0`

2. **Select an algorithm**: Choose from BFS, DFS, Iterative DFS, A* (Manhattan), or A* (Euclidean)

3. **Click "Solve Puzzle"**: Watch the AI find the solution!

4. **Or generate a random puzzle**: Click "Random Puzzle" for instant solvable configurations

### Command-Line Interface

Edit `8PUZZLE.py` to set your initial state:

```python
initial_state = State([[1, 2, 3], [4, 5, 6], [7, 8, 0]], paths=[])
```

Run the script:
```bash
python 8PUZZLE.py
```

## 🎯 Goal State

The goal is to reach this configuration:

```
0 1 2
3 4 5
6 7 8
```

Where 0 represents the empty space.

## 📊 Performance Comparison

| Algorithm | Optimality | Completeness | Memory | Speed |
|-----------|-----------|--------------|---------|-------|
| BFS | ✅ Yes | ✅ Yes | ❌ High | 🟡 Medium |
| DFS | ❌ No | ⚠️ Not guaranteed | ✅ Low | 🟢 Fast |
| Iterative DFS | ✅ Yes | ✅ Yes | 🟡 Medium | 🟡 Medium |
| A* (Manhattan) | ✅ Yes | ✅ Yes | 🟡 Medium | 🟢 Very Fast |
| A* (Euclidean) | ✅ Yes | ✅ Yes | 🟡 Medium | 🟢 Very Fast |

## 🔬 Technical Details

### Solvability
Not all 8-puzzle configurations are solvable. A puzzle is solvable if and only if the number of inversions is even. An inversion is when a tile precedes another tile with a lower number.

### Complexity
- **State Space**: 9! = 362,880 possible states
- **Solvable States**: 181,440 (exactly half)
- **Average Solution Length**: ~22 moves
- **Maximum Solution Length**: 31 moves

## 🤝 Contributing

Contributions are welcome! Feel free to:
- Report bugs
- Suggest new features
- Improve documentation
- Add new algorithms

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👤 Author

**Ab-Romia**
- GitHub: [@Ab-Romia](https://github.com/Ab-Romia)
- Hugging Face: [@Ab-Romia](https://huggingface.co/Ab-Romia)

## 🙏 Acknowledgments

- Classic AI problem from artificial intelligence coursework
- Built with [Gradio](https://gradio.app/) for the web interface
- Deployed on [Hugging Face Spaces](https://huggingface.co/spaces)

## 📚 References

- Russell, S., & Norvig, P. (2020). *Artificial Intelligence: A Modern Approach* (4th ed.)
- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*

---

⭐ If you found this project helpful, please give it a star on GitHub!

🚀 **[Try it now on Hugging Face Spaces!](https://huggingface.co/spaces/Ab-Romia/8-Puzzle-AI)**
