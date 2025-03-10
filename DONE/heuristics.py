import math

def manhattan_heuristic(board):
    distance = 0
    for i in range(3):
        for j in range(3):
            if board[i][j] != 0:
                goal_x, goal_y = divmod(board[i][j], 3)
                distance += abs(i - goal_x) + abs(j - goal_y)
    return distance

def euclidean_heuristic(board):
    distance = 0
    for i in range(3):
        for j in range(3):
            if board[i][j] != 0:
                goal_x, goal_y = divmod(board[i][j], 3)
                distance += math.sqrt((i - goal_x) ** 2 + (j - goal_y) ** 2)
    return distance

def heuristic_func(board):
    goal = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    return manhattan_heuristic(board) + euclidean_heuristic(board)