from src.Node import Node


class Maze:
    def __init__(self, num_cols: int, num_rows: int):
        self.num_cols = num_cols
        self.num_rows = num_rows
        self.maze = list()
        for row_num in range(self.num_rows):
            lst = list()
            for column_num in range(self.num_cols):
                lst.append(Node())
            self.maze.append(lst)