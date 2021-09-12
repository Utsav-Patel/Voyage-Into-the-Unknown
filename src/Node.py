from constants import INF


class Node:
    def __init__(self):
        self.g = INF
        self.h = INF
        self.f = INF
        self.is_visited = False
        self.is_blocked = False
