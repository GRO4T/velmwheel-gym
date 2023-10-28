from velmwheel_rl_navigation.utils import Point


class Node:
    def __init__(self, parent=None, position: Point = None):
        self.parent = parent
        self.position = position

        self.g = 0  # distance between the current node and the start node
        self.h = 0  # heuristic function - estimated distance from the current node to the end node
        self.f = 0  # total cost of the node

    def __eq__(self, other):
        return self.position == other.position
