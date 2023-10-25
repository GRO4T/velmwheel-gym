from collections import namedtuple
from enum import Enum

import cv2
import numpy as np


def is_iterable(obj):
    try:
        iter(obj)
        return True
    except TypeError:
        return False


class Color(Enum):
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    BLACK = (0, 0, 0)
    WHITE = (254, 254, 254)

    def __eq__(self, other):
        if is_iterable(other):
            return all(self.value == other)
        raise TypeError(
            f"Comparison between {type(other)} and {type(self)} not supported"
        )


Point = namedtuple("Point", "x y")

"""
Meters per pixel.
"""
RESOLUTION = 0.05

"""
The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
with yaw as counterclockwise rotation (yaw=0 means no rotation).
Many parts of the system currently ignore yaw.
"""
ORIGIN = Point(-9.6, -10.5)


filename = "test_map_preprocessed.pgm"

img = cv2.imread(filename)
IMG_WIDTH, IMG_HEIGHT, _ = img.shape


def coords_to_pixels(x, y) -> Point:
    assert x >= ORIGIN.x and y >= ORIGIN.y
    x_pixels = int(abs(ORIGIN.x - x) / RESOLUTION)
    y_pixels = int(abs(ORIGIN.y - y) / RESOLUTION)
    return Point(x_pixels, IMG_HEIGHT - y_pixels)


def manhattan_heuristic(start: Point, dest: Point) -> float:
    return abs(start.x - dest.x) + abs(start.y - dest.y)


# draw goal
coords = coords_to_pixels(3, 3)
color = Color.BLUE.value
radius = 5
thickness = -1

img = cv2.circle(img, coords, radius, color, thickness)

# draw initial position
coords = coords_to_pixels(0, 0)
color = Color.RED.value
radius = 5
thickness = -1

img = cv2.circle(img, coords, radius, color, thickness)


# display final image
class Node:
    def __init__(self, parent=None, position: Point = None):
        self.parent = parent
        self.position = position

        self.g = 0  # distance between the current node and the start node
        self.h = 0  # heuristic function - estimated distance from the current node to the end node
        self.f = 0  # total cost of the node

    def __eq__(self, other):
        return self.position == other.position


start_node = Node(None, coords_to_pixels(0, 0))
end_node = Node(None, coords_to_pixels(3, 3))

open_list = []
closed_list = []

path = []

open_list.append(start_node)

while len(open_list) > 0:
    current_node = open_list[0]
    current_index = 0

    for index, item in enumerate(open_list):
        if item.f < current_node.f:
            current_node = item
            current_index = index

    open_list.pop(current_index)
    closed_list.append(current_node)

    if current_node == end_node:
        current = current_node
        while current is not None:
            path.append(current.position)
            current = current.parent
        break

    # Generate children
    children = []
    for delta in [
        (0, -1),
        (0, 1),
        (-1, 0),
        (1, 0),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    ]:  # Adjacent squares
        # Get node position
        dx, dy = delta
        node_position = Point(
            current_node.position.x + dx,
            current_node.position.y + dy,
        )

        # Make sure within range
        if (
            node_position.x > (IMG_WIDTH - 1)
            or node_position.x < 0
            or node_position.y > (IMG_HEIGHT - 1)
            or node_position.y < 0
        ):
            continue

        # Make sure walkable terrain
        color = img[node_position.x][node_position.y]
        if color == Color.BLACK:
            continue

        # Create new node
        new_node = Node(current_node, node_position)

        # Append
        children.append(new_node)

    for child in children:
        for closed_child in closed_list:
            if child == closed_child:
                continue

        child.g = current_node.g + 1
        child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
            (child.position[1] - end_node.position[1]) ** 2
        )
        child.f = child.g + child.h

        # Child is already in the open list
        for open_node in open_list:
            if child == open_node and child.g > open_node.g:
                continue

        # Add the child to the open list
        open_list.append(child)

# draw path
for position in path:
    x, y = position
    img[y][x] = np.array(Color.GREEN.value)

cv2.imshow("A*", img)

cv2.waitKey(0)
cv2.destroyAllWindows()
