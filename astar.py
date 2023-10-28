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
Point.__doc__ = "A point in 2D space."

Map2D_ = namedtuple("Map", "data width height origin resolution")


class Map2D(Map2D_):
    """
    A map represented as a 2D array of pixels.

    Attributes:
        data: Array of pixels.
        width: Array's width.
        height: Array's height.
        origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
                with yaw as counterclockwise rotation (yaw=0 means no rotation).
                Many parts of the system currently ignore yaw.
        resolution: Meters per pixel.
    """

    def coords_to_pixels(self, x: float, y: float) -> Point:
        """Converts given coordinates to pixels on the image."""
        assert x >= self.origin.x and y >= self.origin.y
        x_pixels = int(abs(self.origin.x - x) / self.resolution)
        y_pixels = int(abs(self.origin.y - y) / self.resolution)
        return Point(x_pixels, self.height - y_pixels)


def draw_point(img, center, color):
    img = cv2.circle(img=img, center=center, radius=5, color=color.value, thickness=-1)


def manhattan_heuristic(start: Point, dest: Point) -> float:
    return abs(start.x - dest.x) + abs(start.y - dest.y)


class Node:
    def __init__(self, parent=None, position: Point = None):
        self.parent = parent
        self.position = position

        self.g = 0  # distance between the current node and the start node
        self.h = 0  # heuristic function - estimated distance from the current node to the end node
        self.f = 0  # total cost of the node

    def __eq__(self, other):
        return self.position == other.position


def find_best_open_node(open_list: list, current_node: Node) -> tuple:
    current_index = 0
    for index, item in enumerate(open_list):
        if item.f < current_node.f:
            current_node = item
            current_index = index
    return current_node, current_index


def reconstruct_path(current_node: Node) -> list:
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path


def generate_children(map_2d: Map2D, current_node: Node) -> list:
    """
    Find adjacent nodes that are within map dimensions and are on walkable terrain.

    Args:
        map_2d:         Map represented by a 2D pixel array.
        current_node:   Currently processed node.

    Returns:
        List of adjacent nodes meeting the requirements.
    """
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
    ]:
        dx, dy = delta
        node_position = Point(
            current_node.position.x + dx,
            current_node.position.y + dy,
        )

        # Make sure within range
        if (
            node_position.x > (map_2d.width - 1)
            or node_position.x < 0
            or node_position.y > (map_2d.height - 1)
            or node_position.y < 0
        ):
            continue

        # Make sure walkable terrain
        color = map_2d.data[node_position.y][node_position.x]
        if color == Color.BLACK:
            continue

        new_node = Node(current_node, node_position)
        children.append(new_node)

    return children


def relax_child(
    child: Node, open_list: list, closed_list: list, current_node: Node, end_node: Node
):
    for closed_child in closed_list:
        if child == closed_child:
            return

    child.g = current_node.g + 1
    child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
        (child.position[1] - end_node.position[1]) ** 2
    )
    child.f = child.g + child.h

    for open_node in open_list:
        if child == open_node and child.g > open_node.g:
            return

    open_list.append(child)


def relax(
    children: list,
    open_list: list,
    closed_list: list,
    current_node: Node,
    end_node: Node,
):
    """
    Performs relaxation step of the A* algorithm.

    Side Effect: open_list is modified in-place.

    Args:
        children:       List of nodes adjacent to current.
        open_list:      List of open nodes.
        closed_list:    List of closed nodes.
        current_node:   Current node.
        end_node:       End node.
    """
    for child in children:
        relax_child(child, open_list, closed_list, current_node, end_node)


def astar(map_2d: Map2D, start: Point, end: Point) -> list:
    """
    Given a 2D map, finds the shortest path to goal using A* algorithm.

    Args:
        map_2d:         Map represented by a 2D pixel array.
        start:          Starting point coordinates.
        end:            Goal coordinates.

    Returns:
        Sequence of points that construct the shortest path to goal.
    """
    start_node = Node(None, map_2d.coords_to_pixels(start.x, start.y))
    end_node = Node(None, map_2d.coords_to_pixels(end.x, end.y))

    open_list = []
    closed_list = []
    path = []

    open_list.append(start_node)

    while len(open_list) > 0:
        current_node, current_index = find_best_open_node(
            open_list, current_node=open_list[0]
        )

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            return reconstruct_path(current_node)

        children = generate_children(map_2d, current_node)
        relax(children, open_list, closed_list, current_node, end_node)

    return []


def main():
    # Configuration
    map_filename = "test_map_preprocessed.pgm"
    origin = Point(-9.6, -10.5)
    resolution = 0.05
    start = Point(0, 0)
    goal = Point(7, 3)

    # Read the image
    img = cv2.imread(map_filename)
    img_width, img_height, _ = img.shape

    map_2d = Map2D(
        data=img,
        width=img_width,
        height=img_height,
        origin=origin,
        resolution=resolution,
    )

    # Draw starting position
    draw_point(img, map_2d.coords_to_pixels(*start), Color.RED)

    # Draw goal
    draw_point(img, map_2d.coords_to_pixels(*goal), Color.BLUE)

    # Find the path to goal
    path = astar(map_2d, start, goal)

    if not path:
        raise Exception("Path not found")

    # Draw path
    for position in path:
        x, y = position
        img[y][x] = np.array(Color.GREEN.value)

    # Display the final image
    cv2.imshow("A*", img)

    # Wait for user input then close the window
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
