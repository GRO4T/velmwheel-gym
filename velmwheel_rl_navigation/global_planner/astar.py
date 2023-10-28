from velmwheel_rl_navigation.global_planner.map_2d import Map2D
from velmwheel_rl_navigation.global_planner.node import Node
from velmwheel_rl_navigation.utils import Point, Color


def manhattan_heuristic(start: Point, dest: Point) -> float:
    return abs(start.x - dest.x) + abs(start.y - dest.y)


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
        color = map_2d.get_pixel(node_position)
        if Color.BLACK == color:
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
