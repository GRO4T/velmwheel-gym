import cv2

from velmwheel_rl_navigation.global_planner.astar import astar
from velmwheel_rl_navigation.utils import Color, Point
from velmwheel_rl_navigation.global_planner.map_2d import Map2D


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
    map_2d.draw_point(map_2d.coords_to_pixels(*start), Color.RED)

    # Draw goal
    map_2d.draw_point(map_2d.coords_to_pixels(*goal), Color.BLUE)

    # Find the path to goal
    path = astar(map_2d, start, goal)

    if not path:
        raise Exception("Path not found")

    # Draw path
    for position in path:
        map_2d.set_pixel(position, Color.GREEN)

    # Display the final image
    map_2d.show()


if __name__ == "__main__":
    main()
