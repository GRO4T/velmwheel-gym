import cv2
import numpy as np

from velmwheel_rl_navigation.utils import Point, Color


class Map2D:
    """
    A map represented as a 2D array of pixels.

    Attributes:
        width: Map's width.
        height: Map's height.
    """

    def __init__(
        self,
        data: np.ndarray,
        width: int,
        height: int,
        origin: Point,
        resolution: float,
    ):
        """
        Args:
            data: Array of pixels.
            width: Array's width.
            height: Array's height.
            origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
                    with yaw as counterclockwise rotation (yaw=0 means no rotation).
                    Many parts of the system currently ignore yaw.
            resolution: Meters per pixel.
        """
        self._data = data
        self._width = width
        self._height = height
        self._origin = origin
        self._resolution = resolution
    
    @property
    def width(self) -> int:
        """Map's width."""
        return self._width

    @property
    def height(self) -> int:
        """Map's width."""
        return self._height

    def show(self):
        cv2.imshow("My map", self._data)

        # Wait for user input then close the window
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def coords_to_pixels(self, x: float, y: float) -> Point:
        """Converts given coordinates to pixels on the image."""
        assert x >= self._origin.x and y >= self._origin.y
        x_pixels = int(abs(self._origin.x - x) / self._resolution)
        y_pixels = int(abs(self._origin.y - y) / self._resolution)
        return Point(x_pixels, self._height - y_pixels)

    def draw_point(self, center, color):
        self._data = cv2.circle(
            img=self._data, center=center, radius=5, color=color.value, thickness=-1
        )

    def set_pixel(self, position: Point, color: Color):
        self._data[position.y][position.x] = np.array(color.value)
    
    def get_pixel(self, position) -> np.ndarray:
        return self._data[position.y][position.x]
