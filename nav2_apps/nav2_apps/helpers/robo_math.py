import math
from dataclasses import dataclass


@dataclass
class Point2D:
    x: float = 0.0
    y: float = 0.0


class RoboMath:

    @staticmethod
    def find_2d_coords_from_hypotenuse(r: float, theta: float) -> Point2D:
        return Point2D(
            x=r * math.cos(theta),
            y=r * math.sin(theta)
        )

    @staticmethod
    def find_midpoint(point_1: Point2D, point_2: Point2D) -> Point2D:
        return Point2D(
            x=(point_1.x + point_2.x) / 2,
            y=(point_1.y + point_2.y) / 2
        )

    @staticmethod
    def calculate_vel_by_distance(meters: float, seconds: float) -> float:
        return meters / seconds