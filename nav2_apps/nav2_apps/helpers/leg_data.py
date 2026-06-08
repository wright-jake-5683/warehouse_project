import math
from dataclasses import dataclass, field
from nav2_apps.helpers.robo_math import Point2D 

@dataclass
class LegData:
    index: int = -1
    distance: float = -1.0
    angle: float = float("nan")
    point: Point2D = field(default_factory=Point2D)

    def is_empty(self) -> bool:
        return (
            self.index < 0 and
            self.distance < 0.0 and
            math.isnan(self.angle)
        )