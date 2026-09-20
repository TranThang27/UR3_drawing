import math


def circle_points(x, y, z, radius, step):
    if not all(math.isfinite(v) for v in (x, y, z, radius, step)):
        raise ValueError("Coordinates and dimensions must be finite")
    if min(radius, step) <= 0:
        raise ValueError("radius and step must be positive")

    point_count = max(24, math.ceil(2.0 * math.pi * radius / step))
    return [
        (
            x + radius * math.sin(2.0 * math.pi * i / point_count),
            y + radius * (1.0 - math.cos(2.0 * math.pi * i / point_count)),
            z,
        )
        for i in range(point_count + 1)
    ]
