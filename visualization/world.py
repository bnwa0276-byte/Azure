"""World coordinate management for the GCS."""
from typing import Tuple

def map_world_to_screen(pos: Tuple[float, float, float]) -> Tuple[float, float]:
    """Map a 3D world position to 2D screen coordinates (identity mapping for now)."""
    x, y, _ = pos
    return float(x), float(y)

def clamp_viewport(points, padding=1.0):
    """Compute a simple bounding box for a set of 2D points with padding."""
    if not points:
        return (-padding, -padding, padding, padding)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    return (xmin - padding, ymin - padding, xmax + padding, ymax + padding)
