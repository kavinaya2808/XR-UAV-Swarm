"""Pure-Python helpers (no ROS): formation shapes, slot assignment, safety checks.

Kept free of rclpy so they can be unit-tested and reused.
"""
import math
from itertools import permutations

Vec = tuple  # (x, y, z)

SHAPES = ('line', 'grid', 'circle', 'v')


# --------------------------------------------------------------------- shapes
def formation_offsets(shape, n, spacing):
    """Offsets (dx, dy) around (0, 0) for n drones, before rotation."""
    if n <= 0:
        return []
    if shape == 'line':
        return [((i - (n - 1) / 2.0) * spacing, 0.0) for i in range(n)]
    if shape == 'grid':
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)
        pts = []
        for i in range(n):
            r, c = divmod(i, cols)
            pts.append(((c - (cols - 1) / 2.0) * spacing,
                        (r - (rows - 1) / 2.0) * spacing))
        return pts
    if shape == 'circle':
        if n == 1:
            return [(0.0, 0.0)]
        # chord between neighbours = spacing
        radius = spacing / (2.0 * math.sin(math.pi / n))
        return [(radius * math.cos(2 * math.pi * i / n),
                 radius * math.sin(2 * math.pi * i / n)) for i in range(n)]
    if shape == 'v':
        # leader at the tip (pointing +x), the rest alternate left/right behind it
        # on two arms at +-35 deg; neighbours on an arm are `spacing` apart
        a = math.radians(35)
        pts = [(0.0, 0.0)]
        for i in range(1, n):
            k = (i + 1) // 2
            side = 1 if i % 2 else -1
            pts.append((-k * spacing * math.cos(a), side * k * spacing * math.sin(a)))
        # centre the shape on its centroid
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        return [(x - cx, y - cy) for x, y in pts]
    raise ValueError(f'unknown shape "{shape}" (use one of {", ".join(SHAPES)})')


def formation_targets(shape, n, center, spacing, heading):
    """World-frame targets for a formation centred at `center`, rotated by heading."""
    c, s = math.cos(heading), math.sin(heading)
    return [(center[0] + c * dx - s * dy,
             center[1] + s * dx + c * dy,
             center[2]) for dx, dy in formation_offsets(shape, n, spacing)]


# ----------------------------------------------------------------- assignment
def dist(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def assign_slots(starts, goals):
    """Return perm so that drone i flies to goals[perm[i]], minimising total distance.

    Uses scipy's Hungarian solver if available; otherwise brute force (n <= 8)
    or a greedy fallback.
    """
    n = len(starts)
    if n == 0:
        return []
    cost = [[dist(s, g) for g in goals] for s in starts]
    try:
        from scipy.optimize import linear_sum_assignment
        rows, cols = linear_sum_assignment(cost)
        perm = [0] * n
        for r, c in zip(rows, cols):
            perm[r] = int(c)
        return perm
    except ImportError:
        pass
    if n <= 8:
        return list(min(permutations(range(n)),
                        key=lambda p: sum(cost[i][p[i]] for i in range(n))))
    perm, free = [None] * n, set(range(n))
    pairs = sorted((cost[i][j], i, j) for i in range(n) for j in range(n))
    for _, i, j in pairs:
        if perm[i] is None and j in free:
            perm[i] = j
            free.discard(j)
    return perm


# --------------------------------------------------------------------- safety
class Fence:
    """Axis-aligned flight volume."""

    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.lo = (x_min, y_min, z_min)
        self.hi = (x_max, y_max, z_max)

    def contains(self, p):
        return all(lo <= v <= hi for v, lo, hi in zip(p, self.lo, self.hi))

    def clamp(self, p):
        return tuple(min(max(v, lo), hi) for v, lo, hi in zip(p, self.lo, self.hi))


def check_separation(points, min_sep):
    """Return (ok, closest_pair_index_tuple, distance) for a dict id -> point."""
    ids = list(points)
    worst = (None, math.inf)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            d = dist(points[ids[i]], points[ids[j]])
            if d < worst[1]:
                worst = ((ids[i], ids[j]), d)
    return worst[1] >= min_sep, worst[0], worst[1]


def flight_duration(start, goal, max_speed, min_duration, requested=0.0):
    """Duration that respects max_speed; never shorter than requested/min."""
    needed = dist(start, goal) / max_speed if max_speed > 0 else 0.0
    return max(needed, min_duration, requested)
