import math

from swarm_control.geometry import (
    Fence, SHAPES, assign_slots, check_separation, dist, flight_duration,
    formation_targets,
)


def _min_pair(pts):
    return min(dist(a, b) for i, a in enumerate(pts) for b in pts[i + 1:])


def test_shapes_respect_spacing():
    for shape in SHAPES:
        for n in range(2, 16):
            pts = formation_targets(shape, n, (1.0, 0.5, 1.0), 0.6, 0.7)
            assert len(pts) == n
            assert _min_pair(pts) >= 0.6 - 1e-6, (shape, n)
            assert all(abs(p[2] - 1.0) < 1e-9 for p in pts)


def test_formation_centred():
    for shape in SHAPES:
        pts = formation_targets(shape, 6, (2.0, 1.0, 0.5), 0.5, 0.0)
        cx = sum(p[0] for p in pts) / 6
        cy = sum(p[1] for p in pts) / 6
        assert abs(cx - 2.0) < 0.3 and abs(cy - 1.0) < 0.3, shape


def test_assignment_is_optimal_for_swap():
    starts = [(0, 0, 1), (1, 0, 1)]
    goals = [(1.1, 0, 1), (0.1, 0, 1)]
    assert assign_slots(starts, goals) == [1, 0]


def test_fence_and_separation_and_duration():
    f = Fence(-1, 3, -1, 2, 0.2, 2)
    assert f.contains((0, 0, 1)) and not f.contains((5, 0, 1))
    assert f.clamp((5, -3, 0)) == (3, -1, 0.2)
    ok, pair, d = check_separation({'a': (0, 0, 0), 'b': (0.2, 0, 0), 'c': (2, 0, 0)}, 0.3)
    assert not ok and set(pair) == {'a', 'b'} and math.isclose(d, 0.2)
    assert math.isclose(flight_duration((0, 0, 0), (2, 0, 0), 0.5, 1.0), 4.0)
    assert flight_duration((0, 0, 0), (0.1, 0, 0), 0.5, 1.0) == 1.0
    assert flight_duration((0, 0, 0), (2, 0, 0), 0.5, 1.0, requested=6.0) == 6.0
