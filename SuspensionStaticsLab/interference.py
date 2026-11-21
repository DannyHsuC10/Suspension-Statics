"""Check axial rod envelopes on the undeformed hardpoint geometry."""

from dataclasses import dataclass
from itertools import combinations
from math import hypot, isfinite

import numpy as np


TOLERANCE_M = 1e-9


@dataclass(frozen=True)
class Interference:
    rod_a: str
    rod_b: str
    distance_mm: float
    required_mm: float

    @property
    def overlap_mm(self):
        return self.required_mm - self.distance_mm


def _point_distance(point, start, direction, length):
    offset = point - start
    projection = float(np.dot(offset, direction))
    return hypot(*(offset - min(length, max(0., projection)) * direction))


def _segment_distance(a, b):
    start_a, end_a = a
    start_b, end_b = b
    axis_a, axis_b = end_a - start_a, end_b - start_b
    length_a, length_b = hypot(*axis_a), hypot(*axis_b)
    unit_a, unit_b = axis_a / length_a, axis_b / length_b
    best = min(
        _point_distance(start_a, start_b, unit_b, length_b),
        _point_distance(end_a, start_b, unit_b, length_b),
        _point_distance(start_b, start_a, unit_a, length_a),
        _point_distance(end_b, start_a, unit_a, length_a),
    )
    # The remaining candidate is the common perpendicular between the two axes.
    cross = np.cross(unit_a, unit_b)
    sine = hypot(*cross)
    if sine > 0.:
        normal = cross / sine
        offset = start_b - start_a
        at_a = float(np.dot(np.cross(offset, unit_b), normal)) / sine
        at_b = float(np.dot(np.cross(offset, unit_a), normal)) / sine
        if 0. <= at_a <= length_a and 0. <= at_b <= length_b:
            best = min(best, abs(float(np.dot(offset, normal))))
    return best


def _trim(segment, length, end, allowance):
    if length - allowance <= TOLERANCE_M:
        return None
    points = list(segment)
    points[end] = points[end] + allowance / length * (points[1 - end] - points[end])
    return points


def find_interferences(model, sections):
    """Return interfering rod pairs ordered by the depth of their intersection."""
    shafts = []
    for rod in model.rods:
        section = sections.get(rod.name)
        if section is None:
            continue
        try:
            section.area_mm2()
        except ValueError:
            continue
        start = np.asarray(model.points[rod.a], dtype=float)
        end = np.asarray(model.points[rod.b], dtype=float)
        length = hypot(*(end - start))
        if isfinite(length) and length > TOLERANCE_M:
            shafts.append((rod.name, (start, end), length, section.outer_mm / 2.))

    interferences = []
    for (name_a, a, length_a, radius_a), (name_b, b, length_b, radius_b) in combinations(shafts, 2):
        required_mm = radius_a + radius_b
        required_m = required_mm / 1000.
        shared = {(i, j) for i in range(2) for j in range(2) if hypot(*(a[i] - b[j])) <= TOLERANCE_M}
        coincident = {(0, 0), (1, 1)} <= shared or {(0, 1), (1, 0)} <= shared
        if shared and not coincident:
            i, j = min(shared)
            # Exclude the immediate joint area while checking the remaining shafts.
            a = _trim(a, length_a, i, 2. * required_m)
            b = _trim(b, length_b, j, 2. * required_m)
            if a is None or b is None:
                continue
        distance_m = _segment_distance(a, b)
        if distance_m < required_m - TOLERANCE_M:
            first, second = sorted((name_a, name_b))
            interferences.append(Interference(first, second, distance_m * 1000., required_mm))
    return tuple(sorted(interferences, key=lambda hit: (-hit.overlap_mm, hit.rod_a, hit.rod_b)))
