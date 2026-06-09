"""Numerical metrics for active semantic SLAM experiments.

The functions in this module deliberately avoid ROS dependencies so they can be
unit-tested and used on CSV/NPY exports from recorded rosbag files.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterable, Sequence

import numpy as np


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass(frozen=True)
class NearCollisionEvent:
    """A continuous interval where robot-person distance stays below a threshold."""

    start_time: float
    end_time: float
    min_distance_m: float

    @property
    def duration_s(self) -> float:
        return self.end_time - self.start_time


def _as_points_xy(points: Sequence[Sequence[float]]) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 2:
        raise ValueError("points must be an Nx2 or NxM array")
    return arr[:, :2]


def compute_travel_distance(points_xy: Sequence[Sequence[float]]) -> float:
    """Return path length in meters from ordered 2D or 3D positions."""

    points = _as_points_xy(points_xy)
    if len(points) < 2:
        return 0.0
    deltas = np.diff(points, axis=0)
    return float(np.linalg.norm(deltas, axis=1).sum())


def occupancy_to_probability(grid: np.ndarray) -> np.ndarray:
    """Convert ROS-style occupancy values into Bernoulli occupancy probabilities."""

    grid = np.asarray(grid)
    prob = np.empty(grid.shape, dtype=float)
    prob[grid < 0] = 0.5
    prob[grid == 0] = 0.01
    prob[grid > 0] = 0.99
    return np.clip(prob, 1.0e-6, 1.0 - 1.0e-6)


def compute_entropy_grid(grid: np.ndarray) -> float:
    """Compute Shannon entropy of an occupancy grid in bits."""

    p = occupancy_to_probability(grid)
    entropy = -p * np.log2(p) - (1.0 - p) * np.log2(1.0 - p)
    return float(np.sum(entropy))


def compute_err(initial_grid: np.ndarray, final_grid: np.ndarray, travel_time_s: float) -> float:
    """Compute map Entropy Reduction Rate in bits per second."""

    if travel_time_s <= 0:
        raise ValueError("travel_time_s must be positive")
    return (compute_entropy_grid(initial_grid) - compute_entropy_grid(final_grid)) / travel_time_s


def compute_coverage_ratio(grid: np.ndarray, roi_mask: np.ndarray | None = None) -> float:
    """Return the known-cell ratio inside an optional ROI mask."""

    grid = np.asarray(grid)
    if roi_mask is None:
        roi = np.ones(grid.shape, dtype=bool)
    else:
        roi = np.asarray(roi_mask, dtype=bool)
        if roi.shape != grid.shape:
            raise ValueError("roi_mask shape must match grid shape")
    total = int(np.count_nonzero(roi))
    if total == 0:
        raise ValueError("ROI contains no cells")
    known = np.count_nonzero((grid >= 0) & roi)
    return float(known / total)


def compute_near_collision_events(
    times_s: Sequence[float],
    distances_m: Sequence[float],
    threshold_m: float = 2.0,
    min_duration_s: float = 3.0,
) -> list[NearCollisionEvent]:
    """Detect near-collision intervals from a distance time series."""

    times = np.asarray(times_s, dtype=float)
    distances = np.asarray(distances_m, dtype=float)
    if times.shape != distances.shape:
        raise ValueError("times_s and distances_m must have the same length")
    if len(times) == 0:
        return []

    events: list[NearCollisionEvent] = []
    active_start: int | None = None
    active_min = float("inf")

    def close_event(end_idx: int) -> None:
        nonlocal active_start, active_min
        if active_start is None:
            return
        start_time = float(times[active_start])
        end_time = float(times[end_idx])
        if end_time - start_time >= min_duration_s:
            events.append(NearCollisionEvent(start_time, end_time, float(active_min)))
        active_start = None
        active_min = float("inf")

    for idx, distance in enumerate(distances):
        if distance < threshold_m:
            if active_start is None:
                active_start = idx
            active_min = min(active_min, float(distance))
        else:
            close_event(max(idx - 1, 0))
    close_event(len(times) - 1)
    return events


def confidence_interval_95(values: Iterable[float]) -> tuple[float, float]:
    """Return mean and 95% normal-approximation half-width."""

    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        raise ValueError("values must not be empty")
    mean = float(np.mean(arr))
    if arr.size == 1:
        return mean, 0.0
    half_width = 1.96 * float(np.std(arr, ddof=1)) / sqrt(arr.size)
    return mean, half_width


def aggregate_trials(metric_values: dict[str, Sequence[float]]) -> dict[str, dict[str, float]]:
    """Aggregate each metric over repeated trials."""

    summary: dict[str, dict[str, float]] = {}
    for name, values in metric_values.items():
        arr = np.asarray(values, dtype=float)
        if arr.size == 0:
            raise ValueError(f"metric {name} has no values")
        mean, ci95 = confidence_interval_95(arr)
        summary[name] = {
            "mean": mean,
            "std": float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
            "ci95": ci95,
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "n": float(arr.size),
        }
    return summary

