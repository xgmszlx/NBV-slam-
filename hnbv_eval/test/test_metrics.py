import math

import numpy as np

from hnbv_eval.metrics import (
    compute_coverage_ratio,
    compute_entropy_grid,
    compute_err,
    compute_near_collision_events,
    compute_travel_distance,
    aggregate_trials,
)


def test_travel_distance_sums_segment_lengths():
    assert compute_travel_distance([(0, 0), (3, 4), (6, 8)]) == 10.0


def test_entropy_is_larger_for_unknown_cells_than_known_cells():
    unknown = np.full((2, 2), -1)
    known = np.array([[0, 100], [0, 100]])
    assert compute_entropy_grid(unknown) > compute_entropy_grid(known)


def test_err_uses_entropy_reduction_per_second():
    initial = np.full((2, 2), -1)
    final = np.array([[0, 100], [0, 100]])
    expected = (compute_entropy_grid(initial) - compute_entropy_grid(final)) / 2.0
    assert math.isclose(compute_err(initial, final, 2.0), expected)


def test_coverage_ratio_counts_known_cells_inside_roi():
    grid = np.array([[-1, 0], [100, -1]])
    roi = np.array([[True, True], [True, False]])
    assert compute_coverage_ratio(grid, roi) == 2 / 3


def test_near_collision_requires_minimum_duration():
    times = [0, 1, 2, 3, 4, 5, 6]
    distances = [3.0, 1.8, 1.7, 1.6, 2.5, 1.5, 2.5]
    events = compute_near_collision_events(times, distances, threshold_m=2.0, min_duration_s=3.0)
    assert len(events) == 1
    assert events[0].start_time == 1
    assert events[0].end_time == 3
    assert events[0].min_distance_m == 1.6


def test_aggregate_trials_returns_mean_std_ci_and_count():
    summary = aggregate_trials({"ate_rmse_m": [0.2, 0.3, 0.4]})
    assert summary["ate_rmse_m"]["mean"] == 0.3
    assert summary["ate_rmse_m"]["n"] == 3.0
    assert summary["ate_rmse_m"]["ci95"] > 0.0

