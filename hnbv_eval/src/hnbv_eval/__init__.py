"""Offline evaluation utilities for hierarchical NBV active SLAM."""

from .metrics import (
    aggregate_trials,
    compute_coverage_ratio,
    compute_entropy_grid,
    compute_err,
    compute_near_collision_events,
    compute_travel_distance,
    confidence_interval_95,
)

__all__ = [
    "aggregate_trials",
    "compute_coverage_ratio",
    "compute_entropy_grid",
    "compute_err",
    "compute_near_collision_events",
    "compute_travel_distance",
    "confidence_interval_95",
]

