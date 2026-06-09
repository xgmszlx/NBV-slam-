#!/usr/bin/env python3
"""Compute metrics for one exported experiment run directory."""

import argparse
import json
from pathlib import Path

import numpy as np

from hnbv_eval.metrics import compute_coverage_ratio, compute_entropy_grid, compute_err


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_dir", required=True, help="Directory containing exported run data")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    initial_map = run_dir / "maps" / "occupancy_initial.npy"
    final_map = run_dir / "maps" / "occupancy_final.npy"
    manifest = run_dir / "manifest.yaml"

    metrics = {
        "run_id": run_dir.name,
        "success": False,
        "stop_reason": "not_evaluated",
        "efficiency": {},
        "mapping": {},
        "slam": {},
        "dynamic": {},
        "runtime": {},
        "inputs": {
            "manifest_exists": manifest.exists(),
            "initial_map_exists": initial_map.exists(),
            "final_map_exists": final_map.exists(),
        },
    }

    if initial_map.exists() and final_map.exists():
        grid0 = np.load(initial_map)
        grid1 = np.load(final_map)
        travel_time_s = 1.0
        metrics["mapping"] = {
            "entropy_initial_bits": compute_entropy_grid(grid0),
            "entropy_final_bits": compute_entropy_grid(grid1),
            "err_bits_per_s": compute_err(grid0, grid1, travel_time_s),
            "coverage_final": compute_coverage_ratio(grid1),
        }

    out_path = run_dir / "metrics.json"
    out_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    print(out_path)


if __name__ == "__main__":
    main()

