#!/usr/bin/env python3
"""Aggregate metrics.json files across repeated trials."""

import argparse
import json
from pathlib import Path

from hnbv_eval.metrics import aggregate_trials


def _flatten_metrics(metrics: dict, prefix: str = "") -> dict[str, float]:
    flat = {}
    for key, value in metrics.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            flat.update(_flatten_metrics(value, name))
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            flat[name] = float(value)
    return flat


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--env", required=True)
    parser.add_argument("--modes", nargs="+", required=True)
    args = parser.parse_args()

    root = Path(args.root)
    output = {}
    for mode in args.modes:
        metric_values: dict[str, list[float]] = {}
        for metrics_path in sorted((root / args.env / mode).glob("trial_*/metrics.json")):
            metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            for key, value in _flatten_metrics(metrics).items():
                metric_values.setdefault(key, []).append(value)
        if metric_values:
            output[mode] = aggregate_trials(metric_values)

    out_dir = root / args.env
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "summary.json"
    out_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(out_path)


if __name__ == "__main__":
    main()

