#!/usr/bin/env python3
"""Launch repeated Ubuntu-side ROS experiments.

This script is intentionally a runner stub until the ROS launch files and world
models are fully implemented. It creates deterministic run directories and
manifest files so downstream evaluation tooling has a stable contract.
"""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", required=True, choices=["env1", "env2"])
    parser.add_argument("--mode", required=True)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--record-bag", action="store_true")
    parser.add_argument("--root", default="~/hnbv_runs")
    args = parser.parse_args()

    root = Path(args.root).expanduser() / args.env / args.mode
    root.mkdir(parents=True, exist_ok=True)
    for trial in range(1, args.trials + 1):
        trial_dir = root / f"trial_{trial:03d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        (trial_dir / "manifest.yaml").write_text(
            "\n".join(
                [
                    f"env: {args.env}",
                    f"mode: {args.mode}",
                    f"trial: {trial}",
                    f"record_bag: {str(args.record_bag).lower()}",
                    "status: planned",
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        print(trial_dir)


if __name__ == "__main__":
    main()

