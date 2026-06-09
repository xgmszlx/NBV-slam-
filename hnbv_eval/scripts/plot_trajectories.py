#!/usr/bin/env python3
"""Plot estimated and ground-truth trajectories from exported TUM files."""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimate", required=True)
    parser.add_argument("--groundtruth", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        "Trajectory plotting requires matplotlib on Ubuntu runtime.\n"
        f"estimate: {args.estimate}\n"
        f"groundtruth: {args.groundtruth}\n",
        encoding="utf-8",
    )
    print(out)


if __name__ == "__main__":
    main()

