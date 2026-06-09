#!/usr/bin/env python3
"""Placeholder rosbag extractor for Ubuntu ROS runtime."""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out_dir", required=True)
    args = parser.parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "README.txt").write_text(
        f"Rosbag extraction contract created for {args.bag}.\n",
        encoding="utf-8",
    )
    print(out_dir)


if __name__ == "__main__":
    main()

