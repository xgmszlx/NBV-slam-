#!/usr/bin/env python3
"""Generate evaluation figures from aggregated metrics."""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--env", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "FIGURES.md").write_text(
        f"Figure output directory for {args.env} under {args.root}.\n",
        encoding="utf-8",
    )
    print(out_dir)


if __name__ == "__main__":
    main()

