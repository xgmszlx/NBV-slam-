#!/usr/bin/env python3
"""Generate a Markdown aggregate report."""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--env", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        f"# HNBV Aggregate Report\n\n- root: `{args.root}`\n- env: `{args.env}`\n",
        encoding="utf-8",
    )
    print(out_path)


if __name__ == "__main__":
    main()

