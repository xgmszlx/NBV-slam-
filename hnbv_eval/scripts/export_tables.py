#!/usr/bin/env python3
"""Export paper-style CSV/Markdown tables from aggregate summaries."""

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_dir", required=True)
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser()
    summary_path = run_dir / "summary.json"
    if summary_path.exists():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    else:
        summary = {}

    csv_path = run_dir / "summary_table.csv"
    md_path = run_dir / "summary_table.md"
    csv_lines = ["method,metric,mean,std,ci95"]
    lines = ["| Method | Metric | Mean | Std | CI95 |", "| --- | --- | ---: | ---: | ---: |"]
    for method, metrics in summary.items():
        for metric, values in metrics.items():
            csv_lines.append(
                f"{method},{metric},{values.get('mean', '')},{values.get('std', '')},{values.get('ci95', '')}"
            )
            lines.append(
                f"| {method} | {metric} | {values.get('mean', '')} | {values.get('std', '')} | {values.get('ci95', '')} |"
            )
    csv_path.write_text("\n".join(csv_lines) + "\n", encoding="utf-8")
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(md_path)


if __name__ == "__main__":
    main()
