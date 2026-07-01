#!/usr/bin/env python
#
# Grouped bar chart: SGM ratios by time bracket / category.
#
# Usage:
#   bar_plot.py [options] BASELINE.csv OTHER.csv [MORE.csv ...]

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import prepare_data, resolve_aliases
from plots import bar_comparison_plot, save


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Grouped bar chart of SGM ratios vs baseline."
    )
    parser.add_argument("files", nargs="+", help="CSV run files (first = baseline)")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--metric", default="Time", help="Metric column (default: Time)"
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances matching domain substring"
    )
    parser.add_argument(
        "--solved", action="store_true", help="Keep only instances solved by all runs"
    )
    parser.add_argument(
        "--out", default="bar_plot.pdf", help="Output file (default: bar_plot.pdf)"
    )
    args = parser.parse_args()

    if len(args.files) < 2:
        parser.error("At least two files required (baseline + one comparison).")

    aliases = resolve_aliases(args.files, args.aliases)
    default_cols = {"Nodes", "Time"}
    extra = [args.metric] if args.metric not in default_cols else None
    data = prepare_data(
        args.files,
        aliases,
        extra_cols=extra,
        domain=args.domain,
        solved_only=args.solved,
    )

    plot = bar_comparison_plot(data, aliases, metric=args.metric)
    save(plot, args.out, width=10, height=5)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
