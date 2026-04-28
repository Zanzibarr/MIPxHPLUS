#!/usr/bin/env python
#
# Boxplot of a metric column across runs.
#
# Usage:
#   boxplot.py [options] FILE1 FILE2 [...]

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import (
    prepare_data,
    resolve_aliases,
    compute_optimality_gap,
    _DEFAULT_BEST_KNOWN,
)
from plots import boxplot, save


def main() -> None:
    parser = argparse.ArgumentParser(description="Boxplot of a metric across runs.")
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--metric", default="Time", help="Metric column (default: Time)"
    )
    parser.add_argument(
        "--gap",
        metavar="LB_COL",
        default=None,
        help="Plot optimality gap (%%) for the given lower-bound column (e.g. Relax_LB, Root_LB)",
    )
    parser.add_argument(
        "--best-known",
        default=_DEFAULT_BEST_KNOWN,
        help="Path to best_known.csv (default: results/best_known.csv in repo root)",
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--points", action="store_true", help="Overlay individual data points"
    )
    parser.add_argument(
        "--out", default="boxplot.pdf", help="Output file (default: boxplot.pdf)"
    )
    args = parser.parse_args()

    if args.gap and args.metric != "Time":
        parser.error("--gap and --metric are mutually exclusive.")

    aliases = resolve_aliases(args.files, args.aliases)

    if args.gap:
        data = prepare_data(
            args.files, aliases, domain=args.domain, extra_cols=[args.gap]
        )
        data = compute_optimality_gap(data, args.gap, args.best_known)
        value_col = "Gap"
        title = f"Optimality Gap — {args.gap} (%)"
        y_label = "Gap to optimal (%)"
    else:
        extra = [args.metric] if args.metric not in ("Time", "Nodes") else None
        data = prepare_data(args.files, aliases, domain=args.domain, extra_cols=extra)
        value_col = args.metric
        title = f"{args.metric} Distribution"
        y_label = args.metric

    plot = boxplot(
        data,
        group_col="Model",
        value_col=value_col,
        title=title,
        x_label="Run",
        y_label=y_label,
        show_points=args.points,
        group_order=aliases,
    )
    save(plot, args.out, width=max(6, len(aliases) * 1.5), height=5)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
