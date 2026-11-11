"""
Script used in pair with parse_results.py to compare bound quality across runs of the 'hplus' solver

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> current

Prints the statistics of the boxes drawn by boxplot.py, on the same data:
quartiles, mean, whiskers (most extreme points within 1.5 IQR of the box) and
number of outliers of the gap (%) between each bound column (--gap) and the
best known incumbent, one line per bound column and model.
"""

import argparse

import numpy as np
import polars as pl

from analysis_utils import (
    add_time_limit_arg,
    prepare_data,
    resolve_aliases,
    set_time_limit,
)
from boxplot import compute_gaps


def print_stats(df: pl.DataFrame, gap_cols: list[str], models: list[str]) -> None:
    """One line per (bound column, model); non-finite gaps are dropped, as plotnine does."""
    headers = ["Bound", "Model", "N", "Q1", "Median", "Q3", "Mean", "W_low", "W_high", "Outliers"]
    rows = []
    for col in gap_cols:
        for m in models:
            x = df.filter((pl.col("Phase") == col) & (pl.col("Model") == m))["Gap"]
            x = x.cast(pl.Float64).to_numpy()
            x = x[np.isfinite(x)]
            if len(x) == 0:
                continue
            q1, med, q3 = np.percentile(x, [25, 50, 75])
            iqr = q3 - q1
            lo, hi = x[x >= q1 - 1.5 * iqr].min(), x[x <= q3 + 1.5 * iqr].max()
            n_out = int(((x < lo) | (x > hi)).sum())
            rows.append(
                [col, m, str(len(x))]
                + [f"{v:.2f}" for v in (q1, med, q3, x.mean(), lo, hi)]
                + [str(n_out)]
            )
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    print("  ".join(h.rjust(w) for h, w in zip(headers, widths)))
    print("  ".join("-" * w for w in widths))
    for r in rows:
        print("  ".join(v.rjust(w) for v, w in zip(r, widths)))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Statistics of the gap boxplots drawn by boxplot.py."
    )
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--gap",
        nargs="+",
        required=True,
        help="Bound column(s) to compare (e.g. --gap Relax_LB Root_LB Final_LB)",
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--solved", action="store_true", help="Keep only instances solved by all runs"
    )
    parser.add_argument(
        "--time",
        type=float,
        default=None,
        metavar="T",
        help="Exclude instances where any run exceeded T seconds",
    )
    parser.add_argument(
        "--keep-zero",
        action="store_true",
        help="Keep problems whose gap is 0 for all models and bounds",
    )
    add_time_limit_arg(parser)
    args = parser.parse_args()

    set_time_limit(args.time_limit, cap_unsolved=not args.raw_time)
    aliases = resolve_aliases(args.files, args.aliases)
    data = prepare_data(
        args.files,
        aliases,
        domain=args.domain,
        extra_cols=args.gap,
        max_time=args.time,
    )
    if args.solved:
        data = data.filter(pl.col("Category") == "all-solvable")

    gaps = compute_gaps(data, args.gap)
    # drop problems whose gap is 0 for all models/phases, as boxplot.py does
    if not args.keep_zero:
        gaps = gaps.filter(pl.col("Gap").fill_null(0).abs().max().over("Problem") > 0)
    print(f"Problems: {gaps['Problem'].n_unique()}")
    print_stats(gaps, args.gap, aliases)


if __name__ == "__main__":
    main()
