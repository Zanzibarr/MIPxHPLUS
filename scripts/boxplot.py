"""
Script used in pair with parse_results.py to compare bound quality across runs of the 'hplus' solver

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> 4.2.0

Boxplot of the gap (%) between bound columns (--gap) and the best known
incumbent (from results/best_known.csv, not necessarily a proven optimum):
one group of boxes per bound column, one box per model. Instances whose gap
is 0 for every model and bound are dropped.
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import polars as pl
from plotnine import (
    aes,
    element_line,
    element_text,
    geom_boxplot,
    geom_point,
    ggplot,
    labs,
    position_dodge,
    position_jitterdodge,
    scale_fill_brewer,
    theme,
    theme_minimal,
)

from analysis_utils import INFINITY, prepare_data, require_complete, resolve_aliases

BEST_KNOWN = Path(__file__).parent.parent / "results" / "best_known.csv"


def compute_gaps(data: pl.DataFrame, cols: list[str]) -> pl.DataFrame:
    """Gap (%) between each bound column and the best known incumbent, long format.

    Gap = |col - Incumbent| / max(col, Incumbent) * 100 (standard MIP gap,
    always in [0, 100], works for both lower and upper bounds). The incumbent
    need not be a proven optimum; instances with no known solution (placeholder
    incumbent 1e20) are skipped. Returns Problem | Model | Phase | Gap.
    """
    if not BEST_KNOWN.is_file():
        print(f"ERROR: best_known file not found: {BEST_KNOWN}")
        sys.exit(1)
    bk = (
        pl.read_csv(BEST_KNOWN)
        .filter(pl.col("Incumbent") < INFINITY)
        .select("Instance", pl.col("Incumbent").cast(pl.Float64))
        .rename({"Instance": "Problem"})
    )

    data = require_complete(data, cols)
    frames: list[pl.DataFrame] = []
    for col in cols:
        if col not in data.columns:
            print(f"WARNING: column {col!r} not found in data, skipping.")
            continue
        bound = pl.col(col).cast(pl.Float64)
        gap = (
            (bound - pl.col("Incumbent")).abs()
            / pl.max_horizontal(bound, pl.col("Incumbent"))
            * 100
        ).alias("Gap")
        frames.append(
            data.join(bk, on="Problem", how="inner").select(
                "Problem", "Model", pl.lit(col).alias("Phase"), gap
            )
        )
    return pl.concat(frames)


def gap_boxplot(
    df: pl.DataFrame, gap_cols: list[str], models: list[str], show_points: bool
) -> ggplot:
    df = df.select("Phase", "Gap", "Model").with_columns(
        pl.col("Phase").cast(pl.Enum(gap_cols)),
        pl.col("Model").cast(pl.Enum(models)),
    )
    plot = (
        ggplot(df, aes(x="Phase", y="Gap", fill="Model"))
        + geom_boxplot(
            outlier_shape="o",
            outlier_size=1.5,
            width=0.4,
            alpha=0.85,
            position=position_dodge(width=0.9),
        )
        + scale_fill_brewer(type="qual", palette="Set2")
        + labs(title="Optimality Gap by Bound", x="Bound", y="Gap to optimal (%)")
        + theme_minimal()
        + theme(
            figure_size=(max(6, df["Phase"].n_unique() * 0.9), 5),
            axis_text_x=element_text(angle=45, hjust=1, size=9),
            axis_line=element_line(color="#cccccc"),
            legend_position="right",
            plot_title=element_text(size=12, face="bold"),
        )
    )
    if show_points:
        plot += geom_point(
            size=1.2,
            alpha=0.5,
            color="#444444",
            position=position_jitterdodge(jitter_width=0.1, dodge_width=0.9),
        )
    return plot


def save(plot: ggplot, path: str, width: float) -> None:
    fig = plot.draw()
    fig.set_size_inches(width, 5)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Boxplot of optimality gaps vs the best known incumbents."
    )
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--gap",
        nargs="+",
        metavar="COL",
        required=True,
        help="Bound column(s) to compare (e.g. --gap Relax_LB Root_LB Final_LB)",
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--points", action="store_true", help="Overlay individual data points"
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
        "--out", default="boxplot.pdf", help="Output file (default: boxplot.pdf)"
    )
    args = parser.parse_args()

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
    # drop problems whose gap is 0 for all models/phases
    gaps = gaps.filter(pl.col("Gap").fill_null(0).abs().max().over("Problem") > 0)

    plot = gap_boxplot(gaps, args.gap, aliases, args.points)
    save(plot, args.out, width=max(8, len(args.gap) * 1.5 * len(aliases)))
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
