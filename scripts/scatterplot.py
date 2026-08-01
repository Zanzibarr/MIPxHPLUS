"""
Script used in pair with parse_results.py to compare runs of the 'hplus' solver instance by instance

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> 4.2.0

Log-log scatter plot of a metric (default: Time): each non-baseline model vs
the baseline (first file), one facet per model. Reference lines: y=x (black),
y=2x / y=x/2 (orange), y=10x / y=x/10 (red). With --domain, all instances are
drawn in grey and the matching ones are highlighted in colour.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
import polars as pl
from plotnine import (
    aes,
    element_text,
    facet_wrap,
    geom_abline,
    geom_point,
    ggplot,
    labs,
    scale_x_log10,
    scale_y_log10,
    theme,
    theme_minimal,
)

from analysis_utils import prepare_data, resolve_aliases


def scatter_plot(
    data: pl.DataFrame,
    models: list[str],
    metric: str,
    highlight: pl.DataFrame | None,
) -> ggplot:
    """Log-log scatter of metric: each non-baseline model vs the baseline."""
    panel_size = 5.0
    base = models[0]
    base_col = data.filter(pl.col("Model") == base).select(
        ["Problem", pl.col(metric).alias("_base")]
    )

    def _build(src: pl.DataFrame) -> pl.DataFrame:
        return src.filter(pl.col("Model") != base).join(base_col, on="Problem")

    plot_df = _build(data)
    hl_df = _build(highlight) if highlight is not None else plot_df

    grey_layers: list = []
    if highlight is not None:
        grey_layers = [
            geom_point(alpha=0.1, size=0.2, color="grey", na_rm=True),
            geom_point(data=hl_df, color="white", alpha=0.95, size=1, na_rm=True),
        ]

    return (
        ggplot(plot_df, aes(x="_base", y=metric, color="Model"))
        + geom_abline(intercept=0, slope=1, color="black", alpha=0.7, size=0.5)
        + geom_abline(
            intercept=np.log10(2), slope=1, color="orange", alpha=0.6, size=0.5
        )
        + geom_abline(
            intercept=-np.log10(2), slope=1, color="orange", alpha=0.6, size=0.5
        )
        + geom_abline(intercept=1, slope=1, color="red", alpha=0.6, size=0.5)
        + geom_abline(intercept=-1, slope=1, color="red", alpha=0.6, size=0.5)
        + grey_layers
        + geom_point(data=hl_df, alpha=0.5, size=0.4, na_rm=True)
        + scale_x_log10(limits=(1e-4, 1e3))
        + scale_y_log10(limits=(1e-4, 1e3))
        + facet_wrap("Model", scales="fixed")
        + labs(
            title=f"Performance vs {base}", x=f"Baseline {metric}", y=f"Model {metric}"
        )
        + theme_minimal()
        + theme(
            aspect_ratio=1,
            figure_size=(panel_size * (len(models) - 1) + 1.5, panel_size + 1.0),
            strip_text=element_text(size=10, weight="bold"),
            plot_title=element_text(size=14, weight="bold"),
            axis_title=element_text(size=11),
        )
    )


def save(plot: ggplot, path: str) -> None:
    fig = plot.draw()
    fig.set_size_inches(6, 6)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Log-log scatter: runs vs baseline.")
    parser.add_argument("files", nargs="+", help="CSV run files (first = baseline)")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--metric", default="Time", help="Metric column (default: Time)"
    )
    parser.add_argument(
        "--domain", default="", help="Highlight instances matching domain substring"
    )
    parser.add_argument(
        "--out", default="scatter.pdf", help="Output file (default: scatter.pdf)"
    )
    args = parser.parse_args()

    if len(args.files) < 2:
        parser.error("At least two files required (baseline + one comparison).")

    aliases = resolve_aliases(args.files, args.aliases)
    extra = [args.metric] if args.metric not in ("Nodes", "Time") else None
    data = prepare_data(args.files, aliases, extra_cols=extra)
    highlight = (
        data.filter(pl.col("Problem").str.contains(args.domain))
        if args.domain
        else None
    )

    plot = scatter_plot(data, aliases, args.metric, highlight)
    save(plot, args.out)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
