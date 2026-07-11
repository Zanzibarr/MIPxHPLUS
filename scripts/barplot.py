"""
Script used in pair with parse_results.py to compare runs of the 'hplus' solver group by group

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0

Grouped bar chart of SGM ratios of a metric (default: Time) vs the baseline
(first file): one group of bars per category / time bracket (the [900,+inf)
bracket is omitted), one bar per non-baseline model. Red dashed line at
ratio 1 = baseline performance.
"""

import argparse

import matplotlib.pyplot as plt
import polars as pl
from plotnine import (
    aes,
    element_blank,
    element_text,
    geom_bar,
    geom_hline,
    ggplot,
    labs,
    scale_fill_brewer,
    theme,
    theme_minimal,
)

from analysis_utils import (
    GROUP_ORDER,
    prepare_data,
    require_complete,
    resolve_aliases,
    sgm,
)


def sgm_ratios(
    data: pl.DataFrame, models: list[str], metric: str
) -> tuple[pl.DataFrame, list[str]]:
    """SGM ratio vs the baseline of every other model, per category and bracket.

    Returns (Group | Model | Ratio rows, group order for the x axis).
    """
    base = models[0]
    present = set(data["Time_Bracket"].unique().to_list())
    groups = [
        ("all", data),
        ("solvable", data.filter(pl.col("Category") != "non-solvable")),
        ("all-solvable", data.filter(pl.col("Category") == "all-solvable")),
    ] + [
        (b, data.filter(pl.col("Time_Bracket") == b))
        for b in GROUP_ORDER[3:-1]  # skip the [900,+inf) bracket
        if b in present
    ]

    rows = []
    for group, sub in groups:
        arrs = {
            m: sub.filter(pl.col("Model") == m)[metric].cast(pl.Float64).to_numpy()
            for m in models
        }
        base_sv = sgm(arrs[base])
        for model in models[1:]:
            sv = sgm(arrs[model])
            ratio = sv / base_sv if base_sv != 0 else float("nan")
            rows.append({"Group": group, "Model": model, "Ratio": ratio})

    return pl.DataFrame(rows), [g for g, _ in groups]


def bar_plot(
    plot_df: pl.DataFrame, group_order: list[str], models: list[str], metric: str
) -> ggplot:
    plot_df = plot_df.with_columns(
        pl.col("Group").cast(pl.Enum(group_order)),
        pl.col("Model").cast(pl.Enum(models[1:])),
    )
    return (
        ggplot(plot_df, aes(x="Group", y="Ratio", fill="Model"))
        + geom_bar(stat="identity", position="dodge", width=0.7)
        + scale_fill_brewer(type="qual", palette="Set2")
        + labs(
            title=f"{metric} SGM Ratio vs {models[0]}",
            x="Category / Time Bracket",
            y=f"{metric} Ratio",
        )
        + theme_minimal()
        + theme(
            plot_title=element_text(size=14, face="bold"),
            axis_title=element_text(size=11, face="bold"),
            axis_text_x=element_text(angle=45, hjust=1),
            panel_grid_major_x=element_blank(),
            panel_grid_minor=element_blank(),
        )
        + geom_hline(
            yintercept=1.0, color="red", size=0.8, alpha=0.5, linetype="dashed"
        )
    )


def save(plot: ggplot, path: str) -> None:
    fig = plot.draw()
    fig.set_size_inches(10, 5)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


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
        "--out", default="bar_plot.pdf", help="Output file (default: bar_plot.pdf)"
    )
    args = parser.parse_args()

    if len(args.files) < 2:
        parser.error("At least two files required (baseline + one comparison).")

    aliases = resolve_aliases(args.files, args.aliases)
    extra = [args.metric] if args.metric not in ("Nodes", "Time") else None
    data = prepare_data(args.files, aliases, domain=args.domain, extra_cols=extra)
    data = require_complete(data, [args.metric])

    plot_df, group_order = sgm_ratios(data, aliases, args.metric)
    plot = bar_plot(plot_df, group_order, aliases, args.metric)
    save(plot, args.out)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
