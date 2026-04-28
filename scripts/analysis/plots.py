#!/usr/bin/env python

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import polars as pl
from plotnine import (
    ggplot,
    aes,
    geom_abline,
    geom_bar,
    geom_boxplot,
    geom_hline,
    geom_jitter,
    geom_point,
    geom_rect,
    geom_step,
    scale_color_brewer,
    scale_fill_brewer,
    scale_x_log10,
    scale_y_log10,
    facet_wrap,
    labs,
    theme_minimal,
    theme,
    element_blank,
    element_line,
    element_text,
)

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import compare_data, _GROUP_ORDER, _sgm

# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------


def save(plot, path: str = "plot.pdf", width: float = 6, height: float = 6) -> None:
    fig = plot.draw()
    fig.set_size_inches(width, height)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Cumulative / cactus plot
# ---------------------------------------------------------------------------


def cumulative_plot(
    df: pl.DataFrame,
    x_col: str = "Time",
    status_col: str = "Solved",
    group_col: str = "Model",
    x_limit: float | None = None,
    title: str = "",
    x_label: str = "",
    y_label: str = "Instances Solved",
    log_x: bool = True,
    group_order: list[str] | None = None,
) -> ggplot:
    solved = df.filter(pl.col(status_col))
    x_min = float(solved[x_col].min()) * 0.5 if len(solved) > 0 else 1e-4
    x_end = x_limit or float(df[x_col].max())
    groups = group_order or df[group_col].unique().sort().to_list()

    rows: list[dict] = []
    for group in groups:
        times = solved.filter(pl.col(group_col) == group)[x_col].sort().to_list()
        n = len(times)
        xs = [x_min] + times + [x_end]
        ys = [0] + list(range(1, n + 1)) + [n]
        rows.extend({x_col: x, "y": y, group_col: group} for x, y in zip(xs, ys))

    plot_df = pl.DataFrame(rows)
    if group_order:
        plot_df = plot_df.with_columns(pl.col(group_col).cast(pl.Enum(group_order)))

    plot = (
        ggplot(plot_df, aes(x=x_col, y="y", color=group_col))
        + geom_step(direction="hv", size=0.9, na_rm=True)
        + scale_color_brewer(type="qual", palette="Set1")
        + labs(title=title, x=x_label or x_col, y=y_label)
        + theme_minimal()
        + theme(
            plot_title=element_text(size=14, face="bold"),
            axis_title=element_text(size=11),
            legend_position="bottom",
        )
    )
    if log_x:
        plot += scale_x_log10()
    return plot


# ---------------------------------------------------------------------------
# Boxplot
# ---------------------------------------------------------------------------


def boxplot(
    df: pl.DataFrame,
    group_col: str,
    value_col: str = "Value",
    title: str = "",
    x_label: str = "",
    y_label: str = "",
    show_points: bool = False,
    group_order: list[str] | None = None,
) -> ggplot:
    df = df.select([group_col, value_col])
    if group_order is not None:
        df = df.with_columns(
            pl.col(group_col).cast(pl.String).cast(pl.Enum(group_order))
        )
    else:
        df = df.with_columns(pl.col(group_col).cast(pl.String))

    n_groups = df[group_col].n_unique()
    plot = (
        ggplot(df, aes(x=group_col, y=value_col, fill=group_col))
        + geom_boxplot(outlier_shape="o", outlier_size=1.5, width=0.5, alpha=0.85)
        + scale_fill_brewer(type="qual", palette="Set2")
        + labs(title=title, x=x_label or group_col, y=y_label or value_col)
        + theme_minimal()
        + theme(
            figure_size=(max(6, n_groups * 0.9), 5),
            axis_text_x=element_text(angle=45, hjust=1, size=9),
            axis_line=element_line(color="#cccccc"),
            legend_position="none",
            plot_title=element_text(size=12, face="bold"),
        )
    )
    if show_points:
        plot += geom_jitter(width=0.15, size=1.2, alpha=0.5, color="#444444")
    return plot


# ---------------------------------------------------------------------------
# Bar plot (SGM ratios)
# ---------------------------------------------------------------------------


def bar_plot(
    df: pl.DataFrame,
    x_col: str,
    y_col: str,
    fill_col: str,
    title: str = "",
    x_label: str = "",
    y_label: str = "",
    x_order: list[str] | None = None,
    fill_order: list[str] | None = None,
    reference_y: float | None = None,
) -> ggplot:
    plot_df = df.select([x_col, y_col, fill_col])
    if x_order:
        plot_df = plot_df.with_columns(pl.col(x_col).cast(pl.Enum(x_order)))
    if fill_order:
        plot_df = plot_df.with_columns(pl.col(fill_col).cast(pl.Enum(fill_order)))

    plot = (
        ggplot(plot_df, aes(x=x_col, y=y_col, fill=fill_col))
        + geom_bar(stat="identity", position="dodge", width=0.7)
        + scale_fill_brewer(type="qual", palette="Set2")
        + labs(title=title, x=x_label or x_col, y=y_label or y_col)
        + theme_minimal()
        + theme(
            plot_title=element_text(size=14, face="bold"),
            axis_title=element_text(size=11, face="bold"),
            axis_text_x=element_text(angle=45, hjust=1),
            panel_grid_major_x=element_blank(),
            panel_grid_minor=element_blank(),
        )
    )
    if reference_y is not None:
        plot += geom_hline(
            yintercept=reference_y, color="red", size=0.8, alpha=0.5, linetype="dashed"
        )
    return plot


# ---------------------------------------------------------------------------
# Scatter comparison plot
# ---------------------------------------------------------------------------


def scatter_comparison_plot(
    data: pl.DataFrame,
    models: list[str],
    metric: str = "Time",
    highlight: pl.DataFrame | None = None,
) -> ggplot:
    """Log-log scatter: each non-baseline model vs baseline, one facet per model.

    Reference lines: y=x (black), y=2x / y=0.5x (orange), y=10x / y=0.1x (red).
    If highlight is given, full data shown as grey; highlight in colour.
    """
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
            strip_text=element_text(size=10, weight="bold"),
            plot_title=element_text(size=14, weight="bold"),
            axis_title=element_text(size=11),
        )
    )


# ---------------------------------------------------------------------------
# Bar comparison plot (SGM ratios by group)
# ---------------------------------------------------------------------------


def bar_comparison_plot(
    data: pl.DataFrame,
    models: list[str],
    metric: str = "Time",
) -> ggplot:
    """Grouped bar chart of SGM ratios vs baseline, by time bracket / category."""
    cmp = compare_data(data, metric, models)
    base = models[0]
    non_base = models[1:]

    present = set(cmp["Group"].unique().to_list())
    group_order = [
        g
        for g in _GROUP_ORDER
        if g in present and g not in ("[900,+inf)", "non-solvable")
    ]

    plot_df = cmp.filter((pl.col("Model") != base) & (pl.col("Group") != "[900,+inf)"))

    return bar_plot(
        plot_df,
        x_col="Group",
        y_col="sgm_ratio",
        fill_col="Model",
        title=f"{metric} SGM Ratio vs {base}",
        x_label="Category / Time Bracket",
        y_label=f"{metric} Ratio",
        x_order=group_order,
        fill_order=non_base,
        reference_y=1.0,
    )


# ---------------------------------------------------------------------------
# Window histogram plots
# ---------------------------------------------------------------------------


def _window_rows(
    df: pl.DataFrame,
    x_col: str,
    status_col: str,
    group_col: str,
    groups: list[str],
    edges: np.ndarray,
    n_windows: int,
    bin_fill: float,
    bar_fill: float,
    dodge: bool,
) -> pl.DataFrame:
    n_models = len(groups)
    rows: list[dict] = []
    for model_idx, group in enumerate(groups):
        times = df.filter((pl.col(group_col) == group) & pl.col(status_col))[
            x_col
        ].to_numpy()
        counts, _ = np.histogram(times, bins=edges)
        for i in range(n_windows):
            log_lo, log_hi = np.log10(edges[i]), np.log10(edges[i + 1])
            log_total = log_hi - log_lo
            half_gap_bin = log_total * (1.0 - bin_fill) / 2.0
            bin_start, bin_end = log_lo + half_gap_bin, log_hi - half_gap_bin
            if dodge:
                slot = (bin_end - bin_start) / n_models
                half_gap_bar = slot * (1.0 - bar_fill) / 2.0
                xmin = 10 ** (bin_start + model_idx * slot + half_gap_bar)
                xmax = 10 ** (bin_start + (model_idx + 1) * slot - half_gap_bar)
            else:
                half_gap_bar = (bin_end - bin_start) * (1.0 - bar_fill) / 2.0
                xmin = 10 ** (bin_start + half_gap_bar)
                xmax = 10 ** (bin_end - half_gap_bar)
            rows.append(
                {
                    "xmin": xmin,
                    "xmax": xmax,
                    "ymin": 0,
                    "ymax": int(counts[i]),
                    group_col: group,
                }
            )
    return pl.DataFrame(rows)


def window_plot(
    df: pl.DataFrame,
    x_col: str = "Time",
    status_col: str = "Solved",
    group_col: str = "Model",
    x_limit: float | None = None,
    n_windows: int = 20,
    title: str = "",
    x_label: str = "",
    y_label: str = "Instances Solved",
    group_order: list[str] | None = None,
) -> ggplot:
    solved = df.filter(pl.col(status_col))
    x_min = max(float(solved[x_col].min()), 1e-4) if len(solved) > 0 else 1e-4
    x_end = x_limit or float(df[x_col].max())
    groups = group_order or df[group_col].unique().sort().to_list()
    edges = np.logspace(np.log10(x_min), np.log10(x_end), n_windows + 1)

    plot_df = _window_rows(
        df,
        x_col,
        status_col,
        group_col,
        groups,
        edges,
        n_windows,
        0.85,
        0.90,
        dodge=True,
    )
    if group_order:
        plot_df = plot_df.with_columns(pl.col(group_col).cast(pl.Enum(group_order)))

    return (
        ggplot(
            plot_df,
            aes(xmin="xmin", xmax="xmax", ymin="ymin", ymax="ymax", fill=group_col),
        )
        + geom_rect(alpha=0.85)
        + scale_x_log10(limits=(1e-4, x_end * 1.1))
        + scale_fill_brewer(type="qual", palette="Set1")
        + labs(title=title, x=x_label or x_col, y=y_label)
        + theme_minimal()
        + theme(
            plot_title=element_text(size=14, face="bold"),
            axis_title=element_text(size=11),
            legend_position="bottom",
        )
    )


def window_facet_plot(
    df: pl.DataFrame,
    x_col: str = "Time",
    status_col: str = "Solved",
    group_col: str = "Model",
    x_limit: float | None = None,
    n_windows: int = 20,
    title: str = "",
    x_label: str = "",
    y_label: str = "Instances Solved",
    group_order: list[str] | None = None,
) -> ggplot:
    solved = df.filter(pl.col(status_col))
    x_min = max(float(solved[x_col].min()), 1e-4) if len(solved) > 0 else 1e-4
    x_end = x_limit or float(df[x_col].max())
    groups = group_order or df[group_col].unique().sort().to_list()
    edges = np.logspace(np.log10(x_min), np.log10(x_end), n_windows + 1)

    plot_df = _window_rows(
        df,
        x_col,
        status_col,
        group_col,
        groups,
        edges,
        n_windows,
        0.85,
        0.90,
        dodge=False,
    )
    if group_order:
        plot_df = plot_df.with_columns(pl.col(group_col).cast(pl.Enum(group_order)))

    return (
        ggplot(
            plot_df,
            aes(xmin="xmin", xmax="xmax", ymin="ymin", ymax="ymax", fill=group_col),
        )
        + geom_rect(alpha=0.9)
        + scale_x_log10(limits=(1e-4, x_end * 1.1))
        + scale_fill_brewer(type="qual", palette="Set1")
        + facet_wrap(group_col, ncol=1, scales="fixed")
        + labs(title=title, x=x_label or x_col, y=y_label)
        + theme_minimal()
        + theme(
            plot_title=element_text(size=14, face="bold"),
            axis_title=element_text(size=11),
            strip_text=element_text(size=9),
            legend_position="none",
        )
    )
