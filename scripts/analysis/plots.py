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
    geom_violin,
    geom_hline,
    geom_point,
    geom_polygon,
    geom_segment,
    position_dodge,
    position_jitterdodge,
    geom_rect,
    geom_step,
    scale_color_brewer,
    scale_fill_brewer,
    scale_fill_manual,
    scale_x_continuous,
    coord_flip,
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


def save(
    plot, path: str = "plot.pdf", width: float | None = 6, height: float | None = 6
) -> None:
    fig = plot.draw()
    if width is not None and height is not None:
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
    fill_col: str | None = None,
    title: str = "",
    x_label: str = "",
    y_label: str = "",
    show_points: bool = False,
    group_order: list[str] | None = None,
    fill_order: list[str] | None = None,
) -> ggplot:
    actual_fill = fill_col or group_col
    cols = [group_col, value_col] + (
        [fill_col] if fill_col and fill_col != group_col else []
    )
    df = df.select(cols)

    if group_order is not None:
        df = df.with_columns(
            pl.col(group_col).cast(pl.String).cast(pl.Enum(group_order))
        )
    else:
        df = df.with_columns(pl.col(group_col).cast(pl.String))

    if fill_col is not None and fill_col != group_col:
        if fill_order is not None:
            df = df.with_columns(
                pl.col(fill_col).cast(pl.String).cast(pl.Enum(fill_order))
            )
        else:
            df = df.with_columns(pl.col(fill_col).cast(pl.String))

    n_groups = df[group_col].n_unique()
    plot = (
        ggplot(df, aes(x=group_col, y=value_col, fill=actual_fill))
        + geom_boxplot(
            outlier_shape="o",
            outlier_size=1.5,
            width=0.4,
            alpha=0.85,
            position=position_dodge(width=0.9),
        )
        + scale_fill_brewer(type="qual", palette="Set2")
        + labs(title=title, x=x_label or group_col, y=y_label or value_col)
        + theme_minimal()
        + theme(
            figure_size=(max(6, n_groups * 0.9), 5),
            axis_text_x=element_text(angle=45, hjust=1, size=9),
            axis_line=element_line(color="#cccccc"),
            legend_position="right" if fill_col else "none",
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


# ---------------------------------------------------------------------------
# Violin plot
# ---------------------------------------------------------------------------


def violin_plot(
    df: pl.DataFrame,
    group_col: str,
    value_col: str = "Value",
    fill_col: str | None = None,
    title: str = "",
    x_label: str = "",
    y_label: str = "",
    show_points: bool = False,
    group_order: list[str] | None = None,
    fill_order: list[str] | None = None,
    reference_y: float | None = None,
    global_scale: bool = False,
) -> ggplot:
    actual_fill = fill_col or group_col

    if global_scale:
        from scipy.stats import gaussian_kde

        dodged = fill_col is not None and fill_col != group_col
        groups_list = group_order or df[group_col].cast(pl.String).unique().sort().to_list()
        fills_list = (
            fill_order or df[fill_col].cast(pl.String).unique().sort().to_list()
        ) if dodged else groups_list
        n_groups = len(groups_list)
        n_fills = len(fills_list) if dodged else 1

        dodge_w = 0.9
        violin_slot = dodge_w / n_fills if dodged else dodge_w
        half_w = violin_slot * 0.44
        box_w = half_w * 0.08

        # first pass: compute all KDEs, find global max
        kde_data: dict = {}
        global_max = 0.0
        for group in groups_list:
            for fill in (fills_list if dodged else [group]):
                mask = pl.col(group_col).cast(pl.String) == group
                if dodged:
                    mask = mask & (pl.col(fill_col).cast(pl.String) == fill)
                vals = df.filter(mask)[value_col].drop_nulls().to_numpy()
                vals = vals[np.isfinite(vals)]
                if len(vals) < 2:
                    continue
                kde = gaussian_kde(vals, bw_method="scott")
                margin = max((vals.max() - vals.min()) * 0.1, 1e-6)
                y_pts = np.linspace(vals.min(), vals.max() + margin, 300)
                density = kde(y_pts)
                global_max = max(global_max, density.max())
                kde_data[(group, fill)] = (vals, y_pts, density)

        # second pass: build polygon + boxplot data
        poly_rows: list[dict] = []
        box_rows: list[dict] = []
        median_rows: list[dict] = []
        whisker_rows: list[dict] = []
        point_rows: list[dict] = []

        for gi, group in enumerate(groups_list):
            for fi, fill in enumerate(fills_list if dodged else [group]):
                key = (group, fill)
                if key not in kde_data:
                    continue
                cx = float(gi) + (fi - (n_fills - 1) / 2.0) * violin_slot if dodged else float(gi)
                vals, y_pts, density = kde_data[key]
                norm_density = density / global_max * half_w
                px = np.concatenate([cx - norm_density, (cx + norm_density)[::-1]])
                py = np.concatenate([y_pts, y_pts[::-1]])
                pid = f"{group}_{fill}"
                for x, y in zip(px, py):
                    poly_rows.append({"x": x, "y": y, "grp": pid, actual_fill: fill})

                q25, q50, q75 = np.percentile(vals, [25, 50, 75])
                iqr = q75 - q25
                w_lo = float(max(vals.min(), q25 - 1.5 * iqr))
                w_hi = float(min(vals.max(), q75 + 1.5 * iqr))
                box_rows.append({"xmin": cx - box_w, "xmax": cx + box_w, "ymin": float(q25), "ymax": float(q75)})
                median_rows.append({"x": cx - box_w, "xend": cx + box_w, "y": float(q50), "yend": float(q50)})
                whisker_rows.append({"x": cx, "xend": cx, "y": w_lo, "yend": float(q25)})
                whisker_rows.append({"x": cx, "xend": cx, "y": float(q75), "yend": w_hi})

                if show_points:
                    rng = np.random.default_rng()
                    jitter = rng.uniform(-half_w * 0.2, half_w * 0.2, len(vals))
                    for v, j in zip(vals, jitter):
                        point_rows.append({"x": cx + j, "y": float(v), actual_fill: fill})

        poly_df = pl.DataFrame(poly_rows).with_columns(pl.col(actual_fill).cast(pl.Enum(fills_list)))
        box_df = pl.DataFrame(box_rows)
        median_df = pl.DataFrame(median_rows)
        whisker_df = pl.DataFrame(whisker_rows)

        plot = (
            ggplot(poly_df, aes(x="x", y="y", fill=actual_fill, group="grp"))
            + geom_polygon(alpha=0.85, color="black", size=0.3)
            + geom_segment(
                data=whisker_df,
                mapping=aes(x="x", xend="xend", y="y", yend="yend"),
                color="black", size=0.4, inherit_aes=False,
            )
            + geom_rect(
                data=box_df,
                mapping=aes(xmin="xmin", xmax="xmax", ymin="ymin", ymax="ymax"),
                fill="white", color="black", size=0.4, inherit_aes=False,
            )
            + geom_segment(
                data=median_df,
                mapping=aes(x="x", xend="xend", y="y", yend="yend"),
                color="black", size=0.8, inherit_aes=False,
            )
            + scale_x_continuous(breaks=list(range(n_groups)), labels=groups_list)
            + scale_fill_brewer(type="qual", palette="Set2")
            + labs(title=title, x=x_label or group_col, y=y_label or value_col)
            + theme_minimal()
            + theme(
                figure_size=(max(6, n_groups * 0.9), 5),
                axis_text_x=element_text(angle=45, hjust=1, size=9),
                axis_line=element_line(color="#cccccc"),
                legend_position="right" if fill_col else "none",
                plot_title=element_text(size=12, face="bold"),
            )
        )
        if show_points and point_rows:
            plot += geom_point(
                data=pl.DataFrame(point_rows),
                mapping=aes(x="x", y="y", color=actual_fill),
                size=1.0, alpha=0.35, inherit_aes=False,
            )
        if reference_y is not None:
            plot += geom_hline(
                yintercept=reference_y, color="red", size=0.8, alpha=0.6, linetype="dashed"
            )
        return plot

    cols = [group_col, value_col] + (
        [fill_col] if fill_col and fill_col != group_col else []
    )
    df = df.select(cols)

    if group_order is not None:
        df = df.with_columns(
            pl.col(group_col).cast(pl.String).cast(pl.Enum(group_order))
        )
    else:
        df = df.with_columns(pl.col(group_col).cast(pl.String))

    if fill_col is not None and fill_col != group_col:
        if fill_order is not None:
            df = df.with_columns(
                pl.col(fill_col).cast(pl.String).cast(pl.Enum(fill_order))
            )
        else:
            df = df.with_columns(pl.col(fill_col).cast(pl.String))

    n_groups = df[group_col].n_unique()
    plot = (
        ggplot(df, aes(x=group_col, y=value_col, fill=actual_fill))
        + geom_violin(
            scale="width",
            width=0.6,
            alpha=0.85,
            position=position_dodge(width=0.9),
        )
        + geom_boxplot(
            mapping=aes(group=actual_fill),
            width=0.05,
            outlier_shape=None,
            fill="white",
            alpha=0.8,
            position=position_dodge(width=0.9),
        )
        + scale_fill_brewer(type="qual", palette="Set2")
        + labs(title=title, x=x_label or group_col, y=y_label or value_col)
        + theme_minimal()
        + theme(
            figure_size=(max(6, n_groups * 0.9), 5),
            axis_text_x=element_text(angle=45, hjust=1, size=9),
            axis_line=element_line(color="#cccccc"),
            legend_position="right" if fill_col else "none",
            plot_title=element_text(size=12, face="bold"),
        )
    )
    if reference_y is not None:
        plot += geom_hline(
            yintercept=reference_y, color="red", size=0.8, alpha=0.6, linetype="dashed"
        )
    if show_points:
        plot += geom_point(
            size=1.2,
            alpha=0.5,
            color="#444444",
            position=position_jitterdodge(jitter_width=0.1, dodge_width=0.9),
        )
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
    n_comp = len(models) - 1
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
            figure_size=(panel_size * n_comp + 1.5, panel_size + 1.0),
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


_SET2 = [
    "#66C2A5", "#FC8D62", "#8DA0CB", "#E78AC3",
    "#A6D854", "#FFD92F", "#E5C494", "#B3B3B3",
]


def split_violin_plot(
    df: pl.DataFrame,
    pairs: list[tuple[str, str]],
    group_col: str = "Phase",
    value_col: str = "Gap",
    group_order: list[str] | None = None,
    title: str = "",
    x_label: str = "",
    y_label: str = "",
    show_points: bool = False,
    reference_y: float | None = None,
    global_scale: bool = False,
) -> ggplot:
    """Split violin via geom_polygon — inherits theme_minimal style automatically."""
    from scipy.stats import gaussian_kde

    groups = group_order or sorted(df[group_col].unique().to_list())
    n_groups = len(groups)
    n_pairs = len(pairs)

    all_aliases: list[str] = []
    for left, right in pairs:
        for a in (left, right):
            if a not in all_aliases:
                all_aliases.append(a)
    color_map = {a: _SET2[i % len(_SET2)] for i, a in enumerate(all_aliases)}

    total_width = 0.8
    pair_width = total_width / n_pairs
    half_w = pair_width * 0.44
    box_w = half_w * 0.08

    # first pass: compute all KDEs and global max
    all_kde: dict = {}
    global_max = 0.0
    for gi, group in enumerate(groups):
        gdf = df.filter(pl.col(group_col) == group)
        for pi, (left_alias, right_alias) in enumerate(pairs):
            pair_max = 0.0
            for alias, sign in ((left_alias, -1), (right_alias, +1)):
                vals = gdf.filter(pl.col("Model") == alias)[value_col].drop_nulls().to_numpy()
                vals = vals[np.isfinite(vals)]
                if len(vals) < 2:
                    continue
                kde = gaussian_kde(vals, bw_method="scott")
                margin = max((vals.max() - vals.min()) * 0.1, 1e-6)
                y_pts = np.linspace(vals.min(), vals.max() + margin, 300)
                density = kde(y_pts)
                pair_max = max(pair_max, density.max())
                global_max = max(global_max, density.max())
                all_kde[(gi, pi, alias)] = (sign, vals, y_pts, density, pair_max)

    poly_rows: list[dict] = []
    box_rows: list[dict] = []
    median_rows: list[dict] = []
    whisker_rows: list[dict] = []
    spine_rows: list[dict] = []
    point_rows: list[dict] = []

    for gi, group in enumerate(groups):
        for pi, (left_alias, right_alias) in enumerate(pairs):
            cx = float(gi) + (pi - (n_pairs - 1) / 2.0) * pair_width

            pair_entries = {
                alias: all_kde[(gi, pi, alias)]
                for alias in (left_alias, right_alias)
                if (gi, pi, alias) in all_kde
            }
            if not pair_entries:
                continue

            pair_max = max(v[3].max() for v in pair_entries.values())
            normaliser = global_max if global_scale else pair_max

            for alias, (sign, vals, y_pts, density, _) in pair_entries.items():
                norm_density = density / normaliser * half_w
                edge_x = cx + sign * norm_density
                px = np.concatenate([[cx], edge_x, [cx]])
                py = np.concatenate([[y_pts[0]], y_pts, [y_pts[-1]]])
                pid = f"{group}_{pi}_{alias}"
                for x, y in zip(px, py):
                    poly_rows.append({"x": x, "y": y, "grp": pid, "Model": alias})

                q25, q50, q75 = np.percentile(vals, [25, 50, 75])
                iqr = q75 - q25
                w_lo = float(max(vals.min(), q25 - 1.5 * iqr))
                w_hi = float(min(vals.max(), q75 + 1.5 * iqr))
                xlo, xhi = min(cx, cx + sign * box_w), max(cx, cx + sign * box_w)
                box_rows.append({"xmin": xlo, "xmax": xhi, "ymin": float(q25), "ymax": float(q75)})
                median_rows.append({"x": xlo, "xend": xhi, "y": float(q50), "yend": float(q50)})
                wx = cx + sign * box_w * 0.5
                whisker_rows.append({"x": wx, "xend": wx, "y": w_lo, "yend": float(q25)})
                whisker_rows.append({"x": wx, "xend": wx, "y": float(q75), "yend": w_hi})

            all_y = np.concatenate([e[2] for e in pair_entries.values()])
            spine_rows.append({"x": cx, "xend": cx, "y": float(all_y.min()), "yend": float(all_y.max())})

            if show_points:
                rng = np.random.default_rng()
                for alias, (sign, vals, _, __, ___) in pair_entries.items():
                    jitter = rng.uniform(0, half_w * 0.35, len(vals))
                    for v, j in zip(vals, jitter):
                        point_rows.append({"x": cx + sign * j, "y": float(v), "Model": alias})

    poly_df = pl.DataFrame(poly_rows).with_columns(pl.col("Model").cast(pl.Enum(all_aliases)))
    box_df = pl.DataFrame(box_rows)
    median_df = pl.DataFrame(median_rows)
    whisker_df = pl.DataFrame(whisker_rows)
    spine_df = pl.DataFrame(spine_rows)

    plot = (
        ggplot(poly_df, aes(x="x", y="y", fill="Model", group="grp"))
        + geom_polygon(alpha=0.82, color="black", size=0.3)
        + geom_segment(
            data=spine_df,
            mapping=aes(x="x", xend="xend", y="y", yend="yend"),
            color="#555555", size=0.4, alpha=0.5, inherit_aes=False,
        )
        + geom_segment(
            data=whisker_df,
            mapping=aes(x="x", xend="xend", y="y", yend="yend"),
            color="black", size=0.4, inherit_aes=False,
        )
        + geom_rect(
            data=box_df,
            mapping=aes(xmin="xmin", xmax="xmax", ymin="ymin", ymax="ymax"),
            fill="white", color="black", size=0.4, inherit_aes=False,
        )
        + geom_segment(
            data=median_df,
            mapping=aes(x="x", xend="xend", y="y", yend="yend"),
            color="black", size=0.8, inherit_aes=False,
        )
        + scale_x_continuous(breaks=list(range(n_groups)), labels=groups)
        + scale_fill_manual(values=color_map)
        + labs(title=title, x=x_label or group_col, y=y_label or value_col)
        + theme_minimal()
        + theme(
            figure_size=(max(6, n_groups * 0.9), 5),
            axis_text_x=element_text(angle=45, hjust=1, size=9),
            axis_line=element_line(color="#cccccc"),
            legend_position="right",
            plot_title=element_text(size=12, face="bold"),
        )
    )

    if show_points and point_rows:
        plot += geom_point(
            data=pl.DataFrame(point_rows),
            mapping=aes(x="x", y="y", color="Model"),
            size=1.0, alpha=0.35, inherit_aes=False,
        )

    if reference_y is not None:
        plot += geom_hline(
            yintercept=reference_y, color="red", size=0.8, alpha=0.6, linetype="dashed"
        )

    return plot


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
