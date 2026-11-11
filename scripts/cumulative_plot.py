"""
Script used in pair with parse_results.py to compare runs of the 'hplus' solver over time

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> current

Cactus plot: cumulative number of instances solved (optimality or infeasibility
proved) within each time t, one step line per model, log time axis up to the
time limit. Only instances present in all runs are considered.
"""

import argparse
import math
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import polars as pl
from plotnine import (
    aes,
    annotation_logticks,
    element_blank,
    element_line,
    element_rect,
    element_text,
    geom_point,
    geom_step,
    ggplot,
    labs,
    scale_color_brewer,
    scale_linetype_manual,
    scale_shape_manual,
    scale_x_log10,
    scale_y_continuous,
    theme,
    theme_bw,
)

import analysis_utils
from analysis_utils import (
    add_time_limit_arg,
    prepare_data,
    resolve_aliases,
    set_time_limit,
)

BASE_PREAMBLE: str = (
    r"\usepackage[T1]{fontenc}\usepackage{lmodern}\usepackage{amsmath,amssymb}"
)

# paper style: text typeset by LaTeX with lmodern, as in the paper source
plt.rcParams.update(
    {
        "text.usetex": True,
        "text.latex.preamble": BASE_PREAMBLE,
        "font.family": "serif",
        "pdf.fonttype": 42,
    }
)
FONT_SIZE: int = 10
FIG_SIZE: tuple[float, float] = (6.0, 6.4)  # square panel spanning the width
LINETYPES: list = [
    "solid",
    "dashed",
    "dotted",
    "dashdot",
    (0, (5, 1, 1, 1, 1, 1)),
    (0, (8, 2)),
    (0, (1, 3)),
    (0, (3, 1, 3, 1, 1, 1)),
]
SHAPES: list[str] = ["o", "s", "^", "x", "D", "v", "+", "*"]
N_MARKERS: int = 10  # markers per line, log-spaced in time
N_YTICKS: int = 12  # max number of labelled y ticks
X_MIN: float = 1e-4  # left end of the time axis


def cumulative_data(data: pl.DataFrame, models: list[str]) -> pl.DataFrame:
    """Step points (Time, Solved, Model) of the cumulative solved count per model.

    Each line starts at 0 at X_MIN; instances solved faster than X_MIN are
    counted at X_MIN (a vertical jump there). Lines are extended flat to the
    time limit.
    """
    time_limit = float(analysis_utils.TIME_LIMIT)
    solved = data.filter(pl.col("Solved")).with_columns(
        pl.col("Time").clip(lower_bound=X_MIN)
    )
    start_x = X_MIN

    frames: list[pl.DataFrame] = []
    for model in models:
        times = solved.filter(pl.col("Model") == model)["Time"].sort().to_list()
        n = len(times)
        frames.append(
            pl.DataFrame(
                {
                    "Time": [start_x] + times + [time_limit],
                    "Solved": [0] + list(range(1, n + 1)) + [n],
                    "Model": [model] * (n + 2),
                }
            )
        )
    return pl.concat(frames).with_columns(pl.col("Model").cast(pl.Enum(models)))


def marker_data(df: pl.DataFrame, models: list[str]) -> pl.DataFrame:
    """N_MARKERS points per model on its step line, log-spaced in time.

    Each model is shifted by a fraction of the spacing so markers of different
    models do not overlap where the lines coincide.
    """
    log_lo = math.log10(float(df["Time"].min()))
    log_hi = math.log10(float(analysis_utils.TIME_LIMIT))
    step = (log_hi - log_lo) / N_MARKERS
    frames: list[pl.DataFrame] = []
    for i, model in enumerate(models):
        line = df.filter(pl.col("Model") == model)
        times, counts = line["Time"].to_numpy(), line["Solved"].to_numpy()
        t = 10.0 ** (log_lo + step * (np.arange(N_MARKERS) + (i + 0.5) / len(models)))
        idx = np.searchsorted(times, t, side="right") - 1
        frames.append(
            pl.DataFrame({"Time": t, "Solved": counts[idx], "Model": [model] * len(t)})
        )
    return pl.concat(frames).with_columns(pl.col("Model").cast(pl.Enum(models)))


MACRO_DEF = re.compile(
    r"^\s*\\(newcommand|renewcommand|providecommand|DeclareMathOperator|def)\b"
)


def load_macros(tex_file: str) -> None:
    """Add the macro definitions of a LaTeX file (e.g. the paper) to the preamble.

    Only single-line \\newcommand, \\renewcommand, \\providecommand,
    \\DeclareMathOperator and \\def lines before \\begin{document} are kept, so
    the paper's other packages and settings do not interfere with matplotlib.
    """
    if not Path(tex_file).is_file():
        print(f"ERROR: {tex_file} is not an existing file.")
        sys.exit(1)
    macros: list[str] = []
    for line in Path(tex_file).read_text().splitlines():
        if line.lstrip().startswith(r"\begin{document}"):
            break
        if MACRO_DEF.match(line):
            macros.append(line.split("%")[0].strip())
    plt.rcParams["text.latex.preamble"] = BASE_PREAMBLE + "\n".join(macros)


def _tex_escape(text: str) -> str:
    """Escape LaTeX special characters, unless text is already LaTeX (has a \\)."""
    if "\\" in text:
        return text
    for c in "&%$#_{}":
        text = text.replace(c, "\\" + c)
    return text


def cumulative_plot(df: pl.DataFrame, n_instances: int, models: list[str]) -> ggplot:
    time_limit = float(analysis_utils.TIME_LIMIT)
    lo = math.floor(math.log10(float(df["Time"].min())))
    hi = math.ceil(math.log10(time_limit))
    breaks = [10.0**k for k in range(lo, hi + 1)]
    labels = [f"$10^{{{k}}}$" for k in range(lo, hi + 1)]
    y_step = next(
        (
            m * 10**e
            for e in range(6)
            for m in (1, 2, 2.5, 5)
            if n_instances / (m * 10**e) <= N_YTICKS
        ),
        n_instances,
    )
    y_breaks = list(np.arange(0, n_instances + y_step, y_step))
    y_minor = list(np.arange(0, n_instances + y_step, y_step / 2))
    linetypes = [LINETYPES[i % len(LINETYPES)] for i in range(len(models))]
    shapes = [SHAPES[i % len(SHAPES)] for i in range(len(models))]
    markers = marker_data(df, models)
    tex_models = pl.Enum([_tex_escape(m) for m in models])
    df, markers = (
        d.with_columns(
            pl.col("Model").cast(pl.String).map_elements(_tex_escape, pl.String)
        ).with_columns(pl.col("Model").cast(tex_models))
        for d in (df, markers)
    )
    return (
        ggplot(df, aes(x="Time", y="Solved", color="Model", linetype="Model"))
        + geom_step(direction="hv", size=0.7, na_rm=True)
        + geom_point(
            aes(shape="Model"),
            data=markers,
            fill="white",
            size=2.2,
            stroke=0.6,
            na_rm=True,
        )
        + annotation_logticks(sides="b", alpha=0.8, lengths=(0.02, 0.012, 0.007))
        + scale_x_log10(
            limits=(breaks[0], breaks[-1]), breaks=breaks, labels=labels, expand=(0, 0)
        )
        + scale_y_continuous(
            breaks=y_breaks, minor_breaks=y_minor, limits=(0, n_instances)
        )
        + scale_color_brewer(type="qual", palette="Set1")
        + scale_linetype_manual(values=linetypes)
        + scale_shape_manual(values=shapes)
        + labs(
            x="Time (s)",
            y=f"Instances solved (out of {n_instances})",
            color="",
            linetype="",
            shape="",
        )
        + theme_bw()
        + theme(
            figure_size=FIG_SIZE,
            aspect_ratio=1,
            text=element_text(family="serif", size=FONT_SIZE),
            axis_text=element_text(size=FONT_SIZE - 1, color="black"),
            axis_title=element_text(size=FONT_SIZE),
            axis_ticks_major=element_line(color="black", size=0.5),
            axis_ticks_length_major=3,
            panel_border=element_rect(color="black", fill=None, size=0.8),
            panel_grid_major=element_line(color="#e0e0e0", size=0.4),
            panel_grid_minor_x=element_blank(),
            panel_grid_minor_y=element_line(color="#f0f0f0", size=0.3),
            legend_position="bottom",
            legend_key_width=28,
            legend_text=element_text(size=FONT_SIZE - 1),
            legend_title=element_blank(),
        )
    )


def save(plot: ggplot, path: str) -> None:
    fig = plot.draw()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Cactus plot: cumulative instances solved over time."
    )
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument(
        "--aliases",
        nargs="*",
        help="Display aliases (one per file); may be LaTeX, e.g. '\\mLMCstarLL{}'",
    )
    parser.add_argument(
        "--preamble",
        default=None,
        metavar="TEX",
        help="LaTeX file whose macro definitions are loaded (e.g. the paper source)",
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    add_time_limit_arg(parser)
    parser.add_argument(
        "--out", default="cumulative.pdf", help="Output file (default: cumulative.pdf)"
    )
    args = parser.parse_args()

    if args.preamble:
        load_macros(args.preamble)
    set_time_limit(args.time_limit, cap_unsolved=not args.raw_time)
    aliases = resolve_aliases(args.files, args.aliases)
    data = prepare_data(args.files, aliases, domain=args.domain)

    df = cumulative_data(data, aliases)
    plot = cumulative_plot(df, data["Problem"].n_unique(), aliases)
    save(plot, args.out)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
