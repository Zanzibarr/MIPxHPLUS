"""
Script used in pair with parse_results.py to track changes in the performance of the 'hplus' solver

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> current

Transposed comparison table: one row per model, one column group per subset
(all, all-solvable, and cumulative brackets [T,TIME_LIMIT) by the minimum Time
across runs), each with Solved / Nodes / Time sub-columns. First file is the
baseline: absolute values in its row, deltas / SGM ratios in the others ('*'
prefix means p < PVALUE). --latex prints the tabular instead, significant
entries in bold. --metric replaces Nodes with a custom column (no tests): the
subsets are still computed on all the instances, as in the default table, and
only then restricted to the instances where the column is available in all runs.
"""

import argparse

import polars as pl

import analysis_utils as au
from analysis_utils import (
    add_time_limit_arg,
    prepare_data,
    require_complete,
    resolve_aliases,
    set_time_limit,
)
from table import _category_row

METRICS: tuple[str, ...] = ("Nodes", "Time")


def _fmt_edge(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else str(v)


def compute_subsets(
    data: pl.DataFrame, thresholds: list[float]
) -> list[tuple[str, pl.DataFrame]]:
    """all / all-solvable + one cumulative bracket [T,TIME_LIMIT) per threshold."""
    min_time = data.group_by("Problem").agg(pl.col("Time").min().alias("_min"))
    data = data.join(min_time, on="Problem")
    subsets = [
        ("all", data),
        ("all-solvable", data.filter(pl.col("Category") == "all-solvable")),
    ]
    for t in thresholds:
        label = f"[{_fmt_edge(t)},{_fmt_edge(au.TIME_LIMIT)})"
        subsets.append(
            (
                label,
                data.filter(
                    (pl.col("_min") >= t) & (pl.col("_min") < au.TIME_LIMIT)
                ),
            )
        )
    return [(name, sub.drop("_min")) for name, sub in subsets if sub.height > 0]


def _drop_missing(df: pl.DataFrame, col: str) -> pl.DataFrame:
    """Drop problems where any run has a null in col (no warning, unlike require_complete)."""
    bad = df.filter(pl.col(col).is_null()).select("Problem").unique()
    return df.filter(~pl.col("Problem").is_in(bad["Problem"].implode()))


def compute_stats(
    data: pl.DataFrame,
    models: list[str],
    thresholds: list[float],
    metrics: tuple[str, ...] = METRICS,
) -> tuple[list[tuple[str, int]], dict[str, dict[str, list[str]]]]:
    """Return the (subset, count) list and cells[model][subset] = [solved, *metrics].

    Subsets are always computed on the instances with Nodes and Time available,
    so that a custom metric is compared on the same subsets as the default table.
    """
    data = require_complete(data, list(METRICS))
    extra = [m for m in metrics if m not in METRICS]
    test_sig = not extra
    base = models[0]
    subsets, cells = [], {m: {} for m in models}
    for name, sub in compute_subsets(data, thresholds):
        for m in extra:
            sub = _drop_missing(sub, m)
        if sub.height == 0:
            continue
        row = _category_row(sub, models, name, metrics, test_sig)
        subsets.append((name, row["Count"]))
        cells[base][name] = [str(row[f"Solved({base})"])] + [
            f"{row[f'SGM_{m}({base})']:.3f}" for m in metrics
        ]
        for model in models[1:]:
            cells[model][name] = [row[f"ΔSolved({model})"]] + [
                row[f"{m}_ratio({model})"] for m in metrics
            ]
    return subsets, cells


def print_table(
    subsets: list[tuple[str, int]],
    cells: dict[str, dict[str, list[str]]],
    metrics: tuple[str, ...] = METRICS,
) -> None:
    headers = ["Model"] + [
        f"{h}[{name}]" for name, _ in subsets for h in ("Solved", *metrics)
    ]
    rows = [
        [model] + [v for name, _ in subsets for v in cells[model][name]]
        for model in cells
    ]
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    print("Count: " + ", ".join(f"{name}={n}" for name, n in subsets))
    print("  ".join(h.rjust(w) for h, w in zip(headers, widths)))
    print("  ".join("-" * w for w in widths))
    for r in rows:
        print("  ".join(v.rjust(w) for v, w in zip(r, widths)))


def _latex_cell(v: str) -> str:
    return f"\\textbf{{{v[1:]}}}" if v.startswith("*") else v


def _latex_subset(name: str) -> str:
    return f"\\texttt{{{name}}}" if not name.startswith("[") else f"${name}$"


def print_latex(
    subsets: list[tuple[str, int]],
    cells: dict[str, dict[str, list[str]]],
    metrics: tuple[str, ...] = METRICS,
) -> None:
    n = len(subsets)
    sub_hdr = " & ".join(["Solved", *(m.replace("_", "\\_") for m in metrics)])
    lines = [f"\\begin{{tabular}}{{l{' rrr' * n}}}", "\\toprule"]
    lines.append(
        " & "
        + " & ".join(
            f"\\multicolumn{{3}}{{c}}{{{_latex_subset(name)}}}" for name, _ in subsets
        )
        + " \\\\"
    )
    lines.append(
        " ".join(f"\\cmidrule(lr){{{2 + 3 * i}-{4 + 3 * i}}}" for i in range(n))
    )
    lines.append("Model & " + " & ".join([sub_hdr] * n) + " \\\\")
    lines.append("\\midrule")
    for i, model in enumerate(cells):
        vals = [_latex_cell(v) for name, _ in subsets for v in cells[model][name]]
        lines.append(f"{model} & " + " & ".join(vals) + " \\\\")
        if i == 0:
            lines.append("\\midrule")
    lines += ["\\bottomrule", "\\end{tabular}"]
    print("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Transposed comparison table: models as rows, subsets as columns."
    )
    parser.add_argument("files", nargs="+", help="CSV run files (first = baseline)")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--hard",
        nargs="*",
        type=float,
        default=[1, 10],
        metavar="T",
        help="Lower edges of the cumulative brackets [T,time limit) (default: 1 10)",
    )
    parser.add_argument(
        "--metric",
        default=None,
        help="Column to compare instead of Nodes (disables significance tests)",
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--solved",
        action="store_true",
        help="Keep only instances solved by all runs (e.g. for metrics truncated "
        "by the time limit, such as nodes or callback calls)",
    )
    add_time_limit_arg(parser)
    parser.add_argument(
        "--latex", action="store_true", help="Print a LaTeX tabular instead"
    )
    args = parser.parse_args()

    set_time_limit(args.time_limit, cap_unsolved=not args.raw_time)
    aliases = resolve_aliases(args.files, args.aliases)
    extra = [args.metric] if args.metric else None
    data = prepare_data(args.files, aliases, domain=args.domain, extra_cols=extra)
    if args.solved:
        data = data.filter(pl.col("Category") == "all-solvable")

    metrics = (args.metric, "Time") if args.metric else METRICS
    subsets, cells = compute_stats(data, aliases, args.hard, metrics)
    (print_latex if args.latex else print_table)(subsets, cells, metrics)


if __name__ == "__main__":
    main()
