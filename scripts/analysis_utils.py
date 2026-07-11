"""
Utilities shared by the analysis scripts (table.py, barplot.py, boxplot.py, scatterplot.py)

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0
"""

import sys
from pathlib import Path

import numpy as np
import polars as pl
import scipy.stats

TIME_LIMIT: int = 900
SHIFT: int = 1
PVALUE: float = 0.05
INFINITY: float = 1e20  # Final_UB value when no solution was found
OPT_EPS: float = 1e-6  # bounds closer than this prove optimality

TIME_BRACKETS: list[float] = [0.1, 1, 10, 100]


def _bracket_labels() -> list[str]:
    def _fmt(v: float) -> str:
        return str(int(v)) if v == int(v) else str(v)

    edges = TIME_BRACKETS + [float(TIME_LIMIT)]
    labels, prev = [], 0.0
    for e in edges:
        labels.append(f"[{_fmt(prev)},{_fmt(e)})")
        prev = e
    labels.append(f"[{_fmt(edges[-1])},+inf)")
    return labels


BRACKET_LABELS: list[str] = _bracket_labels()
GROUP_ORDER: list[str] = ["all", "solvable", "all-solvable"] + BRACKET_LABELS


# ---------------------------------------------------------------------------
# Data loading (CSV files produced by parse_results.py)
# ---------------------------------------------------------------------------


def resolve_aliases(files: list[str], aliases: list[str] | None) -> list[str]:
    if aliases:
        if len(aliases) != len(files):
            print(f"ERROR: {len(aliases)} aliases for {len(files)} files.")
            sys.exit(1)
        return aliases
    return [Path(f).stem for f in files]


def _load_run(
    file_path: str,
    alias: str,
    extra_cols: list[str] | None,
    max_time: float | None,
) -> pl.DataFrame:
    """Load a single CSV run file as a polars DataFrame.

    Columns: Model, Problem, Solved (bool), Nodes, Time, [extra_cols...]

    Rows with Status != 0 (errors) are dropped.
    Solved = True iff optimality was proved (Final_LB == Final_UB < INFINITY).
    Unsolved runs have Time capped at TIME_LIMIT.
    """
    if not Path(file_path).is_file():
        print(f"ERROR: {file_path} is not an existing file.")
        sys.exit(1)

    df = pl.read_csv(file_path, infer_schema_length=None)
    df = df.filter(pl.col("Status") == 0)

    lb = pl.col("Final_LB").cast(pl.Float64)
    ub = pl.col("Final_UB").cast(pl.Float64)
    df = df.with_columns(
        ((ub < INFINITY) & ((ub - lb) <= OPT_EPS)).alias("Solved"),
        pl.lit(alias).alias("Model"),
    ).with_columns(
        pl.when(~pl.col("Solved") & (pl.col("Time") >= TIME_LIMIT))
        .then(pl.lit(float(TIME_LIMIT)))
        .otherwise(pl.col("Time"))
        .cast(pl.Float64)
        .alias("Time")
    )
    df = df.rename({"Instance": "Problem", "N_Nodes": "Nodes"})

    keep = ["Model", "Problem", "Solved", "Nodes", "Time"]
    if extra_cols:
        keep += [c for c in extra_cols if c in df.columns and c not in keep]

    df = df.select(keep)
    if max_time is not None:
        df = df.filter(pl.col("Time") <= max_time)
    return df.sort("Problem")


def _add_groups(df: pl.DataFrame) -> pl.DataFrame:
    """Add the Time_Bracket (by min Time across runs) and Category columns."""
    edges = TIME_BRACKETS + [float(TIME_LIMIT)]
    min_time = df.group_by("Problem").agg(pl.col("Time").min().alias("_min"))
    expr = pl.when(pl.col("_min") < edges[0]).then(pl.lit(BRACKET_LABELS[0]))
    for i in range(1, len(edges)):
        expr = expr.when(pl.col("_min") < edges[i]).then(pl.lit(BRACKET_LABELS[i]))
    expr = expr.otherwise(pl.lit(BRACKET_LABELS[-1])).alias("Time_Bracket")
    df = df.join(min_time, on="Problem").with_columns(expr).drop("_min")

    flags = df.group_by("Problem").agg(
        pl.col("Solved").all().alias("_all"),
        pl.col("Solved").any().alias("_any"),
    )
    return (
        df.join(flags, on="Problem")
        .with_columns(
            pl.when(pl.col("_all"))
            .then(pl.lit("all-solvable"))
            .when(pl.col("_any"))
            .then(pl.lit("solvable"))
            .otherwise(pl.lit("non-solvable"))
            .alias("Category")
        )
        .drop(["_all", "_any"])
    )


def prepare_data(
    files: list[str],
    aliases: list[str],
    domain: str = "",
    extra_cols: list[str] | None = None,
    max_time: float | None = None,
) -> pl.DataFrame:
    """Load runs, keep instances present in all of them, add group columns."""
    assert len(aliases) == len(files)
    dfs = [_load_run(f, a, extra_cols, max_time) for f, a in zip(files, aliases)]
    combined = pl.concat(dfs)
    counts = combined.group_by("Problem").agg(pl.col("Model").n_unique().alias("_n"))
    valid = counts.filter(pl.col("_n") == len(files)).select("Problem")
    df = _add_groups(combined.join(valid, on="Problem").sort("Problem"))
    if domain:
        df = df.filter(pl.col("Problem").str.contains(domain))
    return df


def require_complete(df: pl.DataFrame, cols: list[str]) -> pl.DataFrame:
    """Drop problems where any run has a null in any of cols; warn if any removed."""
    present = [c for c in cols if c in df.columns]
    if not present:
        return df
    has_null = pl.any_horizontal([pl.col(c).is_null() for c in present])
    bad = df.filter(has_null).select("Problem").unique()
    if bad.height > 0:
        print(
            f"WARNING: dropped {bad.height} problem(s) with missing values "
            f"in {present} (excluded from all runs for fair comparison)."
        )
        df = df.filter(~pl.col("Problem").is_in(bad["Problem"]))
    return df


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------


def sgm(arr: np.ndarray, shift: float = float(SHIFT)) -> float:
    """Shifted geometric mean."""
    a = np.asarray(arr, dtype=float)
    return float(np.exp(np.mean(np.log(a + shift))) - shift)


def wilcoxon_test(
    x: np.ndarray,
    y: np.ndarray,
    shift: float = float(SHIFT),
    min_abs: float = 0.05,
    min_samples: int = 25,
) -> float:
    """Wilcoxon paired signed-rank test on shifted log-ratios; two-sided p-value.

    Differences below min_abs are treated as zero and dropped; with fewer than
    min_samples non-zero differences a sign (exact binomial) test is used.
    """
    assert len(x) == len(y) and len(x) > 0

    delta = np.around(np.log((y + shift) / (x + shift)), 3)
    delta[np.abs(delta) <= min_abs] = 0.0

    n_smaller = int(np.sum(delta < 0.0))
    n_larger = int(np.sum(delta > 0.0))
    n_diff = n_smaller + n_larger

    if n_diff == 0:
        return 1.0
    if n_diff < min_samples:
        return scipy.stats.binomtest(n_smaller, n_diff, p=0.5).pvalue
    return scipy.stats.wilcoxon(delta[delta != 0.0], zero_method="wilcox")[1]


def mcnemar_test(x: np.ndarray, y: np.ndarray) -> float:
    """McNemar's test (continuity-corrected) on two boolean arrays; p-value."""
    n01 = int(np.logical_and(~x, y).sum())
    n10 = int(np.logical_and(x, ~y).sum())
    if n01 + n10 == 0:
        return 1.0
    stat = (abs(n01 - n10) - 1) ** 2 / float(n01 + n10)
    return float(scipy.stats.chi2(1).sf(stat))
