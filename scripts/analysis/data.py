#!/usr/bin/env python

import sys
from pathlib import Path

import numpy as np
import polars as pl

sys.dont_write_bytecode = True

from stats import mcnemar_test, wilcoxon_test

TIME_LIMIT: int = 900
SHIFT: int = 1
PVALUE: float = 0.05

_GROUP_ORDER: list[str] = [
    "all",
    "solvable",
    "all-solvable",
    "[0,0.1)",
    "[0.1,1)",
    "[1,10)",
    "[10,100)",
    "[100,900)",
    "[900,+inf)",
]


# ---------------------------------------------------------------------------
# Time-bracket helpers
# ---------------------------------------------------------------------------


def _bracket_labels(brackets: list[float], time_limit: float) -> list[str]:
    def _fmt(v: float) -> str:
        return str(int(v)) if v == int(v) else str(v)

    edges = brackets + [time_limit]
    labels, prev = [], 0.0
    for e in edges:
        labels.append(f"[{_fmt(prev)},{_fmt(e)})")
        prev = e
    labels.append(f"[{_fmt(edges[-1])},+inf)")
    return labels


def _add_time_bracket(
    df: pl.DataFrame,
    time_col: str = "Time",
    time_limit: float = float(TIME_LIMIT),
    group_col: str = "Problem",
    brackets: list[float] | None = None,
) -> pl.DataFrame:
    if brackets is None:
        brackets = [0.1, 1, 10, 100]
    labels = _bracket_labels(brackets, time_limit)
    edges = brackets + [time_limit]

    min_time = df.group_by(group_col).agg(pl.col(time_col).min().alias("_min"))
    expr = pl.when(pl.col("_min") < edges[0]).then(pl.lit(labels[0]))
    for i in range(1, len(edges)):
        expr = expr.when(pl.col("_min") < edges[i]).then(pl.lit(labels[i]))
    expr = expr.otherwise(pl.lit(labels[-1])).alias("Time_Bracket")
    return df.join(min_time, on=group_col).with_columns(expr).drop("_min")


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------


def load_run(
    file_path: str,
    alias: str | None = None,
    extra_cols: list[str] | None = None,
    max_time: float | None = None,
) -> pl.DataFrame:
    """Load a single CSV run file as a polars DataFrame.

    Columns: Model, Problem, Solved (bool), Nodes, Time, [extra_cols...]

    Solved = True iff Status == 0.
    Non-optimal runs have Time capped at TIME_LIMIT.
    Rows with Status < 0 or Status == 4 (errors/abort) are dropped.
    """
    if not Path(file_path).is_file():
        print(f"ERROR: {file_path} is not an existing file.")
        sys.exit(1)

    model = alias if alias is not None else Path(file_path).stem
    df = pl.read_csv(file_path, infer_schema_length=None)
    df = df.filter((pl.col("Status") >= 0) & (pl.col("Status") != 4))

    df = df.with_columns(
        pl.when((pl.col("Status") != 0) & (pl.col("Time") >= TIME_LIMIT))
        .then(pl.lit(float(TIME_LIMIT)))
        .otherwise(pl.col("Time"))
        .cast(pl.Float64)
        .alias("Time"),
        (pl.col("Status") == 0).alias("Solved"),
        pl.lit(model).alias("Model"),
    )

    rename_map: dict[str, str] = {}
    if "Instance" in df.columns:
        rename_map["Instance"] = "Problem"
    if "N_Nodes" in df.columns:
        rename_map["N_Nodes"] = "Nodes"
    if rename_map:
        df = df.rename(rename_map)

    keep = ["Model", "Problem", "Solved", "Nodes", "Time"]
    if extra_cols:
        keep += [c for c in extra_cols if c in df.columns]

    df = df.select(keep)
    if max_time is not None:
        df = df.filter(pl.col("Time") <= max_time)
    return df.sort("Problem")


def load_runs(
    files: list[str],
    aliases: list[str] | None = None,
    extra_cols: list[str] | None = None,
    max_time: float | None = None,
) -> pl.DataFrame:
    """Load multiple run files, keeping only instances present in all runs."""
    if aliases is None:
        aliases = [Path(f).stem for f in files]
    assert len(aliases) == len(files)

    dfs = [
        load_run(files[i], aliases[i], extra_cols, max_time) for i in range(len(files))
    ]
    combined = pl.concat(dfs)
    counts = combined.group_by("Problem").agg(pl.col("Model").n_unique().alias("_n"))
    valid = counts.filter(pl.col("_n") == len(files)).select("Problem")
    return combined.join(valid, on="Problem").sort("Problem")


def _add_categories(df: pl.DataFrame) -> pl.DataFrame:
    df = _add_time_bracket(
        df, time_col="Time", time_limit=float(TIME_LIMIT), group_col="Problem"
    )
    flags = df.group_by("Problem").agg(
        pl.col("Solved").all().alias("_all_solved"),
        pl.col("Solved").any().alias("_any_solved"),
    )
    df = df.join(flags, on="Problem")
    return df.with_columns(
        pl.when(pl.col("_all_solved"))
        .then(pl.lit("all-solvable"))
        .when(pl.col("_any_solved"))
        .then(pl.lit("solvable"))
        .otherwise(pl.lit("non-solvable"))
        .alias("Category")
    ).drop(["_all_solved", "_any_solved"])


def prepare_data(
    files: list[str],
    aliases: list[str] | None = None,
    domain: str = "",
    extra_cols: list[str] | None = None,
    solved_only: bool = False,
    max_time: float | None = None,
) -> pl.DataFrame:
    """Load, intersect, and categorize runs. Optionally filter by domain substring."""
    df = _add_categories(load_runs(files, aliases, extra_cols, max_time))
    if domain:
        df = df.filter(pl.col("Problem").str.contains(domain))
    if solved_only:
        df = df.filter(pl.col("Category") == "all-solvable")
    return df


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------


def _require_complete(df: pl.DataFrame, cols: list[str]) -> pl.DataFrame:
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


def _sgm(arr: np.ndarray, shift: float = float(SHIFT)) -> float:
    a = np.asarray(arr, dtype=float)
    return float(np.exp(np.mean(np.log(a + shift))) - shift)


def _category_row(
    df: pl.DataFrame,
    models: list[str],
    category: str,
    metrics: tuple[str, ...],
    test_sig: bool,
) -> dict:
    base = models[0]
    base_df = df.filter(pl.col("Model") == base).sort("Problem")

    row: dict = {"Category": category, "Count": df["Problem"].n_unique()}
    solved_base = int(base_df["Solved"].sum())
    row[f"Solved({base})"] = solved_base

    base_data: dict[str, tuple[np.ndarray, float]] = {}
    for m in metrics:
        arr = base_df[m].cast(pl.Float64).to_numpy()
        sv = _sgm(arr)
        base_data[m] = (arr, sv)
        row[f"SGM_{m}({base})"] = round(sv, 3)

    for model in models[1:]:
        model_df = df.filter(pl.col("Model") == model).sort("Problem")
        solved_m = int(model_df["Solved"].sum())
        delta = solved_m - solved_base
        delta_str = f"+{delta}" if delta > 0 else str(delta)
        if test_sig:
            pv = mcnemar_test(
                base_df["Solved"].to_numpy(), model_df["Solved"].to_numpy()
            )
            if pv < PVALUE:
                delta_str = f"*{delta_str}"
        row[f"ΔSolved({model})"] = delta_str

        for m in metrics:
            base_arr, base_sv = base_data[m]
            model_arr = model_df[m].cast(pl.Float64).to_numpy()
            sv = _sgm(model_arr)
            ratio = sv / base_sv if base_sv != 0 else float("nan")
            ratio_str = f"{ratio:.3f}"
            if test_sig:
                pv = wilcoxon_test(base_arr, model_arr, shift=float(SHIFT))
                if pv < PVALUE:
                    ratio_str = f"*{ratio_str}"
            row[f"{m}_ratio({model})"] = ratio_str

    return row


def compute_stats(
    data: pl.DataFrame,
    metrics: tuple[str, ...] = ("Nodes", "Time"),
    models: list[str] | None = None,
    test_significance: bool = True,
) -> pl.DataFrame:
    """Category/bracket comparison table. Significant differences prefixed with '*'."""
    if models is None:
        models = data["Model"].unique().to_list()
    data = _require_complete(data, list(metrics))

    present_brackets = set(data["Time_Bracket"].unique().to_list())
    bracket_order = [b for b in _GROUP_ORDER[3:] if b in present_brackets]

    solvable = data.filter(pl.col("Category") != "non-solvable")
    all_solvable = data.filter(pl.col("Category") == "all-solvable")

    rows = [
        _category_row(data, models, "all", metrics, test_significance),
        _category_row(solvable, models, "solvable", metrics, test_significance),
        _category_row(all_solvable, models, "all-solvable", metrics, test_significance),
    ]
    for bracket in bracket_order:
        sub = data.filter(pl.col("Time_Bracket") == bracket)
        if sub.height > 0:
            rows.append(_category_row(sub, models, bracket, metrics, test_significance))

    return pl.DataFrame(rows)


def compute_problem_stats(
    data: pl.DataFrame,
    metrics: tuple[str, ...] = ("Nodes", "Time"),
    models: list[str] | None = None,
    test_significance: bool = True,
) -> pl.DataFrame:
    """Per-problem-family stats (row per family prefix before the first '-')."""
    if models is None:
        models = data["Model"].unique().to_list()
    data = _require_complete(data, list(metrics))

    data = data.with_columns(
        pl.col("Problem").str.split("-").list.first().alias("ProbFamily")
    )
    families = sorted(data["ProbFamily"].unique().to_list())

    rows = [
        _category_row(
            data.filter(pl.col("ProbFamily") == fam),
            models,
            fam,
            metrics,
            test_significance,
        )
        for fam in families
    ]
    return pl.DataFrame(rows)


def compare_data(
    data: pl.DataFrame,
    metric: str,
    models: list[str],
) -> pl.DataFrame:
    """SGM and SGM ratios vs baseline for each group and model."""
    data = _require_complete(data, [metric])
    base = models[0]
    present_brackets = set(data["Time_Bracket"].unique().to_list())

    def _group_rows(subset: pl.DataFrame, group: str) -> list[dict]:
        base_arr = (
            subset.filter(pl.col("Model") == base)[metric].cast(pl.Float64).to_numpy()
        )
        base_sv = _sgm(base_arr)
        rows = []
        for model in models:
            arr = (
                subset.filter(pl.col("Model") == model)[metric]
                .cast(pl.Float64)
                .to_numpy()
            )
            sv = _sgm(arr)
            ratio = sv / base_sv if base_sv != 0 else float("nan")
            rows.append(
                {
                    "Group": group,
                    "Model": model,
                    "sgm": sv,
                    "baseline_sgm": base_sv,
                    "sgm_ratio": ratio,
                }
            )
        return rows

    rows = (
        _group_rows(data, "all")
        + _group_rows(data.filter(pl.col("Category") != "non-solvable"), "solvable")
        + _group_rows(data.filter(pl.col("Category") == "all-solvable"), "all-solvable")
    )
    for bracket in _GROUP_ORDER[3:]:
        if bracket in present_brackets:
            sub = data.filter(pl.col("Time_Bracket") == bracket)
            if sub.height > 0:
                rows.extend(_group_rows(sub, bracket))

    group_rank = {g: i for i, g in enumerate(_GROUP_ORDER)}
    model_rank = {m: i for i, m in enumerate(models)}
    df = pl.DataFrame(rows)
    return (
        df.with_columns(
            [
                pl.col("Group")
                .map_elements(lambda g: group_rank.get(g, 999), return_dtype=pl.Int32)
                .alias("_gr"),
                pl.col("Model")
                .map_elements(lambda m: model_rank.get(m, 999), return_dtype=pl.Int32)
                .alias("_mr"),
            ]
        )
        .sort(["_gr", "_mr"])
        .drop(["_gr", "_mr"])
    )


# ---------------------------------------------------------------------------
# Optimality gap
# ---------------------------------------------------------------------------

_DEFAULT_BEST_KNOWN = str(
    Path(__file__).parent.parent.parent / "results" / "best_known.csv"
)


def compute_optimality_gap(
    data: pl.DataFrame,
    col: str,
    best_known_file: str = _DEFAULT_BEST_KNOWN,
) -> pl.DataFrame:
    """Replace col with Gap = |col - Incumbent| / max(col, Incumbent) * 100.

    Standard MIP gap formula; result is always in [0, 100].
    Works for both lower bounds (col ≤ optimal) and upper bounds (col ≥ optimal).
    Joins on Problem; keeps only instances with Optimal=True in best_known.
    Drops col and Incumbent, adds Gap (float, %).
    """
    if not Path(best_known_file).is_file():
        print(f"ERROR: best_known file not found: {best_known_file}")
        sys.exit(1)

    bk = (
        pl.read_csv(best_known_file)
        .filter(pl.col("Optimal").cast(pl.Boolean))
        .select(["Problem", pl.col("Incumbent").cast(pl.Float64)])
    )

    result = data.join(bk, on="Problem", how="inner")
    result = result.with_columns(
        (
            (pl.col(col).cast(pl.Float64) - pl.col("Incumbent")).abs()
            / pl.max_horizontal(pl.col(col).cast(pl.Float64), pl.col("Incumbent"))
            * 100
        ).alias("Gap")
    ).drop([col, "Incumbent"])
    return result


def compute_gaps(
    data: pl.DataFrame,
    cols: list[str],
    best_known_file: str = _DEFAULT_BEST_KNOWN,
) -> pl.DataFrame:
    """Gap (%) vs optimal for multiple bound columns, in long format.

    Returns: Problem, Model, Phase, Gap.
    Works for both lower and upper bounds (detected by value, not name).
    Only keeps instances with Optimal=True in best_known.
    """
    data = _require_complete(data, cols)
    frames: list[pl.DataFrame] = []
    for c in cols:
        if c not in data.columns:
            print(f"WARNING: column {c!r} not found in data, skipping.")
            continue
        gap_df = compute_optimality_gap(data, c, best_known_file)
        frames.append(
            gap_df.select(["Problem", "Model", pl.lit(c).alias("Phase"), "Gap"])
        )

    if not frames:
        return pl.DataFrame({"Problem": [], "Model": [], "Phase": [], "Gap": []})
    return pl.concat(frames)


def compute_gap_diffs(
    data: pl.DataFrame,
    baseline: str,
) -> pl.DataFrame:
    """Paired gap differences vs a baseline model, in long format.

    For each (Problem, Phase), computes Gap_model − Gap_baseline for every
    non-baseline model.  Returns: Problem, Phase, Comparison, Difference.
    """
    base = data.filter(pl.col("Model") == baseline).select(
        ["Problem", "Phase", pl.col("Gap").alias("_base")]
    )
    others = data.filter(pl.col("Model") != baseline)
    joined = others.join(base, on=["Problem", "Phase"])
    return joined.with_columns(
        (pl.col("Gap") - pl.col("_base")).alias("Difference"),
        (pl.col("Model") + pl.lit(f" − {baseline}")).alias("Comparison"),
    ).select(["Problem", "Phase", "Comparison", "Difference"])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def resolve_aliases(files: list[str], aliases: list[str] | None) -> list[str]:
    if aliases:
        if len(aliases) != len(files):
            print(f"ERROR: {len(aliases)} aliases for {len(files)} files.")
            sys.exit(1)
        return aliases
    return [Path(f).stem for f in files]


def print_table(df: pl.DataFrame) -> None:
    headers = df.columns
    rows_str = []
    for row in df.to_dicts():
        rows_str.append(
            [f"{v:.3f}" if isinstance(v, float) else str(v) for v in row.values()]
        )
    col_widths = [
        max(len(h), *(len(r[i]) for r in rows_str)) for i, h in enumerate(headers)
    ]
    print("  ".join(h.rjust(w) for h, w in zip(headers, col_widths)))
    print("  ".join("-" * w for w in col_widths))
    for row in rows_str:
        print("  ".join(v.rjust(w) for v, w in zip(row, col_widths)))
