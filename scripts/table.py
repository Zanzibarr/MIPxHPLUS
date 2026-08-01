"""
Script used in pair with parse_results.py to track changes in the performance of the 'hplus' solver

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> 4.2.0

Comparison table: category + time-bracket rows (or per-family rows with
--by-family), model metric columns. First file is the baseline.
Default metrics: Nodes + Time, with significance tests ('*' prefix means
p < PVALUE). --metric compares a single custom column instead (no tests).
"""

import argparse

import polars as pl

from analysis_utils import (
    GROUP_ORDER,
    PVALUE,
    mcnemar_test,
    prepare_data,
    require_complete,
    resolve_aliases,
    sgm,
    wilcoxon_test,
    OPT_EPS,
)


def _category_row(
    df: pl.DataFrame,
    models: list[str],
    category: str,
    metrics: tuple[str, ...],
    test_sig: bool,
) -> dict:
    """One table row: solved counts and SGM ratios of every model vs the baseline."""
    base = models[0]
    base_df = df.filter(pl.col("Model") == base).sort("Problem")

    row: dict = {"Category": category, "Count": df["Problem"].n_unique()}
    solved_base = int(base_df["Solved"].sum())
    row[f"Solved({base})"] = solved_base

    base_data: dict[str, tuple] = {}
    for m in metrics:
        arr = base_df[m].cast(pl.Float64).to_numpy()
        sv = sgm(arr)
        base_data[m] = (arr, sv)
        row[f"SGM_{m}({base})"] = round(sv, 3)

    for model in models[1:]:
        model_df = df.filter(pl.col("Model") == model).sort("Problem")
        delta = int(model_df["Solved"].sum()) - solved_base
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
            sv = sgm(model_arr)
            ratio = (
                1
                if abs(sv - base_sv) < OPT_EPS
                else (sv / base_sv if base_sv != 0 else float("nan"))
            )
            ratio_str = f"{ratio:.3f}"
            if test_sig and wilcoxon_test(base_arr, model_arr) < PVALUE:
                ratio_str = f"*{ratio_str}"
            row[f"{m}_ratio({model})"] = ratio_str

    return row


def compute_stats(
    data: pl.DataFrame,
    models: list[str],
    metrics: tuple[str, ...] = ("Nodes", "Time"),
    test_significance: bool = True,
) -> pl.DataFrame:
    """Rows: all / solvable / all-solvable + one per non-empty time bracket."""
    data = require_complete(data, list(metrics))

    solvable = data.filter(pl.col("Category") != "non-solvable")
    all_solvable = data.filter(pl.col("Category") == "all-solvable")
    rows = [
        _category_row(data, models, "all", metrics, test_significance),
        _category_row(solvable, models, "solvable", metrics, test_significance),
        _category_row(all_solvable, models, "all-solvable", metrics, test_significance),
    ]

    present = set(data["Time_Bracket"].unique().to_list())
    for bracket in [b for b in GROUP_ORDER[3:] if b in present]:
        sub = data.filter(pl.col("Time_Bracket") == bracket)
        rows.append(_category_row(sub, models, bracket, metrics, test_significance))

    return pl.DataFrame(rows)


def compute_problem_stats(
    data: pl.DataFrame,
    models: list[str],
    metrics: tuple[str, ...] = ("Nodes", "Time"),
    test_significance: bool = True,
) -> pl.DataFrame:
    """One row per problem family (prefix before the first '-')."""
    data = require_complete(data, list(metrics))
    data = data.with_columns(
        pl.col("Problem").str.split("-").list.first().alias("_family")
    )
    rows = [
        _category_row(
            data.filter(pl.col("_family") == fam),
            models,
            fam,
            metrics,
            test_significance,
        )
        for fam in sorted(data["_family"].unique().to_list())
    ]
    return pl.DataFrame(rows)


def print_table(df: pl.DataFrame) -> None:
    headers = df.columns
    rows_str = [
        [f"{v:.3f}" if isinstance(v, float) else str(v) for v in row.values()]
        for row in df.to_dicts()
    ]
    col_widths = [
        max(len(h), *(len(r[i]) for r in rows_str)) for i, h in enumerate(headers)
    ]
    print("  ".join(h.rjust(w) for h, w in zip(headers, col_widths)))
    print("  ".join("-" * w for w in col_widths))
    for row in rows_str:
        print("  ".join(v.rjust(w) for v, w in zip(row, col_widths)))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Comparison table by category and time bracket."
    )
    parser.add_argument("files", nargs="+", help="CSV run files (first = baseline)")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--metric",
        default=None,
        help="Extra metric column to compare (disables significance tests)",
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--time",
        type=float,
        default=None,
        metavar="T",
        help="Exclude instances where any run exceeded T seconds",
    )
    parser.add_argument(
        "--by-family",
        action="store_true",
        help="Rows per problem family instead of category/bracket",
    )
    args = parser.parse_args()

    aliases = resolve_aliases(args.files, args.aliases)
    extra = [args.metric] if args.metric else None
    data = prepare_data(
        args.files, aliases, domain=args.domain, extra_cols=extra, max_time=args.time
    )

    compute = compute_problem_stats if args.by_family else compute_stats
    if args.metric:
        result = compute(
            data, aliases, metrics=(args.metric, "Time"), test_significance=False
        )
    else:
        result = compute(data, aliases)

    print_table(result)


if __name__ == "__main__":
    main()
