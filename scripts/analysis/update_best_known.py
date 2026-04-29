#!/usr/bin/env python
#
# Update results/best_known.csv from one or more run CSV files.
#
# For each problem: best (min) Final_UB is the incumbent; Optimal if any run solved it.
# Checks for consistency before writing, reports statistics.
#
# Usage:
#   update_best_known.py RUN1.csv [RUN2.csv ...]

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

import polars as pl
from data import load_run, _DEFAULT_BEST_KNOWN


def _aggregate_runs(files: list[str]) -> pl.DataFrame:
    frames = [load_run(f, extra_cols=["Final_UB"]) for f in files]
    combined = pl.concat(frames)
    return (
        combined.group_by("Problem")
        .agg(
            pl.col("Solved").any().alias("Optimal"),
            pl.col("Final_UB").cast(pl.Float64).min().alias("Incumbent"),
        )
        .sort("Problem")
    )


def _check(bk: pl.DataFrame, new: pl.DataFrame) -> None:
    merged = bk.join(new, on="Problem", how="inner", suffix="_new")
    errors = []
    for row in merged.to_dicts():
        prob = row["Problem"]
        opt_ref, inc_ref = row["Optimal"], row["Incumbent"]
        opt_new, inc_new = row["Optimal_new"], row["Incumbent_new"]
        if inc_new is None:
            continue
        if opt_ref and inc_new < inc_ref:
            errors.append(
                f"{prob}: new incumbent {inc_new} < existing optimal {inc_ref}"
            )
        if opt_new and inc_ref < inc_new:
            errors.append(f"{prob}: new optimal {inc_new} > existing best {inc_ref}")
    if errors:
        print("ERROR: consistency check failed:")
        for e in errors:
            print(f"  {e}")
        sys.exit(1)


def _update(bk: pl.DataFrame, new: pl.DataFrame) -> tuple[pl.DataFrame, dict]:
    stats = {"added": 0, "opt_updates": 0, "incumbent_updates": 0, "total": 0}
    bk_dict = {r["Problem"]: r for r in bk.to_dicts()}

    for row in new.to_dicts():
        prob = row["Problem"]
        opt_new, inc_new = row["Optimal"], row["Incumbent"]
        if inc_new is None:
            continue
        if prob not in bk_dict:
            bk_dict[prob] = {"Problem": prob, "Optimal": opt_new, "Incumbent": inc_new}
            stats["added"] += 1
            stats["total"] += 1
            continue
        ref = bk_dict[prob]
        changed = False
        if opt_new and not ref["Optimal"]:
            ref["Optimal"] = True
            stats["opt_updates"] += 1
            changed = True
        if inc_new < ref["Incumbent"]:
            ref["Incumbent"] = inc_new
            stats["incumbent_updates"] += 1
            changed = True
        if changed:
            stats["total"] += 1

    updated = pl.DataFrame(list(bk_dict.values())).sort("Problem")
    return updated, stats


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Update best_known.csv from run files."
    )
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument(
        "--best-known",
        default=_DEFAULT_BEST_KNOWN,
        help="Path to best_known.csv (default: results/best_known.csv in repo root)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Check only, do not write"
    )
    args = parser.parse_args()

    bk_path = Path(args.best_known)
    bk = (
        pl.read_csv(str(bk_path))
        if bk_path.is_file()
        else pl.DataFrame({"Problem": [], "Optimal": [], "Incumbent": []})
    )

    new = _aggregate_runs(args.files)
    _check(bk, new)
    updated, stats = _update(bk, new)

    print(
        f"Added: {stats['added']}  Optimal updates: {stats['opt_updates']}  "
        f"Incumbent updates: {stats['incumbent_updates']}  Total changes: {stats['total']}"
    )

    if args.dry_run:
        print("Dry run — no file written.")
        return

    updated.write_csv(str(bk_path))
    print(f"Written: {bk_path}")


if __name__ == "__main__":
    main()
