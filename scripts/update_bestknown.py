"""
Script used in pair with parse_results.py to keep track of the best solutions found so far

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> 4.2.0
"""

import sys
import argparse
from pathlib import Path
import polars as pl

DEFAULT_BESTKNOWN = "../results/best_known.csv"
EPS = 1e-6


def load_run(file: str) -> pl.DataFrame:
    csvfile = pl.read_csv(
        file,
        columns=["Instance", "Status", "Final_LB", "Final_UB"],
        schema_overrides={"Final_LB": pl.Float64, "Final_UB": pl.Float64},
    )
    return (
        csvfile.filter(pl.col("Status") == 0)  # Only instances with no errors
        .drop(["Status"])
        .rename({"Final_LB": "Bound", "Final_UB": "Incumbent"})
    )


def check(bk: pl.DataFrame, new: pl.DataFrame) -> None:
    merged = bk.join(new, on="Instance", how="inner", suffix="_new")
    errors = []
    for row in merged.to_dicts():
        prob = row["Instance"]
        bound_ref, inc_ref, bound_new, inc_new = (
            row["Bound"],
            row["Incumbent"],
            row["Bound_new"],
            row["Incumbent_new"],
        )

        if bound_new > inc_new + EPS:
            errors.append(f"{prob}: new bound is higher than new incumbent")

        if bound_new > inc_ref + EPS:
            errors.append(f"{prob}: new bound is higher than reference incumbent")

        if inc_new < bound_ref - EPS:
            errors.append(f"{prob}: new incumbent is lower than reference bound")

    if errors:
        print("ERROR: consistency check failed:")
        for e in errors:
            print(f"  {e}")
        sys.exit(1)


def update(bk: pl.DataFrame, new: pl.DataFrame) -> tuple[pl.DataFrame, dict]:
    stats = {"added": 0, "bound_updates": 0, "incumbent_updates": 0, "total": 0}
    bk_dict = {r["Instance"]: r for r in bk.to_dicts()}

    for row in new.to_dicts():
        prob = row["Instance"]
        bound_new, inc_new = row["Bound"], row["Incumbent"]

        if prob not in bk_dict:
            bk_dict[prob] = {
                "Instance": prob,
                "Bound": bound_new or 0,
                "Incumbent": inc_new or 1e20,
            }
            stats["added"] += 1
            stats["total"] += 1
            continue

        ref = bk_dict[prob]
        changed = False
        if bound_new > ref["Bound"] + EPS:
            ref["Bound"] = bound_new
            stats["bound_updates"] += 1
            changed = True
        if inc_new < ref["Incumbent"] - EPS:
            ref["Incumbent"] = inc_new
            stats["incumbent_updates"] += 1
            changed = True

        if changed:
            stats["total"] += 1

    updated = pl.DataFrame(list(bk_dict.values())).sort("Instance")
    return updated, stats


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Update best_known.csv from run files."
    )
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument(
        "--best-known",
        default=DEFAULT_BESTKNOWN,
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
        else pl.DataFrame({"Instance": [], "Bound": [], "Incumbent": []})
    )

    for file in args.files:
        new = load_run(file)
        check(bk, new)
        bk, stats = update(bk, new)

        print(f"===== {file} =====")
        print(
            f"Added: {stats['added']}  Bound updates: {stats['bound_updates']}  "
            f"Incumbent updates: {stats['incumbent_updates']}  Total changes: {stats['total']}"
        )

    if args.dry_run:
        print("Dry run — no file written.")
        return

    bk.write_csv(str(bk_path))
    print(f"Written: {bk_path}")


if __name__ == "__main__":
    main()
