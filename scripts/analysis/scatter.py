#!/usr/bin/env python
#
# Log-log scatter plot: each run vs baseline.
#
# Usage:
#   scatter.py [options] BASELINE.csv OTHER.csv [MORE.csv ...]

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import prepare_data, resolve_aliases
from plots import scatter_comparison_plot, save


def main() -> None:
    parser = argparse.ArgumentParser(description="Log-log scatter: runs vs baseline.")
    parser.add_argument("files", nargs="+", help="CSV run files (first = baseline)")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--metric", default="Time", help="Metric column (default: Time)"
    )
    parser.add_argument(
        "--domain", default="", help="Highlight instances matching domain substring"
    )
    parser.add_argument(
        "--solved", action="store_true", help="Keep only instances solved by all runs"
    )
    parser.add_argument(
        "--out", default="scatter.pdf", help="Output file (default: scatter.pdf)"
    )
    args = parser.parse_args()

    if len(args.files) < 2:
        parser.error("At least two files required (baseline + one comparison).")

    aliases = resolve_aliases(args.files, args.aliases)
    full_data = prepare_data(args.files, aliases, solved_only=args.solved)
    highlight = (
        prepare_data(args.files, aliases, domain=args.domain, solved_only=args.solved)
        if args.domain
        else None
    )

    plot = scatter_comparison_plot(
        full_data, aliases, metric=args.metric, highlight=highlight
    )
    n_comp = len(aliases) - 1
    save(plot, args.out, width=6 * n_comp, height=6)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
