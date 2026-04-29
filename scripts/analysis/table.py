#!/usr/bin/env python
#
# Comparison table: category + time-bracket rows, model metric columns.
#
# Usage:
#   table.py [options] FILE1 FILE2 [...]
#
# Default metrics: Nodes + Time (with significance tests).
# Use --metric to compare a single custom column (no significance test).
# Use --by-family for per-problem-family rows instead of category/bracket rows.

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import (
    prepare_data,
    compute_stats,
    compute_problem_stats,
    resolve_aliases,
    print_table,
)


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
        "--solved", action="store_true", help="Keep only instances solved by all runs"
    )
    parser.add_argument(
        "--no-sig", action="store_true", help="Disable significance tests"
    )
    parser.add_argument(
        "--by-family",
        action="store_true",
        help="Rows per problem family instead of category/bracket",
    )
    args = parser.parse_args()

    aliases = resolve_aliases(args.files, args.aliases)

    if args.metric:
        data = prepare_data(
            args.files, aliases, domain=args.domain, extra_cols=[args.metric],
            solved_only=args.solved,
        )
        result = compute_stats(
            data, metrics=(args.metric, "Time"), models=aliases, test_significance=False
        )
    elif args.by_family:
        data = prepare_data(
            args.files, aliases, domain=args.domain, solved_only=args.solved
        )
        result = compute_problem_stats(
            data, models=aliases, test_significance=not args.no_sig
        )
    else:
        data = prepare_data(
            args.files, aliases, domain=args.domain, solved_only=args.solved
        )
        result = compute_stats(data, models=aliases, test_significance=not args.no_sig)

    print_table(result)


if __name__ == "__main__":
    main()
