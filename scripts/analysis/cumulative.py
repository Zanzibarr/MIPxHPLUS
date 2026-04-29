#!/usr/bin/env python
#
# Cumulative (cactus) plot: instances solved vs time.
#
# Usage:
#   cumulative.py [options] FILE1 FILE2 [...]

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import prepare_data, resolve_aliases, TIME_LIMIT
from plots import cumulative_plot, save


def main() -> None:
    parser = argparse.ArgumentParser(description="Cumulative instances-solved plot.")
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--solved", action="store_true", help="Keep only instances solved by all runs"
    )
    parser.add_argument(
        "--out", default="cumulative.pdf", help="Output file (default: cumulative.pdf)"
    )
    args = parser.parse_args()

    aliases = resolve_aliases(args.files, args.aliases)
    data = prepare_data(
        args.files, aliases, domain=args.domain, solved_only=args.solved
    )

    plot = cumulative_plot(
        data,
        x_col="Time",
        status_col="Solved",
        group_col="Model",
        x_limit=float(TIME_LIMIT),
        title="Cumulative Instances Solved",
        x_label="Time (s)",
        y_label="Instances Solved",
        group_order=aliases,
    )
    save(plot, args.out, width=10, height=6)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
