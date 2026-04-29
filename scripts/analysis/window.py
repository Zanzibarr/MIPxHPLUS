#!/usr/bin/env python
#
# Window histogram: solved instances binned over log-scale time windows.
# Use --facet for one panel per model (stacked), default is dodged bars.
#
# Usage:
#   window.py [options] FILE1 FILE2 [...]

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import prepare_data, resolve_aliases, TIME_LIMIT
from plots import window_plot, window_facet_plot, save


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Window histogram of solved instances over time."
    )
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--n-windows",
        type=int,
        default=20,
        help="Number of log-scale bins (default: 20)",
    )
    parser.add_argument(
        "--facet", action="store_true", help="Faceted layout (one panel per model)"
    )
    parser.add_argument(
        "--solved", action="store_true", help="Keep only instances solved by all runs"
    )
    parser.add_argument(
        "--out", default="window.pdf", help="Output file (default: window.pdf)"
    )
    args = parser.parse_args()

    aliases = resolve_aliases(args.files, args.aliases)
    data = prepare_data(args.files, aliases, domain=args.domain, solved_only=args.solved)

    fn = window_facet_plot if args.facet else window_plot
    plot = fn(
        data,
        x_col="Time",
        status_col="Solved",
        group_col="Model",
        x_limit=float(TIME_LIMIT),
        n_windows=args.n_windows,
        title="Instances Solved per Time Window",
        x_label="Time (s)",
        group_order=aliases,
    )
    height = 3 * len(aliases) if args.facet else 6
    save(plot, args.out, width=10, height=height)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
