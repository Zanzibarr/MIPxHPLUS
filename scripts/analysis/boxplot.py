#!/usr/bin/env python
#
# Boxplot of a metric column across runs.
#
# Usage:
#   boxplot.py [options] FILE1 FILE2 [...]

import sys
import argparse
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).parent))

from data import (
    prepare_data,
    resolve_aliases,
    compute_gaps,
    compute_gap_diffs,
    _DEFAULT_BEST_KNOWN,
)
from plots import boxplot, violin_plot, split_violin_plot, save


def main() -> None:
    parser = argparse.ArgumentParser(description="Boxplot of a metric across runs.")
    parser.add_argument("files", nargs="+", help="CSV run files")
    parser.add_argument("--aliases", nargs="*", help="Display aliases (one per file)")
    parser.add_argument(
        "--metric", default="Time", help="Metric column (default: Time)"
    )
    parser.add_argument(
        "--gap",
        nargs="+",
        metavar="COL",
        default=None,
        help="Compare gaps vs optimal across bound columns (e.g. --gap Relax_LB Root_LB Final_LB Initial_UB)",
    )
    parser.add_argument(
        "--best-known",
        default=_DEFAULT_BEST_KNOWN,
        help="Path to best_known.csv (default: results/best_known.csv in repo root)",
    )
    parser.add_argument(
        "--domain", default="", help="Filter instances by domain substring"
    )
    parser.add_argument(
        "--points", action="store_true", help="Overlay individual data points"
    )
    parser.add_argument(
        "--solved", action="store_true", help="Keep only instances solved by all runs"
    )
    parser.add_argument(
        "--violin",
        action="store_true",
        help="Violin plot instead of boxplot (requires --gap)",
    )
    parser.add_argument(
        "--diff",
        action="store_true",
        help="Paired-difference violin vs baseline (requires --gap, implies --violin)",
    )
    parser.add_argument(
        "--global-scale",
        action="store_true",
        help="Normalise all violin widths to a single global max density (requires --violin or --pairs)",
    )
    parser.add_argument(
        "--pairs",
        nargs="+",
        metavar="I,J",
        default=None,
        help="Split violin: pairs of file indices (e.g. --pairs 0,2 1,3); requires --violin and --gap",
    )
    parser.add_argument(
        "--time",
        type=float,
        default=None,
        metavar="T",
        help="Exclude instances where any run exceeded T seconds",
    )
    parser.add_argument(
        "--out", default="boxplot.pdf", help="Output file (default: boxplot.pdf)"
    )
    args = parser.parse_args()

    if args.violin and not args.gap:
        parser.error("--violin requires --gap")
    if args.diff and not args.gap:
        parser.error("--diff requires --gap")
    if args.pairs and not args.gap:
        parser.error("--pairs requires --gap")
    if args.pairs and not args.violin:
        parser.error("--pairs requires --violin")
    if args.pairs and args.diff:
        parser.error("--pairs and --diff are mutually exclusive")

    if args.gap and args.metric != "Time":
        parser.error("--gap and --metric are mutually exclusive.")

    aliases = resolve_aliases(args.files, args.aliases)

    def _parse_pairs(pair_strs: list[str]) -> list[tuple[str, str]]:
        result = []
        for s in pair_strs:
            parts = s.split(",")
            if len(parts) != 2:
                parser.error(f"invalid pair {s!r}, expected format I,J")
            try:
                i, j = int(parts[0]), int(parts[1])
            except ValueError:
                parser.error(f"invalid pair {s!r}, indices must be integers")
            if not (0 <= i < len(aliases) and 0 <= j < len(aliases)):
                parser.error(f"pair indices {i},{j} out of range (0..{len(aliases)-1})")
            result.append((aliases[i], aliases[j]))
        return result

    if args.gap:
        data = prepare_data(
            args.files,
            aliases,
            domain=args.domain,
            extra_cols=args.gap,
            solved_only=args.solved,
            max_time=args.time,
        )
        data = compute_gaps(data, args.gap, args.best_known)
        if args.pairs:
            pairs = _parse_pairs(args.pairs)
            plot = split_violin_plot(
                data,
                pairs=pairs,
                group_col="Phase",
                value_col="Gap",
                group_order=args.gap,
                title="Optimality Gap by Bound",
                x_label="Bound",
                y_label="Gap to optimal (%)",
                show_points=args.points,
                global_scale=args.global_scale,
            )
            width = max(8, len(args.gap) * 1.5 * len(pairs))
        elif args.diff:
            data = compute_gap_diffs(data, baseline=aliases[0])
            comparisons = [f"{a} − {aliases[0]}" for a in aliases[1:]]
            plot = violin_plot(
                data,
                group_col="Phase",
                value_col="Difference",
                fill_col="Comparison",
                title=f"Gap Difference vs {aliases[0]} by Bound",
                x_label="Bound",
                y_label=f"Gap difference vs {aliases[0]} (pp)",
                show_points=args.points,
                group_order=args.gap,
                fill_order=comparisons,
                reference_y=0.0,
            )
            width = max(8, len(args.gap) * 1.5 * len(aliases))
        else:
            if args.violin:
                plot = violin_plot(
                    data,
                    group_col="Phase",
                    value_col="Gap",
                    fill_col="Model",
                    title="Optimality Gap by Bound",
                    x_label="Bound",
                    y_label="Gap to optimal (%)",
                    show_points=args.points,
                    group_order=args.gap,
                    fill_order=aliases,
                    global_scale=args.global_scale,
                )
            else:
                plot = boxplot(
                    data,
                    group_col="Phase",
                    value_col="Gap",
                    fill_col="Model",
                    title="Optimality Gap by Bound",
                    x_label="Bound",
                    y_label="Gap to optimal (%)",
                    show_points=args.points,
                    group_order=args.gap,
                    fill_order=aliases,
                )
            width = max(8, len(args.gap) * 1.5 * len(aliases))
    else:
        extra = [args.metric] if args.metric not in ("Time", "Nodes") else None
        data = prepare_data(
            args.files,
            aliases,
            domain=args.domain,
            extra_cols=extra,
            solved_only=args.solved,
            max_time=args.time,
        )
        plot = boxplot(
            data,
            group_col="Model",
            value_col=args.metric,
            title=f"{args.metric} Distribution",
            x_label="Run",
            y_label=args.metric,
            show_points=args.points,
            group_order=aliases,
        )
        width = max(6, len(aliases) * 1.5)

    save(plot, args.out, width=width, height=5)
    print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
