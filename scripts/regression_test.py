"""
Script used to compare differences in execution between incremental versions

Used for development of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0
"""

import argparse
import math
import re
import subprocess
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
ROOT = SCRIPTS_DIR.parent
RUN_BATCH = SCRIPTS_DIR / "run_batch.py"
DEFAULT_RUNS_DIR = ROOT / "local" / "experiments"
DEFAULT_NEW_EXE = ROOT / "code" / "build" / "hplus"

sys.path.insert(0, str(SCRIPTS_DIR))
from parse_results import parse_log

# Fields (from parse_results rows) that must match between the two executables
COMPARED_FIELDS = [
    "N_Var_Base",
    "N_Var_Acyc",
    "N_Const_Base",
    "N_Const_Acyc",
    "N_Cand_Calls",
    "N_Cand_LM",
    "N_Cand_Sec",
    "N_Relax_Calls",
    "N_Relax_LM",
    "N_Relax_Sec",
    "Cand_LM_Size_Mean",
    "Relax_LM_Size_Mean",
    "N_Nodes",
    "Relax_LB",
    "Root_LB",
    "Final_LB",
    "Initial_UB",
    "Final_UB",
]

# e.g. "[00:00:01.284] [ 315028 KB] [T:0800] [ DEBUG ] Updated best bound: 226.5"
UPDATE_RE = re.compile(r"Updated best (solution|bound): (\S+)")

# Logged by the solver when the time-limit alarm fires: runs that breached the
# time limit legitimately differ (wall-clock noise), so their diff is skipped
TIMEOUT_MARKER = "Time limit reached."


def parse():
    parser = argparse.ArgumentParser(
        description="Run the same random batch with two hplus executables (bash mode) "
        "and report the instances whose runs differ.",
        epilog="Any unrecognised flags are forwarded verbatim to hplus.",
    )
    parser.add_argument("runname", type=str)
    parser.add_argument(
        "--old",
        type=str,
        required=True,
        help="path to the OLD hplus executable",
    )
    parser.add_argument(
        "--new",
        type=str,
        default=str(DEFAULT_NEW_EXE),
        help=f"path to the NEW hplus executable (default: {DEFAULT_NEW_EXE})",
    )
    parser.add_argument(
        "--rundir",
        type=str,
        default=None,
        help="base directory for run folders (default: ../local/experiments)",
    )
    parser.add_argument("--instances", type=str, required=True)
    parser.add_argument(
        "--n",
        type=int,
        default=None,
        help="randomly sample n instances (omit to use all)",
    )
    parser.add_argument(
        "--sample-seed",
        type=int,
        default=0,
        dest="sample_seed",
        help="random seed for --n instance sampling (default: 0)",
    )
    parser.add_argument(
        "--timelimit",
        type=int,
        default=None,
        help="solver time limit in seconds (default: run_batch default)",
    )
    parser.add_argument(
        "--threads",
        type=int,
        default=None,
        help="number of solver threads (default: run_batch default)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="solver random seed (default: run_batch default)",
    )
    args, extra = parser.parse_known_args()
    args.commands = extra
    return args


def run_batch(args, regdir, label, exe):
    """Write and execute one bash batch named `label` inside `regdir` using `exe`."""
    write_cmd = [
        sys.executable,
        str(RUN_BATCH),
        label,
        "--rundir",
        str(regdir),
        "--exe",
        str(exe),
        "--bash",
        "--instances",
        args.instances,
        "--sample-seed",
        str(args.sample_seed),
    ]
    if args.n is not None:
        write_cmd += ["--n", str(args.n)]
    if args.timelimit is not None:
        write_cmd += ["--timelimit", str(args.timelimit)]
    if args.threads is not None:
        write_cmd += ["--threads", str(args.threads)]
    if args.seed is not None:
        write_cmd += ["--seed", str(args.seed)]
    write_cmd += args.commands
    subprocess.run(write_cmd, check=True)

    print(f"Running batch '{label}' ({exe})")
    run_cmd = [
        sys.executable,
        str(RUN_BATCH),
        label,
        "--rundir",
        str(regdir),
        "--mode",
        "run",
    ]
    r = subprocess.run(run_cmd)
    if r.returncode != 0:
        print(
            f"warning: some jobs of batch '{label}' exited with a non-zero status",
            file=sys.stderr,
        )


def read_log(logpath):
    with open(logpath, "r", errors="replace") as f:
        return f.read()


def parse_updates(content):
    """Return the ordered list of bound/solution updates as (kind, value) tuples."""
    return [(kind, float(value)) for kind, value in UPDATE_RE.findall(content)]


def format_updates(seq):
    if not seq:
        return "(none)"
    return ", ".join(
        f"{'sol' if kind == 'solution' else 'bound'} {value:g}" for kind, value in seq
    )


def values_differ(old, new):
    if old is None and new is None:
        return False
    if old is None or new is None:
        return True
    if isinstance(old, float) or isinstance(new, float):
        return not math.isclose(float(old), float(new), rel_tol=1e-9, abs_tol=1e-9)
    return old != new


def diff_instance(old_log, new_log):
    """Compare the two runs of one instance.

    Returns a list of human-readable difference lines (empty if the runs match),
    or None if either run breached the time limit (diff not meaningful).
    """
    old_row = parse_log(old_log)
    new_row = parse_log(new_log)

    # parse_log returns None for unsupported instances (axiom layers)
    if old_row is None and new_row is None:
        return []
    if old_row is None or new_row is None:
        side = "old" if old_row is None else "new"
        return [f"unsupported/unparsable instance on the {side} side only"]

    old_content = read_log(old_log)
    new_content = read_log(new_log)
    if TIMEOUT_MARKER in old_content or TIMEOUT_MARKER in new_content:
        return None

    if old_row["Status"] != 0 or new_row["Status"] != 0:
        return [
            f"Status: old={old_row['Status']} new={new_row['Status']}"
            " (failed execution, counters not compared)"
        ]

    lines = []
    for field in COMPARED_FIELDS:
        if values_differ(old_row[field], new_row[field]):
            lines.append(f"{field}: old={old_row[field]} new={new_row[field]}")

    old_upd = parse_updates(old_content)
    new_upd = parse_updates(new_content)
    if old_upd != new_upd:
        step = next(
            (i for i, (o, n) in enumerate(zip(old_upd, new_upd)) if o != n),
            min(len(old_upd), len(new_upd)),
        )
        lines.append(
            f"Updates: diverge at step {step + 1}"
            f" (old: {len(old_upd)} updates, new: {len(new_upd)})"
        )
        lines.append(f"    old: {format_updates(old_upd)}")
        lines.append(f"    new: {format_updates(new_upd)}")

    return lines


def report(regdir):
    """Compare the logs of the two batches and write/print the regression report.

    Returns the number of instances with differences.
    """
    old_logs = {p.stem: p for p in (regdir / "old" / "logs").glob("*.log")}
    new_logs = {p.stem: p for p in (regdir / "new" / "logs").glob("*.log")}

    names = sorted(set(old_logs) | set(new_logs))
    assert len(names) > 0, f"no logs found in {regdir}"

    blocks = []
    skipped = []
    for name in names:
        if name not in old_logs or name not in new_logs:
            side = "old" if name not in old_logs else "new"
            blocks.append((name, [f"missing log for the {side} executable"]))
            continue
        lines = diff_instance(old_logs[name], new_logs[name])
        if lines is None:
            skipped.append(name)
        elif lines:
            blocks.append((name, lines))

    out = [
        f"Compared {len(names)} instances: {len(blocks)} with differences,"
        f" {len(skipped)} skipped (time limit reached)"
    ]
    for name in skipped:
        out.append(f"    skipped: {name}")
    for name, lines in blocks:
        out.append("")
        out.append(f"{name}:")
        out.extend(f"    {line}" for line in lines)
    text = "\n".join(out) + "\n"

    print()
    print(text, end="")

    report_path = regdir / "report.txt"
    report_path.write_text(text)
    print(f"\nReport written to {report_path}")

    return len(blocks)


def main():
    args = parse()

    old_exe = Path(args.old).expanduser().resolve()
    new_exe = Path(args.new).expanduser().resolve()
    assert old_exe.exists(), f"old executable not found: {old_exe}"
    assert new_exe.exists(), f"new executable not found: {new_exe}"

    base = Path(args.rundir).expanduser().resolve() if args.rundir else DEFAULT_RUNS_DIR
    if args.rundir:
        assert base.exists(), f"--rundir not found: {base}"
    regdir = base / args.runname

    if regdir.exists():
        print(f"{regdir} already exists")
        exit(1)
    regdir.mkdir(parents=True)

    run_batch(args, regdir, "old", old_exe)
    run_batch(args, regdir, "new", new_exe)

    n_diff = report(regdir)
    sys.exit(1 if n_diff > 0 else 0)


if __name__ == "__main__":
    main()
