import sys

sys.dont_write_bytecode = True

import csv
import os
import re
from pathlib import Path

# ---------------------------------------------------------------------------
# Parses FICO Xpress optimizer console logs (produced by run.py's Xpress block)
# into a CSV matching local/old_results/test_gurobi_nomst.csv:
#
#     Instance, Status, N_Nodes, Time, Final_UB
#
# The run.py Xpress block feeds the optimizer these commands after solving and
# appends a wall-clock marker:
#
#     mipoptimize
#     MIPSTATUS      -> echoed value, mapped to our Status convention
#     NODES          -> N_Nodes
#     MIPOBJVAL      -> Final_UB (best integer objective)
#     BESTBOUND      -> (parsed but unused here)
#     >> XPRESS_WALLTIME_MS <ms> <<   -> Time (seconds)
#
# !!! FORMAT ASSUMPTION -- verify against a real log !!!
# NODES / MIPOBJVAL / MIPSTATUS are read from the console's *echo* of each
# attribute name. ATTR_RE assumes the console prints them as
#     <NAME>[: or =]<value>
# one per line, uppercase name. If the produced logs differ, this is the ONLY
# regex that needs adjusting. Time is independent of this (shell wall-clock).
# ---------------------------------------------------------------------------

FIELDNAMES = ["Instance", "Status", "N_Nodes", "Time", "Final_UB"]

NO_SOLUTION = 1e20  # sentinel objective when no integer solution was found

# Status convention (shared with the Gurobi/CPLEX result CSVs):
#    0  optimal (proven)
#    2  stopped early (time / memory limit)
#   -1  out of memory
#   -2  unknown / log incomplete
#
# Xpress MIPSTATUS -> our Status:
XPRESS_STATUS = {
    6: 0,  # XPRS_MIP_OPTIMAL
    4: 2,  # XPRS_MIP_SOLUTION      (feasible found, search stopped)
    3: 2,  # XPRS_MIP_NO_SOL_FOUND  (search stopped, no feasible)
    5: -2,  # XPRS_MIP_INFEAS        (h+ instances are always feasible)
}

DEFAULT_ROW = {
    "Instance": "",
    "Status": -2,
    "N_Nodes": None,
    "Time": None,
    "Final_UB": None,
}


def attr(content, name):
    """Best-effort read of the console-echoed value of an Xpress attribute."""
    m = re.search(
        rf"(?mi)^[ \t]*{name}[ \t]*[:=]?[ \t]*(-?\d[\d.eE+-]*)[ \t]*$", content
    )
    return m.group(1) if m else None


def parse_log(filepath):
    row = dict(DEFAULT_ROW)
    row["Instance"] = Path(filepath).stem

    with open(filepath, "r", errors="replace") as f:
        content = f.read()

    # Time (seconds) from the wall-clock marker appended by run.py
    m = re.search(r">> XPRESS_WALLTIME_MS\s+(\d+) <<", content)
    if m:
        row["Time"] = int(m.group(1)) / 1000.0

    if re.search(r"(?i)out of memory|insufficient memory", content):
        row["Status"] = -1
        return row

    raw = attr(content, "MIPSTATUS")
    if raw is None:
        return row  # incomplete log -> Status -2
    mipstatus = int(float(raw))
    row["Status"] = XPRESS_STATUS.get(mipstatus, -2)

    nodes = attr(content, "NODES")
    if nodes is not None:
        row["N_Nodes"] = int(float(nodes))

    obj = attr(content, "MIPOBJVAL")
    if obj is not None:
        val = float(obj)
        row["Final_UB"] = NO_SOLUTION if abs(val) >= 1e19 else val
    if mipstatus == 3:  # no feasible solution found
        row["Final_UB"] = NO_SOLUTION

    return row


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--h", "-help", "--help"):
        print(f"Usage: python3 {os.path.basename(__file__)} <logs_dir> [out.csv]")
        exit(0)

    logsdir = Path(sys.argv[1])
    assert logsdir.is_dir(), f"logs directory not found: {logsdir}"

    log_files = sorted(logsdir.glob("*.log"))
    assert log_files, f"no log files found in {logsdir}"

    csv_path = (
        Path(sys.argv[2])
        if len(sys.argv) > 2
        else logsdir.parent / f"{logsdir.name}.csv"
    )

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        for i, log in enumerate(log_files, 1):
            print(f"[{i}/{len(log_files)}] {log.stem}")
            writer.writerow(parse_log(log))

    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    main()
