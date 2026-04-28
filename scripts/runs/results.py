import sys

sys.dont_write_bytecode = True

import csv, os, re
from pathlib import Path

SCRIPT_FOLDER = Path(__file__).parent
ROOT = SCRIPT_FOLDER.parent.parent
SCRIPTS_LOCATION = ROOT / "scripts"

# ---------------------------------------------------------------------------
# Log format reference (hplus v3.1.7 -> v3.1.7.1)
#
# The log is structured in four output blocks, each delimited by a header line:
#
#   "------------------------- Results ----------------------"
#       Lines of the form:  >> Label   value <<
#       Fields: Status, Lower bound, Heuristic, Final cost
#
#   "Timer   Threads   Calls   Total(<unit>)   ..."
#   "----..."
#       One row per timer: name  threads  calls  total  mean  min  max  stddev
#       The time unit is embedded in the column header, e.g. Total(us).
#       Only the Total column is used here.
#
#   "Counter   Value"
#   "----..."
#       One row per counter: name  value
#
#   "Gauge   Samples   Total   Mean   Min   Max   Stddev"
#   "----..."
#       One row per gauge: name  samples  total  mean  min  max  stddev
#       Only the Mean column is used here.
#
# Two free-text lines are also parsed from the full log content:
#   "Lower bound at start of cutloop: <value>"
#   "Lower bound at end of cutloop:   <value>"
#
# Status codes:
#   -3  unsupported axiom layers
#   -2  unknown / file did not complete normally  (default)
#   -1  out of memory
#    0  optimal (lb == ub, or CPLEX proved optimality)
#    4  aborted (treated as a separate early-exit case)
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Parsers for the three table blocks
# ---------------------------------------------------------------------------


def parse_timer_table(content):
    """Parse the Timer table from log content.

    Returns (timers, divisor) where:
      - timers   is a dict mapping timer name -> Total value (in the log's native unit)
      - divisor  is the factor to divide by to get seconds (e.g. 1_000_000 for 'us')

    To add a new timer column: just read timers.get("<log_name>", 0) / divisor below.
    """
    divisor = 1_000_000  # fallback: assume microseconds

    # The unit is encoded in the column header, e.g. "Total(us)" or "Total(ms)"
    header_m = re.search(r"Timer\s+Threads\s+Calls\s+Total\((\w+)\)", content)
    if header_m:
        unit = header_m.group(1)
        divisor = {"ns": 1_000_000_000, "us": 1_000_000, "ms": 1_000, "s": 1}.get(
            unit, 1_000_000
        )
    else:
        print("ERROR IN FINDING UNIT IN TIMER TABLE")
        exit(1)

    timers = {}
    section_m = re.search(
        r"Timer\s+Threads[^\n]*\n-+\n(.*?)(?:\n\n|\Z)", content, re.DOTALL
    )
    if section_m:
        for line in section_m.group(1).strip().split("\n"):
            parts = line.split()
            if len(parts) >= 4:
                timers[parts[0]] = float(
                    parts[3]
                )  # columns: name threads calls TOTAL ...

    return timers, divisor


def parse_counter_table(content):
    """Parse the Counter table from log content.

    Returns a dict mapping counter name -> int value.
    To add a new counter column: just read counters.get("<log_name>", 0) below.
    """
    counters = {}
    section_m = re.search(r"Counter\s+Value\n-+\n(.*?)(?:\n\n|\Z)", content, re.DOTALL)
    if section_m:
        for line in section_m.group(1).strip().split("\n"):
            parts = line.split()
            if len(parts) >= 2:
                counters[parts[0]] = int(parts[1])  # columns: name VALUE
    return counters


def parse_gauge_table(content):
    """Parse the Gauge table from log content.

    Returns a dict mapping gauge name -> mean value (float).
    To add a new gauge column: just read gauges.get("<log_name>", 0) below.
    """
    gauges = {}
    section_m = re.search(
        r"Gauge\s+Samples.*\n-+\n(.*?)(?:\n\n|\Z)", content, re.DOTALL
    )
    if section_m:
        for line in section_m.group(1).strip().split("\n"):
            parts = line.split()
            if len(parts) >= 4:
                gauges[parts[0]] = float(
                    parts[3]
                )  # columns: name samples total MEAN ...
    return gauges


# ---------------------------------------------------------------------------
# CSV schema
#
# FIELDNAMES defines the output columns and their order.
# DEFAULT_ROW defines the default value for each field when data is absent.
# Both must be kept in sync when adding or removing columns.
# ---------------------------------------------------------------------------

FIELDNAMES = [
    "Instance",
    "Status",
    # --- Preprocessed instance size ---
    "N_Atoms",  # atoms (facts) after preprocessing
    "N_Acts",  # actions after preprocessing
    "N_Effects",  # add-effects after preprocessing
    # --- MIP model size ---
    "N_Var_Base",  # variables in the base model
    "N_Var_Acyc",  # additional acyclicity variables (0 if not present)
    "N_Const_Base",  # constraints in the base model
    "N_Const_Acyc",  # additional acyclicity constraints
    # --- Cuts added via candidate (integer) callback ---
    "N_Cand_Calls",  # number of candidate callback invocations
    "N_Cand_LM",  # LM cuts added
    "N_Cand_Sec",  # SEC cuts added
    # --- Cuts added via fractional (LP-relaxation) callback ---
    "N_Fract_Calls",  # number of fractional callback invocations
    "N_Fract_LM",  # LM cuts added
    "N_Fract_Sec",  # SEC cuts added
    # --- Aggregate cut totals (cand + fract) ---
    "N_Cuts_LM",
    "N_Cuts_Sec",
    # --- Cut sizes (mean number of non-zeros, from Gauge table) ---
    "Cand_LM_Size_Mean",
    "Fract_LM_Size_Mean",
    # --- Other search counters ---
    "N_CL_It",  # custom cut-loop iterations
    "N_Nodes",  # branch-and-bound nodes expanded
    # --- Bounds ---
    "Relax_LB",  # relaxation lower bound
    "Cutloop_LB",  # cut-loop lower bound
    "Root_LB",  # root-node lower bound
    "Final_LB",  # final lower bound (from Results block)
    "Initial_UB",  # heuristic upper bound (from Results block)
    "Final_UB",  # final upper bound / solution cost (from Results block)
    # --- Timers (all in seconds) ---
    "Pars_CLI_Time",
    "Pars_File_Time",
    "Prep_Time",
    "LMCut_Time",
    "Heur_Time",
    "Build_Time",
    "CL_Time",  # custom cut-loop
    "Fract_CB_Time",  # fractional callback total
    "Fract_LM_Sep_Time",
    "Fract_Sec_Sep_Time",
    "Cand_CB_Time",  # candidate callback total
    "Cand_LM_Sep_Time",
    "Cand_Sec_Sep_Time",
    "CPX_Time",  # total CPLEX MIP solve time
    "Time",  # total wall-clock time
]

DEFAULT_ROW = {
    "Instance": "",
    "Status": -2,
    "N_Atoms": None,
    "N_Acts": None,
    "N_Effects": None,
    "N_Var_Base": None,
    "N_Var_Acyc": None,
    "N_Const_Base": None,
    "N_Const_Acyc": None,
    "N_Cand_Calls": None,
    "N_Cand_LM": None,
    "N_Cand_Sec": None,
    "N_Fract_Calls": None,
    "N_Fract_LM": None,
    "N_Fract_Sec": None,
    "N_Cuts_LM": None,
    "N_Cuts_Sec": None,
    "Cand_LM_Size_Mean": None,
    "Fract_LM_Size_Mean": None,
    "N_CL_It": None,
    "N_Nodes": None,
    "Relax_LB": None,
    "Cutloop_LB": None,
    "Root_LB": None,
    "Final_LB": None,
    "Initial_UB": None,
    "Final_UB": None,
    "Pars_CLI_Time": None,
    "Pars_File_Time": None,
    "Prep_Time": None,
    "LMCut_Time": None,
    "Heur_Time": None,
    "Build_Time": None,
    "CL_Time": None,
    "Fract_CB_Time": None,
    "Fract_LM_Sep_Time": None,
    "Fract_Sec_Sep_Time": None,
    "Cand_CB_Time": None,
    "Cand_LM_Sep_Time": None,
    "Cand_Sec_Sep_Time": None,
    "CPX_Time": None,
    "Time": None,
}


# ---------------------------------------------------------------------------
# Per-file parsing
# ---------------------------------------------------------------------------


def parse_log(filepath):
    """Parse a single log file and return a fully populated row dict.

    The function applies early-exit guards for known failure conditions before
    attempting to parse the structured output blocks.
    """
    row = dict(DEFAULT_ROW)
    p = Path(filepath)
    suffixes = "".join(p.suffixes)
    row["Instance"] = p.name[: -len(suffixes)] if suffixes else p.name

    with open(filepath, "r", errors="replace") as f:
        content = f.read()

    # --- Early-exit guards (checked in order of specificity) ---

    if "Axiom layer is" in content:
        # Instance uses axiom layers, which are not supported
        row["Status"] = -3
        return row

    if "[ ERROR ] OUT OF MEMORY" in content:
        row["Status"] = -1
        return row

    if (
        "Version:" not in content  # log is truncated / not from hplus
        or "[SUCCESS] Execution terminated" not in content
        or "[ ERROR ]" in content  # any other error
    ):
        # Status stays at -2 (unknown)
        return row

    # --- Locate the Results block ---
    # Everything after the header line up to end-of-file
    results = content.partition(
        "------------------------- Results ----------------------"
    )[2]
    if not results:
        return row  # Results block missing; leave Status = -2

    row["Status"] = int(re.search(r">> Status\s+(\d+) <<", results).group(1))

    if row["Status"] == 4:
        # Aborted before producing useful data
        return row

    # --- Parse the three table blocks ---
    timers, divisor = parse_timer_table(content)
    counters = parse_counter_table(content)
    gauges = parse_gauge_table(content)

    # Shorthand: convert a timer value to seconds using the parsed unit
    def t(name):
        return timers.get(name, 0) / divisor

    # --- Preprocessed instance size (Counter table) ---
    row["N_Atoms"] = counters.get("n_prep", 0)
    row["N_Acts"] = counters.get("m_prep", 0)
    row["N_Effects"] = counters.get("nfadd_prep", 0)

    # --- MIP model size (Counter table) ---
    row["N_Var_Base"] = counters.get("n_var_base", 0)
    row["N_Var_Acyc"] = counters.get("n_var_acyc", 0)
    row["N_Const_Base"] = counters.get("n_const_base", 0)
    row["N_Const_Acyc"] = counters.get("n_const_acyc", 0)

    # --- Cuts from candidate (integer solution) callback (Counter table) ---
    row["N_Cand_Calls"] = counters.get("cand_calls", 0)
    row["N_Cand_LM"] = counters.get("cand_lm", 0)
    row["N_Cand_Sec"] = counters.get("cand_sec", 0)

    # --- Cuts from fractional (LP relaxation) callback (Counter table) ---
    row["N_Fract_Calls"] = counters.get("fract_calls", 0)
    row["N_Fract_LM"] = counters.get("fract_lm", 0)
    row["N_Fract_Sec"] = counters.get("fract_sec", 0)

    # --- Aggregate cut totals ---
    row["N_Cuts_LM"] = row["N_Cand_LM"] + row["N_Fract_LM"]
    row["N_Cuts_Sec"] = row["N_Cand_Sec"] + row["N_Fract_Sec"]

    # --- Cut sizes (Gauge table, mean number of non-zeros per cut) ---
    row["Cand_LM_Size_Mean"] = gauges.get("cand_lm_size", None)
    row["Fract_LM_Size_Mean"] = gauges.get("fract_lm_size", None)

    # --- Other search counters (Counter table) ---
    row["N_CL_It"] = counters.get("cloop_it", 0)
    row["N_Nodes"] = counters.get("nodes", 0)

    # --- Bounds (Results block, >> Label value << format) ---
    row["Final_LB"] = float(
        re.search(r">> Lower bound\s+([\d.]+(?:e[+-]?\d+)?) <<", results).group(1)
    )
    m = re.search(r">> Heuristic\s+([\d.]+(?:e[+-]?\d+)?) <<", results)
    if m:
        val = m.group(1)
        row["Initial_UB"] = 1e20 if val == "1e20" else int(float(val))
    m = re.search(r">> Final cost\s+([\d.]+(?:e[+-]?\d+)?) <<", results)
    if m:
        val = m.group(1)
        row["Final_UB"] = 1e20 if val == "1e20" else int(float(val))

    # Lower bound stats
    row["Relax_LB"] = gauges.get("lb_relaxation", None)
    row["Cutloop_LB"] = gauges.get("lb_cutloop", None)
    row["Root_LB"] = gauges.get("lb_root", None)

    # --- Timers (Timer table, converted to seconds) ---
    row["Pars_CLI_Time"] = t("parsing.cli")
    row["Pars_File_Time"] = t("parsing.file")
    row["Prep_Time"] = t("preprocessing")
    row["LMCut_Time"] = t("lmcut")
    row["Heur_Time"] = t("heuristic")
    row["Build_Time"] = t("build")
    row["CL_Time"] = t("cutloop")
    row["Fract_CB_Time"] = t("fract_callback")
    row["Fract_LM_Sep_Time"] = t("fract_lm_separator")
    row["Fract_Sec_Sep_Time"] = t("fract_sec_separator")
    row["Cand_CB_Time"] = t("cand_callback")
    row["Cand_LM_Sep_Time"] = t("cand_lm_separator")
    row["Cand_Sec_Sep_Time"] = t("cand_sec_separator")
    row["CPX_Time"] = t("cpx_execution")
    row["Time"] = t("total")

    return row


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    if len(sys.argv) != 2 or sys.argv[1] in ["-h", "--h", "-help", "--help"]:
        print(f"Usage: python3 {os.path.basename(__file__)} <run_name>")
        exit(0)

    run_name = sys.argv[1]
    rundir = SCRIPTS_LOCATION / run_name
    logsdir = rundir / "logs"

    assert rundir.exists(), f"run directory not found: {rundir}"
    assert logsdir.exists(), f"logs directory not found: {logsdir}"

    log_files = sorted(logsdir.glob("*.log"))
    assert len(log_files) > 0, f"no log files found in {logsdir}"

    csv_path = rundir / f"{run_name}.csv"

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        for i, log in enumerate(log_files, 1):
            print(f"[{i}/{len(log_files)}] {log.stem}")
            writer.writerow(parse_log(log))


if __name__ == "__main__":
    main()
