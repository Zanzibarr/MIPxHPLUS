"""
Script used in pair with run_batch.py to aggregate the results of a batch into a csv file

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0 -> 4.2.0
"""

import re
from pathlib import Path

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
    # --- Cuts added via Relaxional (LP-relaxation) callback ---
    "N_Relax_Calls",  # number of Relaxional callback invocations
    "N_Relax_LM",  # LM cuts added
    "N_Relax_Sec",  # SEC cuts added
    # --- Cut sizes (mean number of non-zeros, from Gauge table) ---
    "Cand_LM_Size_Mean",
    "Relax_LM_Size_Mean",
    # --- LM-Cut sizes ---
    "LM_Cut_N_Landmarks",
    "LM_Cut_AvgSize_Landmarks",
    # --- Other search counters ---
    "N_Nodes",  # branch-and-bound nodes expanded
    # --- Bounds ---
    "Relax_LB",  # relaxation lower bound
    "Root_LB",  # root-node lower bound
    "Final_LB",  # final lower bound (from Results block)
    "Initial_UB",  # heuristic upper bound (from Results block)
    "Final_UB",  # final upper bound / solution cost (from Results block)
    # --- Timers (all in seconds) ---
    "Pars_File_Time",
    "Prep_Time",
    "LMCut_Time",
    "Heur_Time",
    "Build_Time",
    "Relax_CB_Time",  # Relaxional callback total
    "Relax_LM_Sep_Time",
    "Relax_Sec_Sep_Time",
    "Cand_CB_Time",  # candidate callback total
    "Cand_LM_Sep_Time",
    "Cand_Sec_Sep_Time",
    "CPX_Time",  # total CPLEX MIP solve time
    "Time",  # total wall-clock time
]

DEFAULT_ROW = {
    "Instance": "",
    "Status": -1,
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
    "N_Relax_Calls": None,
    "N_Relax_LM": None,
    "N_Relax_Sec": None,
    "Cand_LM_Size_Mean": None,
    "Relax_LM_Size_Mean": None,
    "LM_Cut_N_Landmarks": None,
    "LM_Cut_AvgSize_Landmarks": None,
    "N_Nodes": None,
    "Relax_LB": None,
    "Root_LB": None,
    "Final_LB": None,
    "Initial_UB": None,
    "Final_UB": None,
    "Pars_File_Time": None,
    "Prep_Time": None,
    "LMCut_Time": None,
    "Heur_Time": None,
    "Build_Time": None,
    "Relax_CB_Time": None,
    "Relax_LM_Sep_Time": None,
    "Relax_Sec_Sep_Time": None,
    "Cand_CB_Time": None,
    "Cand_LM_Sep_Time": None,
    "Cand_Sec_Sep_Time": None,
    "CPX_Time": None,
    "Time": None,
}


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
                try:
                    timers[parts[0]] = float(
                        parts[3]
                    )  # columns: name threads calls TOTAL ...
                except ValueError:
                    break  # tables are not blank-line separated: next table's header marks the end

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
                try:
                    counters[parts[0]] = int(parts[1])  # columns: name VALUE
                except ValueError:
                    break  # tables are not blank-line separated: next table's header marks the end
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
                try:
                    gauges[parts[0]] = float(
                        parts[3]
                    )  # columns: name samples total MEAN ...
                except ValueError:
                    break  # tables are not blank-line separated: next table's header marks the end
    return gauges


def parse_regex(regex, content, default=None):
    m = re.search(regex, content)
    return default if not m else m.group(1)


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
        return None

    if (
        "Version:" not in content  # log is truncated / not from hplus
        or "[SUCCESS] Execution terminated" not in content
        or "[ FATAL ]" in content  # any other error
    ):
        # Status stays at -1
        return row

    # --- Parse the three table blocks ---
    timers, divisor = parse_timer_table(content)
    counters = parse_counter_table(content)
    gauges = parse_gauge_table(content)

    # Shorthand: convert a timer value to seconds using the parsed unit
    def t(name):
        return timers.get(name, 0) / divisor

    # --- Preprocessed instance size (Counter table) ---
    row["N_Atoms"] = int(parse_regex(r"# facts:\s*(\S+)", content, 0))
    row["N_Acts"] = int(parse_regex(r"# actions:\s*(\S+)", content, 0))
    row["N_Effects"] = int(parse_regex(r"# first adders:\s*(\S+)", content, 0))

    # --- MIP model size (Counter table) ---
    row["N_Var_Base"] = counters.get("n_var_base", None)
    row["N_Var_Acyc"] = counters.get("n_var_acyc", None)
    row["N_Const_Base"] = counters.get("n_const_base", None)
    row["N_Const_Acyc"] = counters.get("n_const_acyc", None)

    # --- Cuts from candidate (integer solution) callback (Counter table) ---
    row["N_Cand_Calls"] = counters.get("cand_calls", 0)
    row["N_Cand_LM"] = counters.get("cand_lm", 0)
    row["N_Cand_Sec"] = counters.get("cand_sec", 0)

    # --- Cuts from Relaxional (LP relaxation) callback (Counter table) ---
    row["N_Relax_Calls"] = counters.get("relax_calls", 0)
    row["N_Relax_LM"] = counters.get("relax_lm", 0)
    row["N_Relax_Sec"] = counters.get("relax_sec", 0)

    # --- Cut sizes (Gauge table, mean number of non-zeros per cut) ---
    row["Cand_LM_Size_Mean"] = gauges.get("cand_lm_size", None)
    row["Relax_LM_Size_Mean"] = gauges.get("relax_lm_size", None)

    # --- LM-Cut sizes ---
    row["LM_Cut_N_Landmarks"] = gauges.get("lmcut.size", None)
    row["LM_Cut_AvgSize_Landmarks"] = gauges.get("lmcut.lm.avgsize", None)

    # --- Other search counters (Counter table) ---
    row["N_Nodes"] = counters.get("nodes", 0)

    # --- Bounds (Results block, >> Label value << format) ---
    row["Final_LB"] = float(parse_regex(r"Best bound: (\S+)", content, 0))
    row["Final_UB"] = float(parse_regex(r"Best incumbent: (\S+)", content, 1e20))
    row["Initial_UB"] = (
        float(1e20)
        if re.search(r"primal-heur\s+string\s+\"0\"", content) is not None
        else float(parse_regex(r"Updated best solution: (\S+)", content, 1e20))
    )

    # Lower bound stats
    row["Relax_LB"] = gauges.get("lb_relaxation", None)
    row["Root_LB"] = gauges.get("lb_rootnode", None)

    # Status:
    # -1: error
    # 0: correct execution
    row["Status"] = 0

    # --- Timers (Timer table, converted to seconds) ---
    row["Pars_File_Time"] = t("read-instance")
    row["Prep_Time"] = t("preprocess")
    row["LMCut_Time"] = t("lmcut")
    row["Heur_Time"] = t("heur")
    row["Build_Time"] = t("build")
    row["Relax_CB_Time"] = t("relax_callback")
    row["Relax_LM_Sep_Time"] = t("relax_lm_sep")
    row["Relax_Sec_Sep_Time"] = t("relax_sec_sep")
    row["Cand_CB_Time"] = t("cand_callback")
    row["Cand_LM_Sep_Time"] = t("cand_lm_sep")
    row["Cand_Sec_Sep_Time"] = t("cand_sec_sep")
    row["CPX_Time"] = t("cplex")
    row["Time"] = t("total")

    return row
