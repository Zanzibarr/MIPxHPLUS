import argparse
import csv
import os
import random
import re
import subprocess
import sys
import textwrap
from pathlib import Path

# --- Config ---

TIME_LIMIT = 900
MEMORY_LIMIT = 16000
THREADS = 1
SEED = 2122187

ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_RUNS_DIR = Path.home() / "jobs"
EXE_LOCATION = ROOT / "code" / "build"
HPLUS = EXE_LOCATION / "hplus"


def build_command(args):
    timelimit = args.timelimit if args.timelimit is not None else TIME_LIMIT
    memorylimit = args.memorylimit if args.memorylimit is not None else MEMORY_LIMIT
    threads = args.threads if args.threads is not None else THREADS
    seed = args.seed if args.seed is not None else SEED
    base = f"{HPLUS} --run --t={timelimit} --mem={memorylimit} --threads={threads} --s={seed}"
    if args.commands is not None:
        return f"{base} {args.commands}"
    return base


# --- Slurm config ---

SLURM_PARTITION = "arrow"
SLURM_MEM = "14GB"
SLURM_TIME = "00:20:00"
SLURM_CPUS = 4
SLURM_WCKEY = "rop"


# --- Templates ---

BASH_RUN_ALL_TEMPLATE = textwrap.dedent("""\
    #!/usr/bin/env python3
    import subprocess, sys
    from pathlib import Path

    jobs = sorted((Path(__file__).parent / 'jobs').glob('*.sh'))
    n = len(jobs)
    failed = []

    for i, job in enumerate(jobs, 1):
        print(f'[{i}/{n}] {job.stem}', flush=True)
        r = subprocess.run(['bash', str(job)])
        if r.returncode != 0:
            failed.append(job.name)

    if failed:
        print(f'Failed: {failed}', file=sys.stderr)
        sys.exit(1)
""")

SLURM_RUN_ALL_TEMPLATE = textwrap.dedent("""\
    #!/usr/bin/env python3
    import subprocess, sys, time, re
    from pathlib import Path

    def submit(job):
        r = subprocess.run(['sbatch', str(job)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        match = re.search(r'Submitted batch job (\\d+)', r.stdout)
        return match.group(1) if match else None

    jobs = sorted((Path(__file__).parent / 'jobs').glob('*.sh'))
    n = len(jobs)
    failed = []

    for i, job in enumerate(jobs, 1):
        job_id = submit(job)
        if job_id:
            print(f'[{i}/{n}] job {job_id}: {job.stem}', flush=True)
        else:
            print(f'[{i}/{n}] failed to submit: {job.stem}', flush=True)
            failed.append(job)
        time.sleep(0.1)

    while failed:
        print(f'\\nRetrying {len(failed)} failed submission(s)...', flush=True)
        time.sleep(5)
        still_failed = []
        for job in failed:
            job_id = submit(job)
            if job_id:
                print(f'  resubmitted job {job_id}: {job.stem}', flush=True)
            else:
                print(f'  failed again: {job.stem}', flush=True)
                still_failed.append(job)
            time.sleep(0.1)
        failed = still_failed
""")

SLURM_JOB_TEMPLATE = textwrap.dedent("""\
    #!/bin/bash
    #SBATCH --job-name={instance}
    #SBATCH --partition={partition}
    #SBATCH --cpus-per-task={cpus}
    #SBATCH --mem={mem}
    #SBATCH --time={time}
    #SBATCH --exclusive
    #SBATCH --wckey={wckey}
    #SBATCH --requeue
    #SBATCH --output={output_dir}/%x_%j.out
    #SBATCH --error={output_dir}/%x_%j.err
    #SBATCH --mail-type=NONE

    sudo cpupower frequency-set -g performance
    sleep 0.1
    stress-ng -c {cpus} --cpu-ops=100

    ulimit -v 16777216

    {command}
    
    # THE COMMAND ABOVE WROTE THE MPS FILE AT /home/zanellamat/thesis_hplus/logs/cpxout/lp/{instance}.mps
    
    # SOLVE THE MPS USING XPRESS
    source /nfsd/rop/sw/fico/xpressmp99/bin/xpvars.sh

    TMPFILE=$(mktemp)
    cat << EOF > $TMPFILE
    readprob /home/zanellamat/thesis_hplus/logs/cpxout/lp/{instance}.mps
    TIMELIMIT=900
    THREADS=4
    TREEMEMORYLIMIT=4050
    mipoptimize
    MIPSTATUS
    NODES
    MIPOBJVAL
    BESTBOUND
    quit
    EOF

    XPRESS_LOG=/home/zanellamat/thesis_hplus/logs/xpressout/{instance}.log
    _t0=$(date +%s%N)
    optimizer <$TMPFILE >>$XPRESS_LOG 2>&1
    echo ">> XPRESS_WALLTIME_MS $(( ($(date +%s%N) - _t0) / 1000000 )) <<" >>$XPRESS_LOG
    
    # DELETE THE MPS (THE SIZE IS TOO LARGE TO BE KEPT FOR ALL INSTANCES)
    rm /home/zanellamat/thesis_hplus/logs/cpxout/lp/{instance}.mps

    sudo cpupower frequency-set -g powersave
""")


# --- Argument parsing ---


def parse():
    parser = argparse.ArgumentParser(
        description="Manage hplus benchmark runs.",
        epilog="Any unrecognised flags are forwarded verbatim to hplus (e.g. --cloop=1 --seed=42).",
    )
    parser.add_argument("runname", type=str)
    parser.add_argument(
        "--rundir",
        type=str,
        default=None,
        help="base directory for run folders (default: ~/jobs)",
    )
    parser.add_argument("--instances", type=str)
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
        default=TIME_LIMIT,
        help=f"solver time limit in seconds (default: {TIME_LIMIT})",
    )
    parser.add_argument(
        "--memorylimit",
        type=int,
        default=MEMORY_LIMIT,
        help=f"solver memory limit in MB (default: {MEMORY_LIMIT})",
    )
    parser.add_argument(
        "--threads",
        type=int,
        default=THREADS,
        help=f"number of solver threads (default: {THREADS})",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=SEED,
        help=f"solver random seed (default: {SEED})",
    )
    parser.add_argument("--slurm", action="store_true")
    parser.add_argument(
        "--mode",
        choices=["write", "run", "both", "results"],
        default="both",
        help="write: only create scripts | run: only execute existing | both: create and run (default)",
    )
    args, extra = parser.parse_known_args()
    args.commands = " ".join(extra) if extra else None
    return args


# --- Instance loading ---


def instances(args):
    assert args.instances is not None, "--instances is required"
    assert Path(args.instances).exists(), f"instances path not found: {args.instances}"

    if Path(args.instances).is_file():
        with open(args.instances, "r") as f:
            result = f.read().splitlines()
    else:
        folder = Path(args.instances).absolute()
        result = [str(folder / f) for f in os.listdir(args.instances)]

    if args.n is not None:
        assert args.n <= len(
            result
        ), f"--n={args.n} exceeds available instances ({len(result)})"
        rng = random.Random(args.sample_seed)
        result = rng.sample(result, args.n)

    return result


# --- Script writing ---


def write_bash_scripts(args, rundir):
    assert HPLUS.exists(), f"executable not found: {HPLUS}"

    command = build_command(args)

    jobsdir = Path(rundir) / "jobs"
    jobsdir.mkdir()

    logsdir = Path(rundir) / "logs"
    logsdir.mkdir()

    cpxlogsdir = Path(rundir) / "cpxlogs"
    cpxlogsdir.mkdir()

    job_paths = []
    for inst in instances(args):
        instance = Path(inst).stem
        job = jobsdir / f"{instance}.sh"
        job.write_text(
            f"#!/bin/bash\n{command} {inst} --log={logsdir}/{instance}.log --cpxlog={cpxlogsdir}/{instance}.log\n"
        )
        job.chmod(0o755)
        job_paths.append(job)

    assert len(job_paths) > 0, "no instances found"

    run_all = Path(rundir) / "run_all.py"
    run_all.write_text(BASH_RUN_ALL_TEMPLATE)
    run_all.chmod(0o755)


def write_slurm_scripts(args, rundir):
    assert HPLUS.exists(), f"executable not found: {HPLUS}"

    command = build_command(args)

    jobsdir = Path(rundir) / "jobs"
    jobsdir.mkdir()

    logsdir = Path(rundir) / "logs"
    logsdir.mkdir()

    cpxlogsdir = Path(rundir) / "cpxlogs"
    cpxlogsdir.mkdir()

    output_dir = Path(rundir) / "jobs_output"
    output_dir.mkdir()

    job_paths = []
    for inst in instances(args):
        instance = Path(inst).stem
        job = jobsdir / f"{instance}.sh"
        job.write_text(
            SLURM_JOB_TEMPLATE.format(
                instance=instance,
                partition=SLURM_PARTITION,
                cpus=SLURM_CPUS,
                mem=SLURM_MEM,
                time=SLURM_TIME,
                wckey=SLURM_WCKEY,
                output_dir=output_dir,
                command=f"{command} {inst} --log={logsdir}/{instance}.log --cpxlog={cpxlogsdir}/{instance}.log",
            )
        )
        job.chmod(0o755)
        job_paths.append(job)

    assert len(job_paths) > 0, "no instances found"

    run_all = Path(rundir) / "run_all.py"
    run_all.write_text(SLURM_RUN_ALL_TEMPLATE)
    run_all.chmod(0o755)


def write_scripts(args, rundir):

    if rundir.exists():
        print(f"{rundir} already exists")
        exit(1)

    rundir.mkdir(parents=True)

    if args.slurm:
        write_slurm_scripts(args, rundir)
    else:
        write_bash_scripts(args, rundir)

    print(f"Written run scripts in {rundir}")


# --- Results ---


# Xpress MIPSTATUS -> Status convention (0 optimal, 2 stopped early, -1 OOM, -2 unknown)
#   6 XPRS_MIP_OPTIMAL | 4 SOLUTION (feasible, stopped) | 3 NO_SOL_FOUND (stopped) | 5 INFEAS
XPRESS_STATUS = {6: 0, 4: 2, 3: 2, 5: -2}
NO_SOLUTION = 1e20  # sentinel objective when no integer solution was found


def xpress_attr(content, name):
    """Best-effort read of the console-echoed value of an Xpress attribute.

    !! FORMAT ASSUMPTION -- verify against a real log !!
    Assumes the console echoes each queried attribute as "<NAME>[: or =]<value>",
    one per line, uppercase name. This regex is the only thing to adjust if the
    produced logs differ.
    """
    m = re.search(
        rf"(?mi)^[ \t]*{name}[ \t]*[:=]?[ \t]*(-?\d[\d.eE+-]*)[ \t]*$", content
    )
    return m.group(1) if m else None


def custom_parse(file):
    row = {
        "Instance": Path(file).stem,
        "Status": -2,
        "N_Nodes": None,
        "Time": None,
        "Final_UB": None,
    }

    with open(file, "r", errors="replace") as f:
        content = f.read()

    # Time (seconds) from the wall-clock marker appended by the job script
    m = re.search(r">> XPRESS_WALLTIME_MS\s+(\d+) <<", content)
    if m:
        row["Time"] = int(m.group(1)) / 1000.0

    if re.search(r"(?i)out of memory|insufficient memory", content):
        row["Status"] = -1
        return row

    raw = xpress_attr(content, "MIPSTATUS")
    if raw is None:
        return row  # incomplete log -> Status -2
    mipstatus = int(float(raw))
    row["Status"] = XPRESS_STATUS.get(mipstatus, -2)

    nodes = xpress_attr(content, "NODES")
    if nodes is not None:
        row["N_Nodes"] = int(float(nodes))

    obj = xpress_attr(content, "MIPOBJVAL")
    if obj is not None:
        val = float(obj)
        row["Final_UB"] = NO_SOLUTION if abs(val) >= 1e19 else val
    if mipstatus == 3:  # no feasible solution found
        row["Final_UB"] = NO_SOLUTION

    return row


def collect_results(rundir):
    sys.path.insert(0, str(Path(__file__).parent))

    assert Path(rundir).exists(), f"run directory not found: {rundir}"

    logsdir = Path("/home/zanellamat/thesis_hplus/logs/xpressout")
    assert logsdir.exists(), f"logs directory not found: {logsdir}"

    log_files = sorted(logsdir.glob("*.log"))
    assert len(log_files) > 0, f"no log files found in {logsdir}"

    run_name = Path(rundir).name
    csv_path = Path(rundir) / f"{run_name}.csv"

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["Instance", "Status", "N_Nodes", "Time", "Final_UB"]
        )
        writer.writeheader()
        for i, log in enumerate(log_files, 1):
            print(f"[{i}/{len(log_files)}] {log.stem}")
            writer.writerow(custom_parse(log))

    print(f"Results written to {csv_path}")


# --- Execution ---


def execute_scripts(rundir):
    assert Path(rundir).exists(), f"run directory not found: {rundir}"

    run_all = Path(rundir) / "run_all.py"
    assert run_all.exists(), f"run_all.py not found in {rundir}"

    r = subprocess.run(["python3", str(run_all)])
    if r.returncode != 0:
        sys.exit(r.returncode)


# --- Entry point ---


def main():
    args = parse()
    base = Path(args.rundir).expanduser().resolve() if args.rundir else DEFAULT_RUNS_DIR
    if args.rundir:
        assert base.exists(), f"--rundir not found: {base}"
    rundir = base / args.runname

    if args.mode in ("run", "results"):
        assert (
            args.instances is None
        ), f"--instances has no effect with --mode {args.mode}"
        assert (
            args.commands is None
        ), f"--commands has no effect with --mode {args.mode}"
        assert not args.slurm, f"--slurm has no effect with --mode {args.mode}"

    if args.mode in ("write", "both"):
        write_scripts(args, rundir)

    if args.mode in ("run", "both"):
        execute_scripts(rundir)

    if args.mode == "results":
        collect_results(rundir)


if __name__ == "__main__":
    main()
