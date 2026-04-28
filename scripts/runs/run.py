import argparse
import csv
import os
import subprocess
import sys
import textwrap
from pathlib import Path


# --- Config ---

TIME_LIMIT = 900
THREADS = 4
SEED = 2122187

ROOT = Path(__file__).parent.parent.parent
DEFAULT_RUNS_DIR = Path.home() / "jobs"
EXE_LOCATION = ROOT / "code" / "build"
HPLUS = EXE_LOCATION / "hplus"

COMMAND_DEFAULTS = f"{HPLUS} --run --t={TIME_LIMIT} --threads={THREADS} --s={SEED}"


# --- Slurm config ---

SLURM_PARTITION = "arrow"
SLURM_MEM = "14GB"
SLURM_TIME = "00:20:00"
SLURM_CPUS = 4
SLURM_WCKEY = "rop"


# --- Templates ---

BASH_RUN_ALL_TEMPLATE = textwrap.dedent(
    """\
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
"""
)

SLURM_RUN_ALL_TEMPLATE = textwrap.dedent(
    """\
    #!/usr/bin/env python3
    import subprocess, sys, time, re
    from pathlib import Path

    def submit(job):
        r = subprocess.run(['sbatch', str(job)], capture_output=True, text=True)
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
"""
)

SLURM_JOB_TEMPLATE = textwrap.dedent(
    """\
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

    sudo cpupower frequency-set -g powersave
"""
)


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
            return f.read().splitlines()
    else:
        folder = Path(args.instances).absolute()
        return [str(folder / f) for f in os.listdir(args.instances)]


# --- Script writing ---


def write_bash_scripts(args, rundir):
    assert HPLUS.exists(), f"executable not found: {HPLUS}"

    if args.commands is not None:
        command = f"{COMMAND_DEFAULTS} {args.commands}"
    else:
        command = f"{COMMAND_DEFAULTS}"

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
            f"#!/bin/bash\n{command} --log={logsdir}/{instance}.log {inst} --cpxlog={cpxlogsdir}/{instance}.log\n"
        )
        job.chmod(0o755)
        job_paths.append(job)

    assert len(job_paths) > 0, "no instances found"

    run_all = Path(rundir) / "run_all.py"
    run_all.write_text(BASH_RUN_ALL_TEMPLATE)
    run_all.chmod(0o755)


def write_slurm_scripts(args, rundir):
    assert HPLUS.exists(), f"executable not found: {HPLUS}"

    if args.commands is not None:
        command = f"{COMMAND_DEFAULTS} {args.commands}"
    else:
        command = f"{COMMAND_DEFAULTS}"

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
                command=f"{command} --log={logsdir}/{instance}.log --cpxlog={cpxlogsdir}/{instance}.log{inst}",
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


def collect_results(rundir):
    sys.path.insert(0, str(Path(__file__).parent))
    from results import parse_log, FIELDNAMES

    assert Path(rundir).exists(), f"run directory not found: {rundir}"

    logsdir = Path(rundir) / "logs"
    assert logsdir.exists(), f"logs directory not found: {logsdir}"

    log_files = sorted(logsdir.glob("*.log"))
    assert len(log_files) > 0, f"no log files found in {logsdir}"

    run_name = Path(rundir).name
    csv_path = Path(rundir) / f"{run_name}.csv"

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        for i, log in enumerate(log_files, 1):
            print(f"[{i}/{len(log_files)}] {log.stem}")
            writer.writerow(parse_log(log))

    print(f"Results written to {csv_path}")


# --- Execution ---


def execute_scripts(rundir):
    assert Path(rundir).exists(), f"run directory not found: {rundir}"

    run_all = Path(rundir) / "run_all.py"
    assert run_all.exists(), f"run_all.py not found in {rundir}"

    subprocess.run(["python3", str(run_all)], check=True)


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
