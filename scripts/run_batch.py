"""
Script used to run a batch of instances on the UNIPD DEI Cluster through SLURM

Used for experimental evaluation of the 'hplus' solver

This script is compatible with hplus versions: 4.0.0
"""

import argparse
import random
import textwrap
import os
import sys
import csv
import subprocess
from pathlib import Path

TIME_LIMIT = 900
TIME_LIMIT_SLURM = "00:20:00"
THREADS = 4
SEED = 2122187

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNS_DIR = Path.home() / "jobs"
EXE_LOCATION = ROOT / "code" / "build"
HPLUS = EXE_LOCATION / "hplus"


SLURM_JOB_TEMPLATE = textwrap.dedent("""\
    #!/bin/bash
    #SBATCH --job-name={instance}
    #SBATCH --partition=arrow
    #SBATCH --cpus-per-task={threads}
    #SBATCH --mem=14GB
    #SBATCH --time={timelimit_slurm}
    #SBATCH --exclusive
    #SBATCH --wckey=rop
    #SBATCH --requeue
    #SBATCH --output={output_dir}/%x_%j.out
    #SBATCH --error={output_dir}/%x_%j.err
    #SBATCH --mail-type=NONE

    sudo cpupower frequency-set -g performance
    sleep 0.1
    stress-ng -c 4 --cpu-ops=100

    ulimit -v 16777216

    {command}

    sudo cpupower frequency-set -g powersave
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

BASH_RUN_ALL_TEMPLATE = textwrap.dedent("""\
    #!/usr/bin/env python3
    import subprocess, sys
    from pathlib import Path

    jobs = sorted((Path(__file__).parent / 'jobs').glob('*.sh'))
    n = len(jobs)
    failed = []

    for i, job in enumerate(jobs, 1):
        instance = job.stem
        print(f'[{{i}}/{{n}}] {{instance}}', flush=True)
        with open(f'{output_dir}/{{instance}}.log', 'w') as out:
            r = subprocess.run(['bash', str(job)], stdout=out, stderr=subprocess.STDOUT)
        if r.returncode != 0:
            failed.append(job.name)

    if failed:
        print(f'Failed: {{failed}}', file=sys.stderr)
        sys.exit(1)
""")


def parse():
    parser = argparse.ArgumentParser(
        description="Manage hplus benchmark runs.",
        epilog="Any unrecognised flags are forwarded verbatim to hplus.",
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
    parser.add_argument(
        "--exe",
        type=str,
        default=None,
        help=f"path to the hplus executable (default: {HPLUS})",
    )
    parser.add_argument("--bash", action="store_true")
    parser.add_argument(
        "--mode",
        choices=["write", "run", "results"],
        default="write",
        help="write: only create scripts (default) | run: only execute existing | results: runs the result script on a specified batch",
    )
    args, extra = parser.parse_known_args()
    args.commands = " ".join(extra) if extra else None
    args.exe = Path(args.exe).expanduser().resolve() if args.exe else HPLUS
    return args


def build_command(args):
    timelimit = args.timelimit if args.timelimit is not None else TIME_LIMIT
    threads = args.threads if args.threads is not None else THREADS
    seed = args.seed if args.seed is not None else SEED
    base = f"{args.exe} -t {timelimit} -T {threads} -s {seed} -v"
    if args.commands is not None:
        return f"{base} {args.commands}"
    return base


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


def write_slurm_scripts(args, rundir):
    assert args.exe.exists(), f"executable not found: {args.exe}"

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
                threads=THREADS,
                timelimit_slurm=TIME_LIMIT_SLURM,
                output_dir=output_dir,
                command=f"{command} -i {inst} -o false -l {logsdir}/{instance}.log -c {cpxlogsdir}/{instance}.log",
            )
        )
        job.chmod(0o755)
        job_paths.append(job)

    assert len(job_paths) > 0, "no instances found"

    run_all = Path(rundir) / "run_all.py"
    run_all.write_text(SLURM_RUN_ALL_TEMPLATE)
    run_all.chmod(0o755)


def write_bash_scripts(args, rundir):
    assert args.exe.exists(), f"executable not found: {args.exe}"

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
            f"#!/bin/bash\n{command} -i {inst} -o false -l {logsdir}/{instance}.log -c {cpxlogsdir}/{instance}.log\n"
        )
        job.chmod(0o755)
        job_paths.append(job)

    assert len(job_paths) > 0, "no instances found"

    run_all = Path(rundir) / "run_all.py"
    run_all.write_text(
        BASH_RUN_ALL_TEMPLATE.format(
            output_dir=output_dir,
        )
    )
    run_all.chmod(0o755)


def write_scripts(args, rundir):

    if rundir.exists():
        print(f"{rundir} already exists")
        exit(1)

    rundir.mkdir(parents=True)

    if args.bash:
        write_bash_scripts(args, rundir)
    else:
        write_slurm_scripts(args, rundir)

    print(f"Written run scripts in {rundir}")


def execute_scripts(rundir):
    assert Path(rundir).exists(), f"run directory not found: {rundir}"

    run_all = Path(rundir) / "run_all.py"
    assert run_all.exists(), f"run_all.py not found in {rundir}"

    r = subprocess.run(["python3", str(run_all)])
    if r.returncode != 0:
        sys.exit(r.returncode)


def collect_results(rundir):
    sys.path.insert(0, str(Path(__file__).parent))
    from parse_results import parse_log, FIELDNAMES

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
            row = parse_log(log)
            if row is not None:
                writer.writerow(row)

    print(f"Results written to {csv_path}")


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
        assert not args.bash, f"--slurm has no effect with --mode {args.mode}"

    if args.mode == "write":
        write_scripts(args, rundir)

    if args.mode == "run":
        execute_scripts(rundir)

    if args.mode == "results":
        collect_results(rundir)


if __name__ == "__main__":
    main()
