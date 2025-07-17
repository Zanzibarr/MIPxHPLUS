# ONLY FOR CLUSTER USE
import sys

sys.dont_write_bytecode = True

import os, json, shutil
from pathlib import Path


def clear_dir(dir: Path):
    for item in dir.iterdir():
        if item.is_file() or item.is_symlink():
            item.unlink()
        elif item.is_dir():
            shutil.rmtree(item)


def main():
    # verify input correctness
    if len(sys.argv) <= 1 or sys.argv[1] in ["-h", "--h", "-help", "--help"]:
        print(
            f"Usage: >> python3 {os.path.basename(__file__)} <all parameters to pass to the execution>."
        )
        exit(0)

    # read execution parameters
    execution_parameters = " ".join(sys.argv[1:])
    example_parameters = (
        execution_parameters + " --t=900 --mem=16000 --v=3 --log=<instance_name>.log"
    )

    # verify parameters
    if (
        input(
            f"The execution will have the following structure:\n>> ./hplus --run <instance> {example_parameters}\nInsert y if it's correct: "
        )
        != "y"
    ):
        exit(0)

    code_dir = os.path.abspath("..")

    # Load instances configuration
    config_path = os.path.join(code_dir, "test/fast_run_config.json")
    if not os.path.exists(config_path):
        print(f"Error: Configuration file not found at {config_path}")
        print(
            "Please run setup_instances.py first to create the instances configuration"
        )
        exit(1)

    with open(config_path, "r") as f:
        config = json.load(f)

    instances_folder = config["instances_folder"]
    selected_instances = config["selected_instances"]

    print(
        f"Using {len(selected_instances)} pre-selected instances from {instances_folder}"
    )

    # organize batches
    batch_size = 1000
    n_batches = (
        1
        if len(selected_instances) == batch_size
        else (len(selected_instances) // batch_size) + 1
    )

    # jobs
    jobs_folder = os.path.join(code_dir, "test/jobs")
    jobs_outputs = os.path.join(code_dir, "test/jobs_output")

    exec_dir = os.path.join(code_dir, "build")

    print(f"Jobs folder: {jobs_folder}")
    print(f"Jobs output: {jobs_outputs}")
    print(f"Executable: {exec_dir}/hplus")
    if input("Check those paths.\nInsert y if it's all correct: ") != "y":
        exit(0)

    os.makedirs(jobs_folder, exist_ok=True)
    os.makedirs(jobs_outputs, exist_ok=True)
    clear_dir(Path(jobs_outputs))

    for i in range(n_batches):
        batch_path = f"{jobs_folder}/batch_{i}"
        os.makedirs(batch_path, exist_ok=True)
        clear_dir(Path(batch_path))

        batch_start = i * batch_size

        for j in range(min(batch_size, len(selected_instances) - batch_start)):
            idx = batch_start + j
            inst = selected_instances[idx]
            inst_name = inst.replace(".sas", "")
            inst_path = f"{instances_folder}/{inst}"
            job = f"{batch_path}/{inst.replace('.sas', '_job.sh')}"
            job_content = f"""#!/bin/bash
#SBATCH --job-name={inst.replace('.sas', '_matzan')}
#SBATCH --partition=arrow
#SBATCH --ntasks=1
#SBATCH --mem=14GB
#SBATCH --time=00:20:00
#SBATCH --output={jobs_outputs}/{inst_name}.out
#SBATCH --error={jobs_outputs}/{inst_name}.out

# warm up processors
sudo cpupower frequency-set -g performance
sleep 0.1
stress-ng -c 4 --cpu-ops=100
# set limits
ulimit -v 16777216

#####################

{exec_dir}/./hplus --run {inst_path} {execution_parameters} --t=900 --mem=16000 --v=3 --log={inst_name}.log

#####################

# back to power saving mode
sudo cpupower frequency-set -g powersave"""

            with open(job, "w") as f:
                f.write(job_content)

    print(
        f"Created {n_batches} batches with jobs for {len(selected_instances)} instances"
    )
    print("You can now use the original run.py script to execute the batches")


if __name__ == "__main__":
    main()
